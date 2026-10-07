#include "Simulation.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>

using json = nlohmann::json;

namespace shitcad {

// ---- roles --------------------------------------------------------------------

const char* surfaceRoleName(SurfaceRole role) {
    switch (role) {
        case SurfaceRole::Wall:         return "wall";
        case SurfaceRole::InternalWall: return "internal_wall";
        case SurfaceRole::Inlet:        return "inlet";
        case SurfaceRole::Drain:        return "drain";
        case SurfaceRole::Obstruction:  return "obstruction";
    }
    return "wall";
}

const char* surfaceRoleLabel(SurfaceRole role) {
    switch (role) {
        case SurfaceRole::Wall:         return "Vessel wall";
        case SurfaceRole::InternalWall: return "Internal (pipe, baffle)";
        case SurfaceRole::Inlet:        return "Inlet cap";
        case SurfaceRole::Drain:        return "Drain cap";
        case SurfaceRole::Obstruction:  return "Obstruction (not cleaned)";
    }
    return "Vessel wall";
}

bool surfaceRoleScored(SurfaceRole role) {
    return role == SurfaceRole::Wall || role == SurfaceRole::InternalWall;
}

bool surfaceRoleFromName(const std::string& name, SurfaceRole& out) {
    for (int i = 0; i < kSurfaceRoleCount; i++) {
        if (name == surfaceRoleName((SurfaceRole)i)) { out = (SurfaceRole)i; return true; }
    }
    return false;
}

// ---- set-up -------------------------------------------------------------------

SurfaceRole SimulationSetup::roleFor(FeatureID f) const {
    for (const auto& r : roles) if (r.meshFeature == f) return r.role;
    return SurfaceRole::Wall;
}

void SimulationSetup::setRole(FeatureID f, SurfaceRole role) {
    for (auto& r : roles) {
        if (r.meshFeature == f) { r.role = role; return; }
    }
    roles.push_back({f, role});
}

SimNozzle* SimulationSetup::findNozzle(uint32_t id) {
    for (auto& n : nozzles) if (n.id == id) return &n;
    return nullptr;
}

const SimNozzle* SimulationSetup::findNozzle(uint32_t id) const {
    for (const auto& n : nozzles) if (n.id == id) return &n;
    return nullptr;
}

uint32_t SimulationSetup::addNozzle(SimNozzle n) {
    n.id = nextNozzleID++;
    if (n.name.empty()) n.name = "nozzle" + std::to_string(n.id);
    nozzles.push_back(n);
    return n.id;
}

void SimulationSetup::removeNozzle(uint32_t id) {
    nozzles.erase(std::remove_if(nozzles.begin(), nozzles.end(),
                                 [id](const SimNozzle& n) { return n.id == id; }),
                  nozzles.end());
}

bool sameSimulation(const SimulationSetup& a, const SimulationSetup& b) {
    json ja, jb;
    simulationToJson(a, ja);
    simulationToJson(b, jb);
    return ja == jb;
}

// ---- frames -------------------------------------------------------------------

static const MeshImportFeatureData* hostData(const FeatureHistory& h, FeatureID id) {
    const Feature* f = h.findFeature(id);
    if (!f || f->type != FeatureType::MeshImport) return nullptr;
    return &std::get<MeshImportFeatureData>(f->data);
}

// Is this surface actually going into the spec? Suppressed, rolled back and
// errored surfaces are left out, and a nozzle anchored to one would otherwise
// keep firing from where that surface used to be.
static bool featureIsActive(const FeatureHistory& h, FeatureID id) {
    const auto& feats = h.features();
    for (int i = 0; i < (int)feats.size(); i++) {
        if (feats[i].id != id) continue;
        return !feats[i].suppressed && !h.isRolledBack(i) && !feats[i].hasError;
    }
    return false;
}

static void normalise(double v[3]) {
    double m = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (m > 1e-12) { v[0] /= m; v[1] /= m; v[2] /= m; }
}

bool nozzleWorld(const SimNozzle& n, const FeatureHistory& history, double pos[3], double axis[3]) {
    const MeshImportFeatureData* host = n.hostFeature ? hostData(history, n.hostFeature) : nullptr;
    if (n.hostFeature && !host) return false;
    if (!host) {
        for (int i = 0; i < 3; i++) { pos[i] = n.position[i]; axis[i] = n.axis[i]; }
    } else {
        const double* r = host->transform.r;
        host->transform.apply(n.position, pos);
        for (int i = 0; i < 3; i++)
            axis[i] = r[i * 3] * n.axis[0] + r[i * 3 + 1] * n.axis[1] + r[i * 3 + 2] * n.axis[2];
    }
    normalise(axis);
    return true;
}

void setNozzleWorld(SimNozzle& n, const FeatureHistory& history, const double pos[3], const double axis[3]) {
    const MeshImportFeatureData* host = n.hostFeature ? hostData(history, n.hostFeature) : nullptr;
    double a[3] = {axis[0], axis[1], axis[2]};
    normalise(a);
    if (!host) {
        n.hostFeature = NullFeatureID; // host gone: keep the nozzle where it is, in world
        for (int i = 0; i < 3; i++) { n.position[i] = pos[i]; n.axis[i] = a[i]; }
        return;
    }
    // Inverse of p = R q + t is q = R^T (p - t); R is orthonormal.
    const double* r = host->transform.r;
    const double d[3] = {pos[0] - host->transform.t[0], pos[1] - host->transform.t[1],
                         pos[2] - host->transform.t[2]};
    for (int i = 0; i < 3; i++) {
        n.position[i] = r[0 + i] * d[0] + r[3 + i] * d[1] + r[6 + i] * d[2];
        n.axis[i] = r[0 + i] * a[0] + r[3 + i] * a[1] + r[6 + i] * a[2];
    }
}

// ---- spec export ----------------------------------------------------------------

// Nozzle parameters are float in the UI; widening to double for JSON would
// write 0.3 as 0.30000001192092896. Round to what the float actually resolves.
static double tidy(float v) { return std::round((double)v * 1e6) / 1e6; }

static std::string posixPath(std::string p) {
    std::replace(p.begin(), p.end(), '\\', '/');
    return p;
}

static bool specUnit(const std::string& unit) {
    // Units cip-sim's spec reader accepts (cipsim/spec.py LENGTH_TO_M).
    static const std::set<std::string> ok = {"m", "dm", "cm", "mm", "um", "in", "ft"};
    return ok.count(unit) > 0;
}

bool buildTier1Spec(const SimulationSetup& sim, const FeatureHistory& history,
                    std::string& outJson, std::vector<std::string>& warnings, std::string& error) {
    json doc;
    doc["version"] = 1;
    doc["units"] = "mm";
    // Declared, not converted: the spec is in this app's frame, and consumers
    // that care about gravity read it from here.
    doc["up"] = {0.0, 1.0, 0.0};
    doc["meta"] = {{"produced_by", "SHITcad"}, {"frame", "SHITcad world, mm, Y up"}};

    json surfaces = json::array();
    std::set<std::string> names;
    int scored = 0;
    const auto& feats = history.features();
    for (int i = 0; i < (int)feats.size(); i++) {
        const Feature& f = feats[i];
        if (f.type != FeatureType::MeshImport) continue;
        if (f.suppressed || history.isRolledBack(i)) {
            warnings.push_back("'" + f.name + "' is suppressed or rolled back and is left out");
            continue;
        }
        if (f.hasError) {
            warnings.push_back("'" + f.name + "' has an error (" + f.errorMsg + ") and is left out");
            continue;
        }
        const auto& md = std::get<MeshImportFeatureData>(f.data);
        if (!specUnit(md.unit)) {
            error = "'" + f.name + "' uses unit '" + md.unit +
                    "', which the simulation cannot read. Change it in Rotate / Move.";
            return false;
        }
        if (!names.insert(f.name).second) {
            error = "Two surfaces are named '" + f.name +
                    "'. Rename one (right-click it in the timeline): results are reported per name.";
            return false;
        }
        SurfaceRole role = sim.roleFor(f.id);
        scored += surfaceRoleScored(role) ? 1 : 0;

        json s;
        s["name"] = f.name;
        s["file"] = posixPath(md.sourcePath);
        s["units"] = md.unit;
        s["scored"] = surfaceRoleScored(role);
        // What the surface is, for CFD case generation (cip-sim spec `role`).
        s["role"] = surfaceRoleName(role);
        if (!md.transform.isIdentity()) {
            // Spec semantics match MeshTransform exactly: rotation applied to the
            // unit-scaled file coordinates, translation in the spec's units (mm).
            s["transform"] = {
                {"rotation", std::vector<double>(md.transform.r, md.transform.r + 9)},
                {"translation", std::vector<double>(md.transform.t, md.transform.t + 3)},
            };
        }
        if (!md.skins.keep.empty()) {
            // One point ON each kept skin, so cip-sim picks the same skins
            // whatever its nearest-skin rule does with a point between two.
            // The stored points may sit a little off a re-exported file.
            std::vector<int> kept;
            std::string err;
            MeshFileInfo info;
            if (!resolveSkins(md.sourcePath, md.skins.keep, kept, err) || !probeMeshFile(md.sourcePath, info, err)) {
                error = "'" + f.name + "': " + err;
                return false;
            }
            std::sort(kept.begin(), kept.end());
            kept.erase(std::unique(kept.begin(), kept.end()), kept.end());
            json pts = json::array();
            for (int k : kept) {
                const double* p = info.skins[k].point;
                pts.push_back({p[0], p[1], p[2]});
            }
            s["skins"] = pts;
        }
        s["meta"] = {{"role", surfaceRoleName(role)}, {"shitcad_feature_id", f.id}};
        surfaces.push_back(s);
    }

    if (surfaces.empty()) {
        error = "No surfaces. Import the vessel STL first.";
        return false;
    }
    if (scored == 0) {
        error = "No surface is a wall to be cleaned. Set at least one surface to Vessel wall or Internal.";
        return false;
    }

    json nozzles = json::array();
    for (const auto& n : sim.nozzles) {
        double pos[3], axis[3];
        if (!nozzleWorld(n, history, pos, axis)) {
            error = "Nozzle '" + n.name + "' was placed on a surface that has been deleted. Move or delete it.";
            return false;
        }
        if (n.hostFeature != NullFeatureID && !featureIsActive(history, n.hostFeature)) {
            error = "Nozzle '" + n.name + "' sits on a surface that is left out of this run "
                    "(suppressed, rolled back, or failed to load). Unsuppress it, or delete the nozzle.";
            return false;
        }
        if (!(n.halfAngleDeg > 0.0f && n.halfAngleDeg <= 180.0f)) {
            error = "Nozzle '" + n.name + "': spray half-angle must be between 0 and 180 degrees.";
            return false;
        }
        if (!(n.mdotKgS > 0.0f)) {
            error = "Nozzle '" + n.name + "': flow rate must be positive.";
            return false;
        }
        nozzles.push_back({
            {"name", n.name},
            {"position", {pos[0], pos[1], pos[2]}},
            {"axis", {axis[0], axis[1], axis[2]}},
            {"half_angle_deg", tidy(n.halfAngleDeg)},
            {"mdot_kg_s", tidy(n.mdotKgS)},
            {"pressure_bar", tidy(n.pressureBar)},
            {"meta", {{"shitcad_nozzle_id", n.id}}},
        });
    }
    if (nozzles.empty()) {
        error = "No nozzles. Use Place nozzle and click on a surface.";
        return false;
    }

    doc["surfaces"] = surfaces;
    doc["nozzles"] = nozzles;
    // `replace` rather than the default throw: this runs every frame
    // (the stale-results check), and a path byte that is not valid UTF-8 would
    // otherwise terminate the app rather than show an error.
    outJson = doc.dump(2, ' ', false, json::error_handler_t::replace);
    return true;
}

// ---- project file -----------------------------------------------------------------

void simulationToJson(const SimulationSetup& sim, json& out) {
    out = json::object();
    out["rays"] = sim.rays;
    out["bounces"] = sim.bounces;
    out["nextNozzleID"] = sim.nextNozzleID;
    out["roles"] = json::array();
    for (const auto& r : sim.roles)
        out["roles"].push_back({{"feature", r.meshFeature}, {"role", surfaceRoleName(r.role)}});
    out["nozzles"] = json::array();
    for (const auto& n : sim.nozzles) {
        out["nozzles"].push_back({
            {"id", n.id}, {"name", n.name}, {"hostFeature", n.hostFeature},
            {"position", {n.position[0], n.position[1], n.position[2]}},
            {"axis", {n.axis[0], n.axis[1], n.axis[2]}},
            {"halfAngleDeg", n.halfAngleDeg}, {"mdotKgS", n.mdotKgS},
            {"pressureBar", n.pressureBar},
        });
    }
}

bool simulationFromJson(const json& in, SimulationSetup& out, std::string& error) {
    try {
        SimulationSetup s;
        s.rays = in.value("rays", s.rays);
        s.bounces = in.value("bounces", s.bounces);
        s.nextNozzleID = in.value("nextNozzleID", s.nextNozzleID);
        for (const auto& r : in.value("roles", json::array())) {
            SimSurfaceRole sr;
            sr.meshFeature = r.at("feature").get<FeatureID>();
            if (!surfaceRoleFromName(r.at("role").get<std::string>(), sr.role)) {
                error = "unknown surface role '" + r.at("role").get<std::string>() + "'";
                return false;
            }
            s.roles.push_back(sr);
        }
        for (const auto& j : in.value("nozzles", json::array())) {
            SimNozzle n;
            n.id = j.at("id").get<uint32_t>();
            n.name = j.at("name").get<std::string>();
            n.hostFeature = j.value("hostFeature", (FeatureID)NullFeatureID);
            auto p = j.at("position").get<std::vector<double>>();
            auto a = j.at("axis").get<std::vector<double>>();
            if (p.size() != 3 || a.size() != 3) {
                error = "nozzle '" + n.name + "' position/axis must have 3 values";
                return false;
            }
            for (int i = 0; i < 3; i++) { n.position[i] = p[i]; n.axis[i] = a[i]; }
            n.halfAngleDeg = j.value("halfAngleDeg", n.halfAngleDeg);
            n.mdotKgS = j.value("mdotKgS", n.mdotKgS);
            n.pressureBar = j.value("pressureBar", n.pressureBar);
            s.nextNozzleID = std::max(s.nextNozzleID, n.id + 1);
            s.nozzles.push_back(n);
        }
        out = std::move(s);
        return true;
    } catch (const json::exception& e) {
        error = std::string("invalid simulation block: ") + e.what();
        return false;
    }
}

} // namespace shitcad
