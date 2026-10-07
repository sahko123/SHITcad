// App.h first: windows.h defines `near` and `far` as empty macros, which breaks
// headers that use those names (Viewport3D.h's makePerspective).
#include "App.h"
#include "Utf8Path.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>
#undef near
#undef far

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace shitcad {

// ---- engine settings ------------------------------------------------------------

static fs::path engineSettingsPath() {
    const char* appdata = std::getenv("APPDATA");
    return fs::path(appdata ? appdata : ".") / "SHITcad" / "simulation.json";
}

void App::loadEngineSettings() {
    simEngine_.loaded = true;
    std::ifstream in(engineSettingsPath());
    if (!in) return;
    try {
        json j = json::parse(in);
        simEngine_.cipSimPath = j.value("cipSimPath", simEngine_.cipSimPath);
        simEngine_.python = j.value("python", simEngine_.python);
        simEngine_.cfdCasesDir = j.value("cfdCasesDir", simEngine_.cfdCasesDir);
    } catch (...) {
        // A corrupt settings file just means asking again.
    }
}

void App::saveEngineSettings() {
    std::error_code ec;
    fs::create_directories(engineSettingsPath().parent_path(), ec);
    std::ofstream out(engineSettingsPath());
    out << json{{"cipSimPath", simEngine_.cipSimPath}, {"python", simEngine_.python},
                {"cfdCasesDir", simEngine_.cfdCasesDir}}.dump(2) << "\n";
}

std::string App::engineProblem() const {
    if (simEngine_.cipSimPath.empty()) return "Set the cip-sim folder under Engine.";
    std::error_code ec;
    if (!fs::is_regular_file(fsPath(simEngine_.cipSimPath) / "cipsim" / "cli.py", ec))
        return "No cipsim/cli.py in " + simEngine_.cipSimPath + " - is that the cip-sim folder?";
    if (simEngine_.python.empty()) return "Set the Python executable under Engine.";
    return {};
}

// ---- what a run depends on --------------------------------------------------------

// Size and modification time of a file, so a re-export is noticed. The spec
// records only the path, and cip-sim re-reads that path at run time: without
// this, re-exporting the vessel left the results silently describing geometry
// that no longer exists.
static std::string fileStamp(const std::string& path) {
    std::error_code ec;
    const auto size = fs::file_size(fsPath(path), ec);
    const auto when = fs::last_write_time(fsPath(path), ec);
    return path + "|" + (ec ? "missing" : std::to_string((unsigned long long)size) + "|" +
                              std::to_string((long long)when.time_since_epoch().count()));
}

bool App::runInputs(std::string& inputs, std::string& error) const {
    std::string spec;
    std::vector<std::string> warnings;
    if (!buildTier1Spec(simulation_, featureHistory_, spec, warnings, error)) return false;

    // Rays and bounces are passed on the command line, not written into the
    // spec, so comparing specs alone missed them: dragging "Splash bounces"
    // from 2 to 0 changes never-reached from 2.8% to 21.4% with no warning.
    std::string out = spec;
    out += "\n#rays=" + std::to_string(simulation_.rays);
    out += "\n#bounces=" + std::to_string(simulation_.bounces);
    for (const auto& f : featureHistory_.features()) {
        if (f.type != FeatureType::MeshImport) continue;
        out += "\n#stl=" + fileStamp(std::get<MeshImportFeatureData>(f.data).sourcePath);
    }
    inputs = std::move(out);
    return true;
}

void App::clearSimulationRun() {
    if (simRunner_.running()) simRunner_.cancel();
    releaseSimulationResults();
    simView_ = SimResultView{};
    simPhase_ = SimPhase::Idle;
    simSummary_ = RunSummary{};
    simRunLog_.clear();
    simRunError_.clear();
    simRunDir_.clear();
    simRunInputs_.clear();
    paraviewMessage_.clear();
}

// ---- running ----------------------------------------------------------------------

static std::string timestamp() {
    std::time_t now = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", std::localtime(&now));
    return buf;
}

// Runs sit next to the project when it has been saved, so results travel with
// it; otherwise in the temp folder.
std::string App::simRunBase() const {
    if (!currentFilePath_.empty()) {
        fs::path proj = fsPath(currentFilePath_);
        return utf8(proj.parent_path() / fsPath(utf8(proj.stem()) + "_sim"));
    }
    return utf8(fs::temp_directory_path() / "SHITcad_sim");
}

void App::startTier1Run() {
    if (simPhase_ == SimPhase::Running) return;
    simRunLog_.clear();
    simRunError_.clear();
    // A summary left over from a previous run must never be attributed to this
    // one: a cancelled run that had already printed its result could otherwise
    // be reported as this run's answer.
    simSummary_ = RunSummary{};

    std::string problem = engineProblem();
    if (!problem.empty()) {
        simRunError_ = problem;
        simPhase_ = SimPhase::Failed;
        return;
    }
    std::string spec, err;
    std::vector<std::string> warnings;
    if (!buildTier1Spec(simulation_, featureHistory_, spec, warnings, err)) {
        simRunError_ = err;
        simPhase_ = SimPhase::Failed;
        return;
    }
    for (const auto& w : warnings) simRunLog_.push_back("warning: " + w);
    std::string inputs;
    if (!runInputs(inputs, err)) {
        simRunError_ = err;
        simPhase_ = SimPhase::Failed;
        return;
    }

    fs::path dir = fsPath(simRunBase()) / ("tier1_" + timestamp());
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
        simRunError_ = "Could not create " + utf8(dir) + ": " + ec.message();
        simPhase_ = SimPhase::Failed;
        return;
    }
    fs::path specPath = dir / "spec.json";
    {
        std::ofstream out(specPath, std::ios::binary);
        out << spec << "\n";
        if (!out) {
            simRunError_ = "Could not write " + utf8(specPath);
            simPhase_ = SimPhase::Failed;
            return;
        }
    }

    std::vector<std::string> argv = {
        simEngine_.python, "-u", "-m", "cipsim.cli", "tier1",
        "--spec", utf8(specPath), "--out", utf8(dir),
        "--rays", std::to_string(simulation_.rays), "--bounces", std::to_string(simulation_.bounces)};
    if (!simRunner_.start(argv, simEngine_.cipSimPath, true, err)) {
        simRunError_ = err;
        simPhase_ = SimPhase::Failed;
        return;
    }
    simRunDir_ = utf8(dir);
    simRunInputs_ = inputs;
    simRunStart_ = nowSeconds();
    simPhase_ = SimPhase::Running;
}

void App::pollSimulationRun() {
    pollCfdCase();
    std::vector<std::string> lines;
    if (paraviewLauncher_.running()) {
        paraviewLauncher_.poll(lines);
        for (const auto& l : lines) {
            const std::string kind = eventKind(l);
            if (kind == "started") {
                json j = json::parse(l, nullptr, false);
                paraviewMessage_ = std::string("Opened in ") +
                    (j.is_object() && j.value("viewer", "") == "windows" ? "ParaView" : "WSL ParaView");
            } else if (kind == "error") {
                paraviewMessage_ = "ParaView: " + eventMessage(l);
            }
        }
        if (paraviewLauncher_.finished() && paraviewLauncher_.exitCode() != 0 && paraviewMessage_.empty())
            paraviewMessage_ = "ParaView launcher failed: " + paraviewLauncher_.stderrTail();
        lines.clear();
    }

    if (simPhase_ != SimPhase::Running) return;
    simRunner_.poll(lines);

    bool gotResult = false;
    RunSummary summary;
    std::string errors;
    for (const auto& l : lines) {
        const std::string kind = eventKind(l);
        if (kind == "progress" || kind == "started") {
            if (kind == "progress") simRunLog_.push_back(eventMessage(l));
        } else if (kind == "warning") {
            simRunLog_.push_back("warning: " + eventMessage(l));
        } else if (kind == "error") {
            errors += (errors.empty() ? "" : "\n") + eventMessage(l);
        } else if (kind == "result") {
            std::string perr;
            if (parseResultEvent(l, summary, perr)) gotResult = true;
            else errors += perr;
        } else if (kind.empty()) {
            // Not JSON: a native library wrote to stdout, or the line was
            // corrupted. Keep it rather than dropping it silently - it is
            // usually the only clue about what went wrong.
            simRunLog_.push_back("engine output: " + l.substr(0, 200));
        }
    }
    if (!errors.empty()) simRunError_ += (simRunError_.empty() ? "" : "\n") + errors;
    if (gotResult) simSummary_ = summary;

    if (!simRunner_.finished()) return;
    simRunEnd_ = nowSeconds();
    if (simRunner_.exitCode() == 0 && !simSummary_.viewerJson.empty()) {
        std::string err;
        if (loadSimulationResults(simSummary_, simRunInputs_, simRunDir_, err)) {
            simPhase_ = SimPhase::Done;
        } else {
            simRunError_ = err;
            simPhase_ = SimPhase::Failed;
        }
    } else {
        if (simRunError_.empty()) {
            // No error event: the engine itself failed to start (bad Python,
            // missing packages). stderr is the only record of why.
            const std::string& tail = simRunner_.stderrTail();
            simRunError_ = "The simulation engine exited with code " + std::to_string(simRunner_.exitCode()) +
                           (tail.empty() ? "." : ":\n" + tail.substr(tail.size() > 1500 ? tail.size() - 1500 : 0));
        }
        simPhase_ = SimPhase::Failed;
    }
    simSummary_ = RunSummary{};
}

// ---- results ------------------------------------------------------------------------

void App::releaseSimulationResults() {
    if (simView_.vao) glDeleteVertexArrays(1, &simView_.vao);
    if (simView_.vboGeom) glDeleteBuffers(1, &simView_.vboGeom);
    if (simView_.vboColour) glDeleteBuffers(1, &simView_.vboColour);
    simView_.vao = simView_.vboGeom = simView_.vboColour = 0;
    simView_.vertexCount = 0;
    simView_.loaded = false;
}

bool App::loadSimulationResults(const RunSummary& summary, const std::string& inputs,
                                const std::string& runDir, std::string& error) {
    ResultMesh mesh;
    if (!loadResultMesh(summary.viewerJson, mesh, error)) return false;

    releaseSimulationResults();
    simView_.mesh = std::move(mesh);
    simView_.summary = summary;
    simView_.inputs = inputs;
    // Which imported meshes these results replace on screen, by name. A mesh
    // imported after the run is not in this list and stays visible.
    simView_.coveredFeatures.clear();
    for (const auto& f : featureHistory_.features()) {
        if (f.type != FeatureType::MeshImport) continue;
        if (std::find(simView_.mesh.surfaceNames.begin(), simView_.mesh.surfaceNames.end(), f.name)
            != simView_.mesh.surfaceNames.end())
            simView_.coveredFeatures.push_back(f.id);
    }
    simView_.closed = trianglesAreClosed(simView_.mesh.positionsMm.data(), simView_.mesh.triangles);
    simView_.runDir = runDir;
    int reach = simView_.mesh.fieldIndex("reach");
    simView_.field = reach >= 0 ? reach : 0;
    // Park the (scene-wide) section plane in the middle of the new results
    // while it is off, so switching it on cuts somewhere useful.
    if (!section_.enabled)
        section_.position = (simView_.mesh.boundsMin[section_.axis] + simView_.mesh.boundsMax[section_.axis]) * 0.5f;

    // Geometry: position + flat normal per vertex, three vertices per triangle.
    const ResultMesh& m = simView_.mesh;
    std::vector<float> geom(m.triangles * 18);
    for (size_t t = 0; t < m.triangles; t++) {
        const float* p = &m.positionsMm[t * 9];
        const float e1[3] = {p[3] - p[0], p[4] - p[1], p[5] - p[2]};
        const float e2[3] = {p[6] - p[0], p[7] - p[1], p[8] - p[2]};
        float n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
        float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (len > 0) for (float& c : n) c /= len;
        for (int v = 0; v < 3; v++) {
            float* g = &geom[t * 18 + v * 6];
            g[0] = p[v * 3]; g[1] = p[v * 3 + 1]; g[2] = p[v * 3 + 2];
            g[3] = n[0]; g[4] = n[1]; g[5] = n[2];
        }
    }
    glGenVertexArrays(1, &simView_.vao);
    glGenBuffers(1, &simView_.vboGeom);
    glGenBuffers(1, &simView_.vboColour);
    glBindVertexArray(simView_.vao);
    glBindBuffer(GL_ARRAY_BUFFER, simView_.vboGeom);
    glBufferData(GL_ARRAY_BUFFER, geom.size() * sizeof(float), geom.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glBindBuffer(GL_ARRAY_BUFFER, simView_.vboColour);
    glBufferData(GL_ARRAY_BUFFER, m.triangles * 9 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glBindVertexArray(0);

    simView_.vertexCount = (int)(m.triangles * 3);
    simView_.colourDirty = true;
    simView_.loaded = true;
    simView_.show = true;
    return true;
}

// Shader sources live in SimResults.cpp so the tests can compile them.


void App::renderSimulationResults(const float* view, const float* proj, const float* eyePos) {
    if (workspace_ != Workspace::Simulation || !simView_.loaded || !simView_.show) return;
    if (!resultShader_.id() && !resultShader_.compile(kResultVertSrc, kResultFragSrc)) {
        simView_.show = false; // compile errors are printed by ShaderProgram
        return;
    }
    if (simView_.colourDirty) {
        std::vector<float> perTri, perVertex(simView_.mesh.triangles * 9);
        colourByField(simView_.mesh, simView_.field, perTri);
        for (size_t t = 0; t < simView_.mesh.triangles; t++)
            for (int v = 0; v < 3; v++)
                for (int k = 0; k < 3; k++) perVertex[t * 9 + v * 3 + k] = perTri[t * 3 + k];
        glBindBuffer(GL_ARRAY_BUFFER, simView_.vboColour);
        glBufferSubData(GL_ARRAY_BUFFER, 0, perVertex.size() * sizeof(float), perVertex.data());
        simView_.colourDirty = false;
    }

    resultShader_.use();
    resultShader_.setMat4("uView", view);
    resultShader_.setMat4("uProj", proj);
    resultShader_.setVec3("uEyePos", eyePos[0], eyePos[1], eyePos[2]);
    resultShader_.setVec3("uLightDir", 0.3f, 0.8f, 0.5f);
    applyClip(resultShader_, &section_);

    glBindVertexArray(simView_.vao);
    glDrawArrays(GL_TRIANGLES, 0, simView_.vertexCount);
    glBindVertexArray(0);
}

void App::openResultsInParaView() {
    if (!simView_.loaded || simView_.summary.vtp.empty()) return;
    if (paraviewLauncher_.running()) return;
    std::string problem = engineProblem();
    if (!problem.empty()) { paraviewMessage_ = problem; return; }
    std::string err;
    paraviewMessage_ = "Opening ParaView...";
    if (!paraviewLauncher_.start({simEngine_.python, "-u", "-m", "cipsim.cli", "paraview", simView_.summary.vtp},
                                 simEngine_.cipSimPath, false, err))
        paraviewMessage_ = err;
}

// ---- panel sections -------------------------------------------------------------------

App::SimRunModel App::simRunModel() {
    if (!simEngine_.loaded) loadEngineSettings();
    SimRunModel m;
    m.cipSimPath = simEngine_.cipSimPath;
    m.python = simEngine_.python;
    m.problem = engineProblem();
    m.running = simPhase_ == SimPhase::Running;
    m.done = simPhase_ == SimPhase::Done;
    m.cancelled = simPhase_ == SimPhase::Cancelled;
    m.seconds = m.running ? nowSeconds() - simRunStart_ : simRunEnd_ - simRunStart_;
    if (!m.running) m.error = simRunError_;
    m.log = simRunLog_;
    return m;
}

void App::setEnginePaths(const std::string& cipSimPath, const std::string& python) {
    if (cipSimPath == simEngine_.cipSimPath && python == simEngine_.python) return;
    simEngine_.cipSimPath = cipSimPath;
    simEngine_.python = python;
    saveEngineSettings();
}

void App::browseEngineFolder() {
    std::string p = chooseFile(FileDialog::PickFolder, "Select the cip-sim folder (contains cipsim\\cli.py)");
    if (!p.empty()) { simEngine_.cipSimPath = p; saveEngineSettings(); }
}

void App::cancelSimulationRun() {
    if (simPhase_ != SimPhase::Running) return;
    simRunner_.cancel();
    simPhase_ = SimPhase::Cancelled;
    // The engine may already have printed its result before the kill landed;
    // that answer belongs to no run now.
    simSummary_ = RunSummary{};
}

static const char* fieldLabel(const std::string& name) {
    if (name == "reach") return "Coverage";
    if (name == "total_flux") return "Total flux";
    if (name == "direct_flux") return "Direct flux";
    if (name == "bounce_flux") return "Splash flux";
    if (name == "incidence_deg") return "Impact angle";
    if (name == "surface_id") return "Surface";
    return name.c_str();
}

App::SimResultsModel App::simResultsModel() const {
    SimResultsModel m;
    m.loaded = simView_.loaded;
    if (!m.loaded) return m;
    // Stale: anything the run depended on has changed - the spec, the run
    // settings, or an STL on disk.
    std::string now, err;
    m.stale = !(runInputs(now, err) && now == simView_.inputs);
    m.show = simView_.show;
    m.field = simView_.field;
    const ResultMesh& mesh = simView_.mesh;
    for (const auto& f : mesh.fields) m.fieldLabels.push_back(fieldLabel(f.name));
    if (simView_.field < (int)mesh.fields.size()) {
        const ResultField& f = mesh.fields[simView_.field];
        m.categorical = f.categorical;
        if (f.categorical) {
            for (int k = 0; k < (int)std::max(f.labels.size(), f.colours.size()); k++) {
                SimResultsModel::Swatch s;
                categoryColour(f, k, s.rgb);
                s.label = k < (int)f.labels.size() ? f.labels[k] : std::to_string(k);
                if (f.name == "reach") {
                    const auto& o = simView_.summary.overall;
                    const double pct = k == 0 ? o.dryPct : k == 1 ? o.splashPct : o.directPct;
                    char buf[64];
                    snprintf(buf, sizeof(buf), "  %.1f%%", pct);
                    s.label += buf;
                }
                m.categories.push_back(std::move(s));
            }
            m.reachNote = f.name == "reach" && (int)m.categories.size() > 2;
        } else {
            m.hi = f.p95 > 0 ? f.p95 : f.maxValue;
            m.unit = f.unit;
            m.flux = f.name.find("flux") != std::string::npos;
            m.fluxTrustworthy = simView_.summary.fluxTrustworthy;
            m.noData = f.noData;
        }
    }
    m.overall = simView_.summary.overall;
    m.surfaces = simView_.summary.surfaces;
    m.paraviewMessage = paraviewMessage_;
    return m;
}

void App::setResultsField(int field) {
    if (field < 0 || field >= (int)simView_.mesh.fields.size()) return;
    simView_.field = field;
    simView_.colourDirty = true;
}

// ---- CFD case ----------------------------------------------------------------------

App::CfdCaseModel App::cfdCaseModel() {
    if (!simEngine_.loaded) loadEngineSettings();
    CfdCaseModel m;
    m.casesDir = simEngine_.cfdCasesDir;
    m.mesh = cfd_.mesh;
    m.problem = engineProblem();
    if (m.problem.empty() && simEngine_.cfdCasesDir.empty()) m.problem = "Set the folder CFD cases go in.";
    m.running = cfd_.phase == SimPhase::Running;
    m.done = cfd_.phase == SimPhase::Done;
    m.cancelled = cfd_.phase == SimPhase::Cancelled;
    m.seconds = m.running ? nowSeconds() - cfd_.start : cfd_.end - cfd_.start;
    m.status = cfd_.status;
    if (!m.running) m.error = cfd_.error;
    m.warnings = cfd_.warnings;
    m.caseDir = cfd_.caseDir;
    m.casePosix = cfd_.casePosix;
    m.summary = cfd_.summary;
    return m;
}

void App::setCfdCasesDir(const std::string& dir) {
    if (dir == simEngine_.cfdCasesDir) return;
    simEngine_.cfdCasesDir = dir;
    saveEngineSettings();
}

void App::startCfdCase() {
    if (cfd_.phase == SimPhase::Running) return;
    const bool mesh = cfd_.mesh;
    cfd_ = CfdCaseState{};
    cfd_.mesh = mesh;
    auto fail = [this](const std::string& why) {
        cfd_.error = why;
        cfd_.phase = SimPhase::Failed;
    };
    if (!simEngine_.loaded) loadEngineSettings();
    std::string problem = engineProblem();
    if (!problem.empty()) return fail(problem);
    // The same spec Tier 1 runs on, roles included: what the generator turns
    // into patches and boundary conditions.
    std::string spec, err;
    std::vector<std::string> warnings;
    if (!buildTier1Spec(simulation_, featureHistory_, spec, warnings, err)) return fail(err);
    cfd_.warnings = warnings;

    const std::string stamp = timestamp();
    fs::path dir = fsPath(simRunBase()) / ("cfd_" + stamp);
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) return fail("Could not create " + utf8(dir) + ": " + ec.message());
    fs::path specPath = dir / "spec.json";
    {
        std::ofstream out(specPath, std::ios::binary);
        out << spec << "\n";
        if (!out) return fail("Could not write " + utf8(specPath));
    }

    // A case per press, named after the project: OpenFOAM writes its results
    // into the case, so an earlier one is never overwritten.
    std::string name;
    const std::string stem = currentFilePath_.empty() ? std::string("untitled") : utf8(fsPath(currentFilePath_).stem());
    for (char c : stem) name += (std::isalnum((unsigned char)c) || c == '-' || c == '_') ? c : '_';
    std::string casesDir = simEngine_.cfdCasesDir;
    while (casesDir.size() > 1 && casesDir.back() == '/') casesDir.pop_back();
    const std::string out = casesDir + "/" + name + "_" + stamp;

    std::vector<std::string> argv = {simEngine_.python, "-u", "-m", "cipsim.cli", "case",
                                     "--spec", utf8(specPath), "--out", out};
    if (mesh) argv.push_back("--mesh");
    if (!cfdRunner_.start(argv, simEngine_.cipSimPath, true, err)) return fail(err);
    cfd_.status = "Writing the case...";
    cfd_.start = nowSeconds();
    cfd_.phase = SimPhase::Running;
}

void App::pollCfdCase() {
    if (cfd_.phase != SimPhase::Running) return;
    std::vector<std::string> lines;
    cfdRunner_.poll(lines);
    std::string errors;
    for (const auto& l : lines) {
        const std::string kind = eventKind(l);
        if (kind == "progress") {
            cfd_.status = eventMessage(l);
        } else if (kind == "warning") {
            cfd_.warnings.push_back(eventMessage(l));
        } else if (kind == "error") {
            errors += (errors.empty() ? "" : "\n") + eventMessage(l);
        } else if (kind == "result") {
            json j = json::parse(l, nullptr, false);
            if (!j.is_object()) continue;
            // value() throws on a null, and case_posix IS null for a case
            // written to a Windows folder: read the strings defensively.
            auto str = [&j](const char* k) { return j.contains(k) && j[k].is_string() ? j[k].get<std::string>() : std::string(); };
            cfd_.caseDir = str("case");
            cfd_.casePosix = str("case_posix");
            std::string s;
            if (j.contains("patches") && j["patches"].is_object()) {
                for (const auto& [patch, role] : j["patches"].items())
                    s += (s.empty() ? "" : ", ") + patch + " (" + (role.is_string() ? role.get<std::string>() : "?") + ")";
                s = std::to_string(j["patches"].size()) + " surfaces: " + s;
            }
            if (j.contains("mesh") && j["mesh"].is_object()) {
                const auto& m = j["mesh"];
                const long long cells = m.contains("cells") && m["cells"].is_number() ? m["cells"].get<long long>() : 0;
                s += "\nMeshed: " + std::to_string(cells) + " cells, checkMesh " +
                     (m.contains("check_mesh_ok") && m["check_mesh_ok"].is_boolean() && m["check_mesh_ok"].get<bool>()
                          ? "OK" : "reported problems - see log.checkMesh");
            } else {
                s += "\nNot meshed: run ./Allmesh in the case.";
            }
            cfd_.summary = s;
        } else if (kind.empty()) {
            cfd_.warnings.push_back("engine output: " + l.substr(0, 200));
        }
    }
    if (!errors.empty()) cfd_.error += (cfd_.error.empty() ? "" : "\n") + errors;
    if (!cfdRunner_.finished()) return;
    cfd_.end = nowSeconds();
    if (cfdRunner_.exitCode() == 0 && !cfd_.caseDir.empty()) {
        cfd_.phase = SimPhase::Done;
    } else {
        if (cfd_.error.empty()) {
            const std::string& tail = cfdRunner_.stderrTail();
            cfd_.error = "The case generator exited with code " + std::to_string(cfdRunner_.exitCode()) +
                         (tail.empty() ? "." : ":\n" + tail.substr(tail.size() > 1500 ? tail.size() - 1500 : 0));
        }
        cfd_.phase = SimPhase::Failed;
    }
}

void App::cancelCfdCase() {
    if (cfd_.phase != SimPhase::Running) return;
    cfdRunner_.cancel();
    cfd_.phase = SimPhase::Cancelled;
    cfd_.end = nowSeconds();
    // Killing the Windows side does not stop a mesher already started inside
    // WSL; say so rather than imply the case folder is clean.
    cfd_.status = "Cancelled. A mesher already running inside WSL finishes on its own; delete the case folder if unwanted.";
}

void App::openCfdCaseFolder() {
    if (cfd_.caseDir.empty()) return;
    ShellExecuteW(nullptr, L"open", fsPath(cfd_.caseDir).c_str(), nullptr, nullptr, 1 /* SW_SHOWNORMAL */);
}

void App::openRunFolder() {
    if (simView_.runDir.empty()) return;
    ShellExecuteW(nullptr, L"open", fsPath(simView_.runDir).c_str(), nullptr, nullptr, 1 /* SW_SHOWNORMAL */);
}

} // namespace shitcad
