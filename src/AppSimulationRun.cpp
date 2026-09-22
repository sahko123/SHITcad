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

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <nlohmann/json.hpp>

#include <algorithm>
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
    } catch (...) {
        // A corrupt settings file just means asking again.
    }
}

void App::saveEngineSettings() {
    std::error_code ec;
    fs::create_directories(engineSettingsPath().parent_path(), ec);
    std::ofstream out(engineSettingsPath());
    out << json{{"cipSimPath", simEngine_.cipSimPath}, {"python", simEngine_.python}}.dump(2) << "\n";
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

    // Runs sit next to the project when it has been saved, so results travel
    // with it; otherwise in the temp folder.
    fs::path base;
    if (!currentFilePath_.empty()) {
        fs::path proj = fsPath(currentFilePath_);
        base = proj.parent_path() / fsPath(utf8(proj.stem()) + "_sim");
    } else {
        base = fs::temp_directory_path() / "SHITcad_sim";
    }
    fs::path dir = base / ("tier1_" + timestamp());
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
    simRunStart_ = glfwGetTime();
    simPhase_ = SimPhase::Running;
}

void App::pollSimulationRun() {
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
    simRunEnd_ = glfwGetTime();
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

void App::drawSimulationRunSection() {
    if (!simEngine_.loaded) loadEngineSettings();

    if (ImGui::CollapsingHeader("Run", ImGuiTreeNodeFlags_DefaultOpen)) {
        std::string problem = engineProblem();
        if (ImGui::TreeNodeEx("Engine (cip-sim)", problem.empty() ? 0 : ImGuiTreeNodeFlags_DefaultOpen)) {
            static char pathBuf[512];
            static char pyBuf[260];
            if (!ImGui::IsAnyItemActive()) {
                snprintf(pathBuf, sizeof(pathBuf), "%s", simEngine_.cipSimPath.c_str());
                snprintf(pyBuf, sizeof(pyBuf), "%s", simEngine_.python.c_str());
            }
            ImGui::TextUnformatted("cip-sim folder");
            ImGui::SetNextItemWidth(-70);
            ImGui::InputText("##cipsim", pathBuf, sizeof(pathBuf));
            if (ImGui::IsItemDeactivatedAfterEdit()) { simEngine_.cipSimPath = pathBuf; saveEngineSettings(); }
            ImGui::SameLine();
            if (ImGui::Button("Browse")) {
                std::string p = openNativeFolderDialog("Select the cip-sim folder (contains cipsim\\cli.py)");
                if (!p.empty()) { simEngine_.cipSimPath = p; saveEngineSettings(); }
            }
            ImGui::TextUnformatted("Python");
            ImGui::SetNextItemWidth(-1);
            ImGui::InputText("##python", pyBuf, sizeof(pyBuf));
            if (ImGui::IsItemDeactivatedAfterEdit()) { simEngine_.python = pyBuf; saveEngineSettings(); }
            if (!problem.empty()) ImGui::TextColored({1, 0.8f, 0.3f, 1}, "%s", problem.c_str());
            else ImGui::TextDisabled("Saved for this computer, not in the project.");
            ImGui::TreePop();
        }

        if (simPhase_ == SimPhase::Running) {
            double elapsed = glfwGetTime() - simRunStart_;
            ImGui::Text("Running Tier 1... %.0f s", elapsed);
            if (!simRunLog_.empty()) ImGui::TextDisabled("%s", simRunLog_.back().c_str());
            if (ImGui::Button("Cancel", {-1, 0})) {
                simRunner_.cancel();
                simPhase_ = SimPhase::Cancelled;
                // The engine may already have printed its result before the
                // kill landed; that answer belongs to no run now.
                simSummary_ = RunSummary{};
            }
        } else {
            bool ready = problem.empty();
            if (!ready) ImGui::BeginDisabled();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.25f, 1.0f));
            if (ImGui::Button("Run Tier 1 coverage", {-1, 0})) startTier1Run();
            ImGui::PopStyleColor();
            if (!ready) ImGui::EndDisabled();
            if (simPhase_ == SimPhase::Done)
                ImGui::TextColored({0.4f, 0.9f, 0.5f, 1}, "Finished in %.0f s", simRunEnd_ - simRunStart_);
            if (simPhase_ == SimPhase::Cancelled) ImGui::TextDisabled("Cancelled.");
        }
        if (!simRunError_.empty() && simPhase_ != SimPhase::Running) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored({1, 0.45f, 0.45f, 1}, "%s", simRunError_.c_str());
            ImGui::PopTextWrapPos();
        }
        if (!simRunLog_.empty() && ImGui::TreeNode("Log")) {
            for (const auto& l : simRunLog_) ImGui::TextDisabled("%s", l.c_str());
            ImGui::TreePop();
        }
    }
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

void App::drawSimulationResultsSection() {
    if (!simView_.loaded) return;
    if (!ImGui::CollapsingHeader("Results", ImGuiTreeNodeFlags_DefaultOpen)) return;

    // Stale: anything the run depended on has changed - the spec, the run
    // settings, or an STL on disk.
    {
        std::string now, err;
        const bool same = runInputs(now, err) && now == simView_.inputs;
        if (!same)
            ImGui::TextColored({1, 0.8f, 0.3f, 1},
                               "Set-up, settings or geometry changed since this run - run again.");
    }

    ImGui::Checkbox("Show on geometry", &simView_.show);
    const ResultMesh& m = simView_.mesh;
    ImGui::SetNextItemWidth(-1);
    if (ImGui::BeginCombo("##field", simView_.field < (int)m.fields.size() ? fieldLabel(m.fields[simView_.field].name) : "")) {
        for (int i = 0; i < (int)m.fields.size(); i++) {
            if (ImGui::Selectable(fieldLabel(m.fields[i].name), i == simView_.field)) {
                simView_.field = i;
                simView_.colourDirty = true;
            }
        }
        ImGui::EndCombo();
    }

    // Legend
    if (simView_.field < (int)m.fields.size()) {
        const ResultField& f = m.fields[simView_.field];
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const float sw = ImGui::GetTextLineHeight();
        if (f.categorical) {
            for (int k = 0; k < (int)std::max(f.labels.size(), f.colours.size()); k++) {
                float c[3];
                categoryColour(f, k, c);
                ImVec2 p = ImGui::GetCursorScreenPos();
                dl->AddRectFilled(p, {p.x + sw, p.y + sw}, IM_COL32((int)(c[0] * 255), (int)(c[1] * 255), (int)(c[2] * 255), 255));
                ImGui::Dummy({sw, sw});
                ImGui::SameLine();
                std::string label = k < (int)f.labels.size() ? f.labels[k] : std::to_string(k);
                if (f.name == "reach") {
                    const auto& o = simView_.summary.overall;
                    const double pct = k == 0 ? o.dryPct : k == 1 ? o.splashPct : o.directPct;
                    ImGui::Text("%s  %.1f%%", label.c_str(), pct);
                } else {
                    ImGui::TextUnformatted(label.c_str());
                }
                if (f.name == "reach" && k == 2)
                    ImGui::TextDisabled("  percentages are of scored wall area;\n"
                                        "  caps and obstructions are drawn muted");
            }
        } else {
            const float hi = f.p95 > 0 ? f.p95 : f.maxValue;
            ImVec2 p = ImGui::GetCursorScreenPos();
            const float barW = ImGui::GetContentRegionAvail().x, barH = sw;
            const int steps = 32;
            for (int s = 0; s < steps; s++) {
                float a[3], b[3];
                rampColour((float)s / steps, a);
                rampColour((float)(s + 1) / steps, b);
                dl->AddRectFilledMultiColor({p.x + barW * s / steps, p.y}, {p.x + barW * (s + 1) / steps, p.y + barH},
                    IM_COL32((int)(a[0] * 255), (int)(a[1] * 255), (int)(a[2] * 255), 255),
                    IM_COL32((int)(b[0] * 255), (int)(b[1] * 255), (int)(b[2] * 255), 255),
                    IM_COL32((int)(b[0] * 255), (int)(b[1] * 255), (int)(b[2] * 255), 255),
                    IM_COL32((int)(a[0] * 255), (int)(a[1] * 255), (int)(a[2] * 255), 255));
            }
            ImGui::Dummy({barW, barH});
            ImGui::Text("0");
            ImGui::SameLine(barW - 90);
            ImGui::Text("%.3g %s", hi, f.unit.c_str());
            ImVec2 q = ImGui::GetCursorScreenPos();
            dl->AddRectFilled(q, {q.x + sw, q.y + sw},
                IM_COL32((int)(kNoValueColour[0] * 255), (int)(kNoValueColour[1] * 255), (int)(kNoValueColour[2] * 255), 255));
            ImGui::Dummy({sw, sw});
            ImGui::SameLine();
            ImGui::TextDisabled("none reached (scale tops out at the 95th percentile)");
            if (f.name.find("flux") != std::string::npos) {
                ImVec2 q2 = ImGui::GetCursorScreenPos();
                dl->AddRectFilled(q2, {q2.x + sw, q2.y + sw},
                    IM_COL32((int)(kUnsampledColour[0] * 255), (int)(kUnsampledColour[1] * 255),
                             (int)(kUnsampledColour[2] * 255), 255));
                ImGui::Dummy({sw, sw});
                ImGui::SameLine();
                ImGui::TextDisabled("sprayed, but no ray sampled it - raise rays");
                if (!simView_.summary.fluxTrustworthy)
                    ImGui::TextColored({1, 0.8f, 0.3f, 1},
                                       "Flux map under-sampled: raise rays. Coverage is unaffected.");
            }
            if (f.noData > 0)
                ImGui::TextDisabled("%d faces have no value for this field", f.noData);
        }
    }

    // Coverage table
    if (ImGui::BeginTable("coverage", 5, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Surface", ImGuiTableColumnFlags_WidthStretch, 2.0f);
        ImGui::TableSetupColumn("Direct");
        ImGui::TableSetupColumn("Splash");
        ImGui::TableSetupColumn("Dry");
        ImGui::TableSetupColumn("m2");
        ImGui::TableHeadersRow();
        auto rowOut = [](const CoverageRow& r, bool bold) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (bold) ImGui::TextUnformatted(r.name.c_str());
            else if (r.scored) ImGui::TextUnformatted(r.name.c_str());
            else ImGui::TextDisabled("%s (cap)", r.name.c_str());
            ImGui::TableNextColumn(); ImGui::Text("%.1f%%", r.directPct);
            ImGui::TableNextColumn(); ImGui::Text("%.1f%%", r.splashPct);
            ImGui::TableNextColumn();
            if (r.dryPct > 0.05) ImGui::TextColored({1, 0.5f, 0.45f, 1}, "%.1f%%", r.dryPct);
            else ImGui::Text("%.1f%%", r.dryPct);
            ImGui::TableNextColumn(); ImGui::Text("%.3g", r.areaM2);
        };
        rowOut(simView_.summary.overall, true);
        for (const auto& r : simView_.summary.surfaces) rowOut(r, false);
        ImGui::EndTable();
    }
    ImGui::TextDisabled("Tier 1 is line of sight + range-limited splash, not CFD.");

    if (ImGui::Button("Open in ParaView")) openResultsInParaView();
    ImGui::SameLine();
    if (ImGui::Button("Open run folder"))
        ShellExecuteW(nullptr, L"open", fsPath(simView_.runDir).c_str(), nullptr, nullptr, 1 /* SW_SHOWNORMAL */);
    if (!paraviewMessage_.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("%s", paraviewMessage_.c_str());
        ImGui::PopTextWrapPos();
    }
}

} // namespace shitcad
