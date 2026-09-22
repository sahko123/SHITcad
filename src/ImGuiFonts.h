#pragma once
#include <imgui.h>
#include <cstdio>

namespace shitcad {

// The UI font, shared by every host so text measures the same in each: a
// system TTF at 15 px times the display scale, or ImGui's bitmap font scaled
// if none is found.
inline void loadImGuiFonts(float dpiScale) {
    ImGuiIO& io = ImGui::GetIO();
    float fontSize = 15.0f * dpiScale;
    const char* fontPaths[] = {
        "C:/Windows/Fonts/segoeui.ttf",   // Segoe UI (Windows 10/11)
        "C:/Windows/Fonts/calibri.ttf",    // Calibri fallback
        "C:/Windows/Fonts/arial.ttf",      // Arial fallback
    };
    for (const char* path : fontPaths) {
        FILE* f = fopen(path, "rb");
        if (f) {
            fclose(f);
            io.Fonts->AddFontFromFileTTF(path, fontSize);
            return;
        }
    }
    // Fall back to default bitmap font with scaling
    io.FontGlobalScale = dpiScale;
}

} // namespace shitcad
