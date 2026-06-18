#include "Preferences.h"

namespace shitcad {

static Theme s_activeTheme = Theme::light();
static ProfileDetectorBackend s_profileBackend = ProfileDetectorBackend::Custom;

const Theme& activeTheme() { return s_activeTheme; }
Theme& activeThemeMut() { return s_activeTheme; }

ProfileDetectorBackend activeProfileBackend() { return s_profileBackend; }
void setActiveProfileBackend(ProfileDetectorBackend backend) { s_profileBackend = backend; }

Theme Theme::dark() {
    Theme t;
    t.bgColor[0] = 0.12f; t.bgColor[1] = 0.12f; t.bgColor[2] = 0.14f;
    t.gridMinor[0] = 0.2f;  t.gridMinor[1] = 0.2f;  t.gridMinor[2] = 0.2f;
    t.gridMajor[0] = 0.35f; t.gridMajor[1] = 0.35f; t.gridMajor[2] = 0.35f;

    t.sketchLine[0] = 1.0f; t.sketchLine[1] = 1.0f; t.sketchLine[2] = 1.0f;
    t.sketchUnderconstrained[0] = 0.4f; t.sketchUnderconstrained[1] = 0.65f; t.sketchUnderconstrained[2] = 1.0f;
    t.sketchSelected[0] = 1.0f; t.sketchSelected[1] = 0.65f; t.sketchSelected[2] = 0.0f;
    t.sketchGridMinor = 0.2f;
    t.sketchGridMajor = 0.35f;
    t.toolPreview[0] = 0.39f; t.toolPreview[1] = 0.71f; t.toolPreview[2] = 1.0f; t.toolPreview[3] = 0.8f;
    t.snapColor[0] = 1.0f; t.snapColor[1] = 0.78f; t.snapColor[2] = 0.0f; t.snapColor[3] = 1.0f;

    t.dimLineColor = IM_COL32(0, 206, 209, 220);
    t.dimTextColor = IM_COL32(255, 255, 255, 240);
    t.dimBgColor   = IM_COL32(30, 30, 35, 200);
    t.dimDrivenLineColor = IM_COL32(140, 140, 140, 180);
    t.dimDrivenTextColor = IM_COL32(180, 180, 180, 200);

    t.sketchProjected[0] = 0.6f; t.sketchProjected[1] = 0.2f; t.sketchProjected[2] = 0.9f;

    t.bodyColor[0] = 0.6f; t.bodyColor[1] = 0.65f; t.bodyColor[2] = 0.7f;
    t.edgeColor[0] = 0.08f; t.edgeColor[1] = 0.08f; t.edgeColor[2] = 0.08f;
    return t;
}

Theme Theme::light() {
    Theme t;
    t.bgColor[0] = 0.78f; t.bgColor[1] = 0.78f; t.bgColor[2] = 0.80f;
    t.gridMinor[0] = 0.65f; t.gridMinor[1] = 0.65f; t.gridMinor[2] = 0.65f;
    t.gridMajor[0] = 0.55f; t.gridMajor[1] = 0.55f; t.gridMajor[2] = 0.55f;

    t.sketchLine[0] = 0.1f; t.sketchLine[1] = 0.1f; t.sketchLine[2] = 0.1f;
    t.sketchUnderconstrained[0] = 0.2f; t.sketchUnderconstrained[1] = 0.45f; t.sketchUnderconstrained[2] = 0.85f;
    t.sketchSelected[0] = 1.0f; t.sketchSelected[1] = 0.5f; t.sketchSelected[2] = 0.0f;
    t.sketchGridMinor = 0.78f;
    t.sketchGridMajor = 0.65f;
    t.toolPreview[0] = 0.2f; t.toolPreview[1] = 0.5f; t.toolPreview[2] = 0.9f; t.toolPreview[3] = 0.8f;
    t.snapColor[0] = 0.9f; t.snapColor[1] = 0.6f; t.snapColor[2] = 0.0f; t.snapColor[3] = 1.0f;

    t.dimLineColor = IM_COL32(0, 140, 160, 220);
    t.dimTextColor = IM_COL32(20, 20, 20, 240);
    t.dimBgColor   = IM_COL32(240, 240, 245, 200);
    t.dimDrivenLineColor = IM_COL32(140, 140, 145, 180);
    t.dimDrivenTextColor = IM_COL32(100, 100, 105, 200);

    t.sketchProjected[0] = 0.5f; t.sketchProjected[1] = 0.1f; t.sketchProjected[2] = 0.8f;

    t.bodyColor[0] = 0.7f; t.bodyColor[1] = 0.72f; t.bodyColor[2] = 0.75f;
    t.edgeColor[0] = 0.30f; t.edgeColor[1] = 0.30f; t.edgeColor[2] = 0.30f;
    return t;
}

void Preferences::applyTheme() {
    s_activeTheme = lightMode ? Theme::light() : Theme::dark();

    // Reset user-adjustable colors to theme defaults
    edgeColor[0] = s_activeTheme.edgeColor[0];
    edgeColor[1] = s_activeTheme.edgeColor[1];
    edgeColor[2] = s_activeTheme.edgeColor[2];
    sketchLineColor[0] = s_activeTheme.sketchLine[0];
    sketchLineColor[1] = s_activeTheme.sketchLine[1];
    sketchLineColor[2] = s_activeTheme.sketchLine[2];

    // Dimension label colors from theme
    ImU32 dl = s_activeTheme.dimLineColor;
    dimLineCol[0] = ((dl>>0)&0xFF)/255.0f; dimLineCol[1] = ((dl>>8)&0xFF)/255.0f;
    dimLineCol[2] = ((dl>>16)&0xFF)/255.0f; dimLineCol[3] = ((dl>>24)&0xFF)/255.0f;
    ImU32 dt = s_activeTheme.dimTextColor;
    dimTextCol[0] = ((dt>>0)&0xFF)/255.0f; dimTextCol[1] = ((dt>>8)&0xFF)/255.0f;
    dimTextCol[2] = ((dt>>16)&0xFF)/255.0f; dimTextCol[3] = ((dt>>24)&0xFF)/255.0f;
    ImU32 db = s_activeTheme.dimBgColor;
    dimBgCol[0] = ((db>>0)&0xFF)/255.0f; dimBgCol[1] = ((db>>8)&0xFF)/255.0f;
    dimBgCol[2] = ((db>>16)&0xFF)/255.0f; dimBgCol[3] = ((db>>24)&0xFF)/255.0f;

    // Constraint icon colors
    if (lightMode) {
        conTextCol[0] = 100/255.0f; conTextCol[1] = 60/255.0f; conTextCol[2] = 160/255.0f; conTextCol[3] = 200/255.0f;
        conBgCol[0] = 1.0f; conBgCol[1] = 1.0f; conBgCol[2] = 1.0f; conBgCol[3] = 160/255.0f;
    } else {
        conTextCol[0] = 180/255.0f; conTextCol[1] = 160/255.0f; conTextCol[2] = 255/255.0f; conTextCol[3] = 200/255.0f;
        conBgCol[0] = 30/255.0f; conBgCol[1] = 30/255.0f; conBgCol[2] = 40/255.0f; conBgCol[3] = 160/255.0f;
    }

    if (lightMode) {
        ImGui::StyleColorsLight();
    } else {
        ImGui::StyleColorsDark();
    }

    ImGuiStyle& style = ImGui::GetStyle();

    // Geometry — rounded, spacious, modern
    style.WindowRounding    = 6.0f;
    style.FrameRounding     = 4.0f;
    style.GrabRounding      = 4.0f;
    style.TabRounding       = 4.0f;
    style.PopupRounding     = 4.0f;
    style.ChildRounding     = 4.0f;
    style.ScrollbarRounding = 6.0f;

    style.WindowBorderSize  = 1.0f;
    style.FrameBorderSize   = 0.0f;
    style.PopupBorderSize   = 1.0f;

    style.FramePadding      = {8, 5};
    style.ItemSpacing       = {8, 5};
    style.ItemInnerSpacing  = {6, 4};
    style.WindowPadding     = {10, 10};
    style.ScrollbarSize     = 14.0f;
    style.GrabMinSize       = 12.0f;

    // Color overrides for a polished Fusion 360-inspired look
    if (lightMode) {
        // Toolbar / window backgrounds — soft warm gray
        style.Colors[ImGuiCol_WindowBg]         = {0.94f, 0.94f, 0.95f, 1.0f};
        style.Colors[ImGuiCol_ChildBg]          = {0.94f, 0.94f, 0.95f, 1.0f};
        style.Colors[ImGuiCol_PopupBg]          = {0.97f, 0.97f, 0.98f, 0.98f};
        style.Colors[ImGuiCol_Border]           = {0.78f, 0.78f, 0.80f, 0.65f};

        // Buttons
        style.Colors[ImGuiCol_Button]           = {0.84f, 0.84f, 0.86f, 1.0f};
        style.Colors[ImGuiCol_ButtonHovered]    = {0.74f, 0.80f, 0.92f, 1.0f};
        style.Colors[ImGuiCol_ButtonActive]     = {0.55f, 0.65f, 0.85f, 1.0f};

        // Frames (inputs, combos)
        style.Colors[ImGuiCol_FrameBg]          = {0.88f, 0.88f, 0.90f, 1.0f};
        style.Colors[ImGuiCol_FrameBgHovered]   = {0.82f, 0.85f, 0.92f, 1.0f};
        style.Colors[ImGuiCol_FrameBgActive]    = {0.75f, 0.80f, 0.90f, 1.0f};

        // Header (tree nodes, collapsible)
        style.Colors[ImGuiCol_Header]           = {0.82f, 0.85f, 0.92f, 0.6f};
        style.Colors[ImGuiCol_HeaderHovered]    = {0.70f, 0.78f, 0.92f, 0.8f};
        style.Colors[ImGuiCol_HeaderActive]     = {0.55f, 0.65f, 0.85f, 1.0f};

        // Accent — blue highlight
        style.Colors[ImGuiCol_CheckMark]        = {0.20f, 0.45f, 0.80f, 1.0f};
        style.Colors[ImGuiCol_SliderGrab]       = {0.30f, 0.50f, 0.80f, 1.0f};
        style.Colors[ImGuiCol_SliderGrabActive] = {0.20f, 0.40f, 0.75f, 1.0f};

        // Separators
        style.Colors[ImGuiCol_Separator]        = {0.75f, 0.75f, 0.78f, 0.5f};

        // Title bar
        style.Colors[ImGuiCol_TitleBg]          = {0.88f, 0.88f, 0.90f, 1.0f};
        style.Colors[ImGuiCol_TitleBgActive]    = {0.82f, 0.85f, 0.92f, 1.0f};

        // Text
        style.Colors[ImGuiCol_Text]             = {0.12f, 0.12f, 0.14f, 1.0f};
        style.Colors[ImGuiCol_TextDisabled]     = {0.50f, 0.50f, 0.52f, 1.0f};
    } else {
        // Dark theme — charcoal with blue accents
        style.Colors[ImGuiCol_WindowBg]         = {0.16f, 0.16f, 0.18f, 1.0f};
        style.Colors[ImGuiCol_ChildBg]          = {0.16f, 0.16f, 0.18f, 1.0f};
        style.Colors[ImGuiCol_PopupBg]          = {0.14f, 0.14f, 0.16f, 0.98f};
        style.Colors[ImGuiCol_Border]           = {0.30f, 0.30f, 0.33f, 0.50f};

        // Buttons
        style.Colors[ImGuiCol_Button]           = {0.24f, 0.24f, 0.27f, 1.0f};
        style.Colors[ImGuiCol_ButtonHovered]    = {0.30f, 0.38f, 0.55f, 1.0f};
        style.Colors[ImGuiCol_ButtonActive]     = {0.25f, 0.35f, 0.58f, 1.0f};

        // Frames
        style.Colors[ImGuiCol_FrameBg]          = {0.20f, 0.20f, 0.23f, 1.0f};
        style.Colors[ImGuiCol_FrameBgHovered]   = {0.26f, 0.30f, 0.40f, 1.0f};
        style.Colors[ImGuiCol_FrameBgActive]    = {0.28f, 0.34f, 0.50f, 1.0f};

        // Header
        style.Colors[ImGuiCol_Header]           = {0.24f, 0.28f, 0.38f, 0.6f};
        style.Colors[ImGuiCol_HeaderHovered]    = {0.28f, 0.35f, 0.52f, 0.8f};
        style.Colors[ImGuiCol_HeaderActive]     = {0.25f, 0.35f, 0.58f, 1.0f};

        // Accent
        style.Colors[ImGuiCol_CheckMark]        = {0.40f, 0.65f, 1.00f, 1.0f};
        style.Colors[ImGuiCol_SliderGrab]       = {0.35f, 0.55f, 0.85f, 1.0f};
        style.Colors[ImGuiCol_SliderGrabActive] = {0.40f, 0.60f, 0.90f, 1.0f};

        // Separators
        style.Colors[ImGuiCol_Separator]        = {0.32f, 0.32f, 0.36f, 0.5f};

        // Title bar
        style.Colors[ImGuiCol_TitleBg]          = {0.14f, 0.14f, 0.16f, 1.0f};
        style.Colors[ImGuiCol_TitleBgActive]    = {0.20f, 0.24f, 0.34f, 1.0f};

        // Text
        style.Colors[ImGuiCol_Text]             = {0.92f, 0.92f, 0.94f, 1.0f};
        style.Colors[ImGuiCol_TextDisabled]     = {0.50f, 0.50f, 0.52f, 1.0f};
    }
}

} // namespace shitcad
