#include "Preferences.h"

namespace shitcad {

static Theme s_activeTheme = Theme::light();
static ProfileDetectorBackend s_profileBackend = ProfileDetectorBackend::Custom;

const Theme& activeTheme() { return s_activeTheme; }

bool operator==(const Preferences& a, const Preferences& b) {
    auto eq = [](const float* x, const float* y, int n) {
        for (int i = 0; i < n; i++) if (x[i] != y[i]) return false;
        return true;
    };
    return a.lightMode == b.lightMode && a.showWireframe == b.showWireframe &&
           eq(a.edgeColor, b.edgeColor, 3) && a.edgeThickness == b.edgeThickness &&
           eq(a.sketchLineColor, b.sketchLineColor, 3) && a.sketchLineThickness == b.sketchLineThickness &&
           a.tangentSnapPx == b.tangentSnapPx && a.profileBackend == b.profileBackend &&
           eq(a.dimLineCol, b.dimLineCol, 4) && eq(a.dimTextCol, b.dimTextCol, 4) &&
           eq(a.dimBgCol, b.dimBgCol, 4) && eq(a.conTextCol, b.conTextCol, 4) &&
           eq(a.conBgCol, b.conBgCol, 4);
}
Theme& activeThemeMut() { return s_activeTheme; }

ProfileDetectorBackend activeProfileBackend() { return s_profileBackend; }
void setActiveProfileBackend(ProfileDetectorBackend backend) { s_profileBackend = backend; }

Theme Theme::dark() {
    Theme t;
    t.bgColor[0] = 0.12f; t.bgColor[1] = 0.12f; t.bgColor[2] = 0.14f;
    t.skyZenith[0] = 0.09f; t.skyZenith[1] = 0.11f; t.skyZenith[2] = 0.16f;
    t.skyHorizon[0] = 0.23f; t.skyHorizon[1] = 0.25f; t.skyHorizon[2] = 0.30f;
    t.groundHorizon[0] = 0.15f; t.groundHorizon[1] = 0.16f; t.groundHorizon[2] = 0.18f;
    t.groundNadir[0] = 0.08f; t.groundNadir[1] = 0.08f; t.groundNadir[2] = 0.09f;
    t.gridMinor[0] = 0.2f;  t.gridMinor[1] = 0.2f;  t.gridMinor[2] = 0.2f;
    t.gridMajor[0] = 0.35f; t.gridMajor[1] = 0.35f; t.gridMajor[2] = 0.35f;

    t.sketchLine[0] = 1.0f; t.sketchLine[1] = 1.0f; t.sketchLine[2] = 1.0f;
    t.sketchUnderconstrained[0] = 0.4f; t.sketchUnderconstrained[1] = 0.65f; t.sketchUnderconstrained[2] = 1.0f;
    t.sketchSelected[0] = 1.0f; t.sketchSelected[1] = 0.65f; t.sketchSelected[2] = 0.0f;
    t.sketchGridMinor = 0.2f;
    t.sketchGridMajor = 0.35f;
    t.toolPreview[0] = 0.39f; t.toolPreview[1] = 0.71f; t.toolPreview[2] = 1.0f; t.toolPreview[3] = 0.8f;
    t.snapColor[0] = 1.0f; t.snapColor[1] = 0.78f; t.snapColor[2] = 0.0f; t.snapColor[3] = 1.0f;

    t.dimLineColor = rgba32(0, 206, 209, 220);
    t.dimTextColor = rgba32(255, 255, 255, 240);
    t.dimBgColor   = rgba32(30, 30, 35, 200);
    t.dimDrivenLineColor = rgba32(140, 140, 140, 180);
    t.dimDrivenTextColor = rgba32(180, 180, 180, 200);

    t.sketchProjected[0] = 0.6f; t.sketchProjected[1] = 0.2f; t.sketchProjected[2] = 0.9f;

    t.bodyColor[0] = 0.6f; t.bodyColor[1] = 0.65f; t.bodyColor[2] = 0.7f;
    t.edgeColor[0] = 0.08f; t.edgeColor[1] = 0.08f; t.edgeColor[2] = 0.08f;
    return t;
}

Theme Theme::light() {
    Theme t;
    t.bgColor[0] = 0.78f; t.bgColor[1] = 0.78f; t.bgColor[2] = 0.80f;
    t.skyZenith[0] = 0.60f; t.skyZenith[1] = 0.71f; t.skyZenith[2] = 0.85f;
    t.skyHorizon[0] = 0.91f; t.skyHorizon[1] = 0.94f; t.skyHorizon[2] = 0.98f;
    t.groundHorizon[0] = 0.80f; t.groundHorizon[1] = 0.82f; t.groundHorizon[2] = 0.86f;
    t.groundNadir[0] = 0.66f; t.groundNadir[1] = 0.67f; t.groundNadir[2] = 0.70f;
    t.gridMinor[0] = 0.54f; t.gridMinor[1] = 0.54f; t.gridMinor[2] = 0.54f;
    t.gridMajor[0] = 0.42f; t.gridMajor[1] = 0.42f; t.gridMajor[2] = 0.42f;

    t.sketchLine[0] = 0.1f; t.sketchLine[1] = 0.1f; t.sketchLine[2] = 0.1f;
    t.sketchUnderconstrained[0] = 0.2f; t.sketchUnderconstrained[1] = 0.45f; t.sketchUnderconstrained[2] = 0.85f;
    t.sketchSelected[0] = 1.0f; t.sketchSelected[1] = 0.5f; t.sketchSelected[2] = 0.0f;
    t.sketchGridMinor = 0.78f;
    t.sketchGridMajor = 0.65f;
    t.toolPreview[0] = 0.2f; t.toolPreview[1] = 0.5f; t.toolPreview[2] = 0.9f; t.toolPreview[3] = 0.8f;
    t.snapColor[0] = 0.9f; t.snapColor[1] = 0.6f; t.snapColor[2] = 0.0f; t.snapColor[3] = 1.0f;

    t.dimLineColor = rgba32(0, 140, 160, 220);
    t.dimTextColor = rgba32(20, 20, 20, 240);
    t.dimBgColor   = rgba32(240, 240, 245, 200);
    t.dimDrivenLineColor = rgba32(140, 140, 145, 180);
    t.dimDrivenTextColor = rgba32(100, 100, 105, 200);

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
    Color32 dl = s_activeTheme.dimLineColor;
    dimLineCol[0] = ((dl>>0)&0xFF)/255.0f; dimLineCol[1] = ((dl>>8)&0xFF)/255.0f;
    dimLineCol[2] = ((dl>>16)&0xFF)/255.0f; dimLineCol[3] = ((dl>>24)&0xFF)/255.0f;
    Color32 dt = s_activeTheme.dimTextColor;
    dimTextCol[0] = ((dt>>0)&0xFF)/255.0f; dimTextCol[1] = ((dt>>8)&0xFF)/255.0f;
    dimTextCol[2] = ((dt>>16)&0xFF)/255.0f; dimTextCol[3] = ((dt>>24)&0xFF)/255.0f;
    Color32 db = s_activeTheme.dimBgColor;
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
}

} // namespace shitcad
