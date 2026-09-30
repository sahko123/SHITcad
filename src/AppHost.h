#pragma once
#include "FileDialogs.h"
#include <string>

namespace shitcad {

// What App needs from the program that hosts it (the window, the event loop).
// App never talks to Qt directly; the host owns the window and the GL context,
// and drives App once per frame:
//
//   app.frame(dt, framebufferW, framebufferH, input);   // input, tools, overlay
//   app.paint();                                        // 3D scene
//   ...then draws app.overlay() over it
//
// with the GL context current.
class AppHost {
public:
    virtual ~AppHost() = default;
    virtual void setWindowTitle(const std::string& utf8Title) = 0;
    // Ask for another frame. A no-op while the host renders continuously;
    // an on-demand host must honour it (see App::post).
    virtual void requestRedraw() {}
    // Show a file or folder chooser; `utf8Path` is the choice, empty if
    // cancelled. The return value is unused.
    virtual bool chooseFile(FileDialog kind, const char* title, std::string& utf8Path) = 0;
};

} // namespace shitcad
