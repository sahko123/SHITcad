#pragma once
#include <cstdint>

namespace shitcad {

// Every file / folder chooser the app opens, described portably so a host can
// show its own dialog (QFileDialog in the Qt build). The GLFW build keeps the
// Win32 dialogs in Serialization.cpp.
enum class FileDialog : uint8_t {
    OpenProject, SaveProject,
    OpenImport,                   // STEP, IGES or STL
    OpenStl, SaveStl, OpenStep, SaveStep, OpenIges, SaveIges,
    SaveObj, SaveDxf, SaveJson,
    PickFolder,
};

struct FileDialogSpec {
    bool save;                    // save (confirm overwrite) vs open (must exist)
    bool folder;                  // choose a directory
    const char* title;
    const char* filter;           // Qt syntax: "STL Files (*.stl);;All Files (*)"
    const char* defaultSuffix;    // appended on save when none is typed; may be ""
};

inline const FileDialogSpec& fileDialogSpec(FileDialog d) {
    static const FileDialogSpec kSpecs[] = {
        {false, false, "Open Project", "SHITcad Files (*.shitcad);;All Files (*)", ""},
        {true,  false, "Save Project", "SHITcad Files (*.shitcad);;All Files (*)", "shitcad"},
        {false, false, "Import", "All Supported (*.step *.stp *.igs *.iges *.stl);;STEP Files (*.step *.stp);;"
                                 "IGES Files (*.igs *.iges);;STL Files (*.stl);;All Files (*)", ""},
        {false, false, "Import STL", "STL Files (*.stl);;All Files (*)", ""},
        {true,  false, "Export STL", "STL Files (*.stl);;All Files (*)", "stl"},
        {false, false, "Import STEP", "STEP Files (*.step *.stp);;All Files (*)", ""},
        {true,  false, "Export STEP", "STEP Files (*.step *.stp);;All Files (*)", "step"},
        {false, false, "Import IGES", "IGES Files (*.igs *.iges);;All Files (*)", ""},
        {true,  false, "Export IGES", "IGES Files (*.igs *.iges);;All Files (*)", "igs"},
        {true,  false, "Export OBJ", "OBJ Files (*.obj);;All Files (*)", "obj"},
        {true,  false, "Export DXF", "DXF Files (*.dxf);;All Files (*)", "dxf"},
        {true,  false, "Export JSON", "JSON Files (*.json);;All Files (*)", "json"},
        {false, true,  "Select Folder", "", ""},
    };
    return kSpecs[(int)d];
}

} // namespace shitcad
