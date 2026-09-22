#pragma once
#include <filesystem>
#include <string>

namespace shitcad {

// Every path SHITcad holds in a std::string is UTF-8: the native dialogs are
// converted at the boundary (see Serialization.h ansiToUtf8), paths typed into
// ImGui are UTF-8, and paths read back from JSON are UTF-8.
//
// The standard library disagrees on Windows: std::ofstream(std::string),
// std::filesystem::path(std::string) and path::string() all use the ANSI code
// page. Handing them a UTF-8 path fails to open anything outside ASCII, and
// path::string() throws on a name ANSI cannot represent. Go through these two
// instead. (OCCT's `const char*` file APIs already take UTF-8 - pass those the
// std::string directly, never a path::string().)
inline std::filesystem::path fsPath(const std::string& utf8) {
    return std::filesystem::u8path(utf8);
}

inline std::string utf8(const std::filesystem::path& p) {
    return p.u8string(); // std::string under C++17
}

} // namespace shitcad
