#pragma once
// Shared by the test executables: the CHECK counter, tolerance comparisons and the
// closed-box STL fixture every mesh test builds its inputs from.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <vector>

inline int g_failures = 0;
inline int g_checks = 0;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            g_failures++;                                                   \
            std::printf("  FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); \
            std::printf(__VA_ARGS__);                                       \
            std::printf("\n");                                              \
        }                                                                   \
    } while (0)

inline bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
inline bool nearRel(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fabs(b); }

struct Tri { float v[9]; };

// Closed box with outward-facing (counter-clockwise from outside) triangles.
inline std::vector<Tri> boxTriangles(float x0, float y0, float z0, float x1, float y1, float z1) {
    const float c[8][3] = {
        {x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
        {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1},
    };
    const int q[6][4] = {
        {0, 3, 2, 1}, // bottom  -z
        {4, 5, 6, 7}, // top     +z
        {0, 1, 5, 4}, // front   -y
        {2, 3, 7, 6}, // back    +y
        {1, 2, 6, 5}, // right   +x
        {3, 0, 4, 7}, // left    -x
    };
    std::vector<Tri> out;
    for (const auto& f : q) {
        for (const auto& t : {std::array<int, 3>{f[0], f[1], f[2]}, std::array<int, 3>{f[0], f[2], f[3]}}) {
            Tri tri;
            for (int k = 0; k < 3; k++)
                for (int a = 0; a < 3; a++) tri.v[k * 3 + a] = c[t[k]][a];
            out.push_back(tri);
        }
    }
    return out;
}

inline void writeBinaryStl(const std::filesystem::path& path, const std::vector<Tri>& tris) {
    std::ofstream f(path, std::ios::binary);
    char header[80] = {};
    f.write(header, 80);
    uint32_t n = (uint32_t)tris.size();
    f.write((const char*)&n, 4);
    for (const auto& t : tris) {
        const float zero[3] = {0, 0, 0};
        f.write((const char*)zero, 12);
        f.write((const char*)t.v, 36);
        uint16_t attr = 0;
        f.write((const char*)&attr, 2);
    }
}

inline void writeBoxStl(const std::filesystem::path& path,
                        float x0, float y0, float z0, float x1, float y1, float z1) {
    writeBinaryStl(path, boxTriangles(x0, y0, z0, x1, y1, z1));
}
