#include "ExtrudeTool.h"
#include "ProfileDetector.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace shitcad {

// Ray-casting point-in-polygon on pre-tessellated boundary
static bool pointInTessellation(const std::vector<Point2D>& tess, Point2D localPos) {
    int tn = (int)tess.size();
    if (tn < 3) return false;
    bool inside = false;
    for (int j = 0, k = tn - 1; j < tn; k = j++) {
        Point2D a = tess[j], b = tess[k];
        if (((a.y > localPos.y) != (b.y > localPos.y)) &&
            (localPos.x < (b.x - a.x) * (localPos.y - a.y) / (b.y - a.y) + a.x)) {
            inside = !inside;
        }
    }
    return inside;
}

static bool pointInProfileBoundary(const Sketch& sketch, const ClosedProfile& p, Point2D localPos,
                                    const std::vector<Point2D>* cached = nullptr) {
    if (cached && !cached->empty()) {
        return pointInTessellation(*cached, localPos);
    }
    auto tess = tessellateProfile(sketch, p);
    return pointInTessellation(tess, localPos);
}

int hitTestProfile(const Sketch& sketch, const std::vector<ClosedProfile>& profiles,
                   Point2D localPos, const std::vector<ProfileRenderCache>& renderCache) {
    bool hasCache = (renderCache.size() == profiles.size());

    for (int i = 0; i < (int)profiles.size(); i++) {
        const auto& p = profiles[i];
        const std::vector<Point2D>* cached = (hasCache && !renderCache[i].tessPoints.empty())
            ? &renderCache[i].tessPoints : nullptr;

        if (!pointInProfileBoundary(sketch, p, localPos, cached)) continue;

        // Check that point is not inside any hole
        bool inHole = false;
        for (const auto& hole : p.holes) {
            // Holes don't have their own render cache, compute on demand
            if (pointInProfileBoundary(sketch, hole, localPos)) {
                inHole = true;
                break;
            }
        }
        if (!inHole) return i;
    }
    return -1;
}

// Ear-clipping triangulation helper
static void earClipTriangulate(const std::vector<Point2D>& pts, std::vector<int>& triIndices) {
    int n = (int)pts.size();
    if (n < 3) return;

    std::vector<int> indices(n);
    for (int j = 0; j < n; j++) indices[j] = j;

    // Ensure CCW winding
    double signedArea = 0;
    for (int j = 0; j < n; j++) {
        int jn = (j + 1) % n;
        signedArea += pts[j].x * pts[jn].y - pts[jn].x * pts[j].y;
    }
    if (signedArea < 0) std::reverse(indices.begin(), indices.end());

    auto cross2D = [](Point2D o, Point2D a, Point2D b) -> double {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };

    auto pointInTriangle = [&](Point2D p, Point2D a, Point2D b, Point2D c) -> bool {
        double d1 = cross2D(p, a, b);
        double d2 = cross2D(p, b, c);
        double d3 = cross2D(p, c, a);
        bool hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        bool hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);
        return !(hasNeg && hasPos);
    };

    while ((int)indices.size() > 2) {
        int sz = (int)indices.size();
        bool earFound = false;
        for (int j = 0; j < sz; j++) {
            int prev = (j - 1 + sz) % sz;
            int next = (j + 1) % sz;
            Point2D pA = pts[indices[prev]];
            Point2D pB = pts[indices[j]];
            Point2D pC = pts[indices[next]];

            if (cross2D(pA, pB, pC) <= 1e-7f) continue;

            bool isEar = true;
            for (int k = 0; k < sz; k++) {
                if (k == prev || k == j || k == next) continue;
                if (pointInTriangle(pts[indices[k]], pA, pB, pC)) {
                    isEar = false;
                    break;
                }
            }
            if (!isEar) continue;

            triIndices.push_back(indices[prev]);
            triIndices.push_back(indices[j]);
            triIndices.push_back(indices[next]);
            indices.erase(indices.begin() + j);
            earFound = true;
            break;
        }
        if (!earFound) break;
    }
}

// Merge outer polygon with holes using bridge edges so ear-clipping can handle them.
// Returns a single simple polygon that can be triangulated.
static std::vector<Point2D> mergePolygonWithHoles(
        const std::vector<Point2D>& outer,
        const std::vector<std::vector<Point2D>>& holes) {
    if (holes.empty()) return outer;

    std::vector<Point2D> result = outer;

    for (const auto& hole : holes) {
        if (hole.size() < 3) continue;

        // Find rightmost vertex of hole
        int rightmostIdx = 0;
        for (int i = 1; i < (int)hole.size(); i++) {
            if (hole[i].x > hole[rightmostIdx].x ||
                (hole[i].x == hole[rightmostIdx].x && hole[i].y > hole[rightmostIdx].y))
                rightmostIdx = i;
        }
        Point2D holePoint = hole[rightmostIdx];

        // Find closest vertex on result polygon
        int bestIdx = 0;
        double bestDist = std::numeric_limits<double>::max();
        for (int i = 0; i < (int)result.size(); i++) {
            double dx = result[i].x - holePoint.x;
            double dy = result[i].y - holePoint.y;
            double dist = dx * dx + dy * dy;
            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = i;
            }
        }

        // Build merged polygon: outer[0..bestIdx] + hole loop + back to outer[bestIdx..]
        std::vector<Point2D> merged;
        for (int i = 0; i <= bestIdx; i++)
            merged.push_back(result[i]);

        int holeN = (int)hole.size();
        for (int i = 0; i <= holeN; i++)
            merged.push_back(hole[(rightmostIdx + i) % holeN]);

        for (int i = bestIdx; i < (int)result.size(); i++)
            merged.push_back(result[i]);

        result = merged;
    }

    return result;
}

void buildProfileRenderCaches(const Sketch& sketch,
                              const std::vector<ClosedProfile>& profiles,
                              std::vector<ProfileRenderCache>& out) {
    out.resize(profiles.size());
    for (int i = 0; i < (int)profiles.size(); i++) {
        const auto& profile = profiles[i];
        auto& cache = out[i];

        if (profile.isCircle()) {
            cache.tessPoints = tessellateProfile(sketch, profile);
            // No triIndices — circle rendering uses triangle fan directly
        } else {
            cache.tessPoints = tessellateProfile(sketch, profile);

            // Tessellate holes
            for (const auto& hole : profile.holes) {
                cache.holeTessPoints.push_back(tessellateProfile(sketch, hole));
            }

            // Merge outer boundary with holes for correct triangulation
            if (cache.holeTessPoints.empty()) {
                cache.triMeshPoints = cache.tessPoints;
                earClipTriangulate(cache.triMeshPoints, cache.triIndices);
            } else {
                cache.triMeshPoints = mergePolygonWithHoles(cache.tessPoints, cache.holeTessPoints);
                earClipTriangulate(cache.triMeshPoints, cache.triIndices);
            }
        }
    }
}

} // namespace shitcad
