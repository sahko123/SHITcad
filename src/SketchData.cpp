#include "SketchData.h"
#include "Constants.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace shitcad {

// ─── Index-accelerated lookups ─────────────────────────────────────

void Sketch::rebuildIndices() {
    pointIndex_.clear();
    for (size_t i = 0; i < points.size(); i++) pointIndex_[points[i].id] = i;
    lineIndex_.clear();
    for (size_t i = 0; i < lines.size(); i++) lineIndex_[lines[i].id] = i;
    circleIndex_.clear();
    for (size_t i = 0; i < circles.size(); i++) circleIndex_[circles[i].id] = i;
    arcIndex_.clear();
    for (size_t i = 0; i < arcs.size(); i++) arcIndex_[arcs[i].id] = i;
    ellipseIndex_.clear();
    for (size_t i = 0; i < ellipses.size(); i++) ellipseIndex_[ellipses[i].id] = i;
    ellipseArcIndex_.clear();
    for (size_t i = 0; i < ellipseArcs.size(); i++) ellipseArcIndex_[ellipseArcs[i].id] = i;
    splineIndex_.clear();
    for (size_t i = 0; i < splines.size(); i++) splineIndex_[splines[i].id] = i;
    constraintIndex_.clear();
    for (size_t i = 0; i < constraints.size(); i++) constraintIndex_[constraints[i].id] = i;
}

PointEntity* Sketch::findPoint(EntityID id) {
    auto it = pointIndex_.find(id);
    if (it != pointIndex_.end() && it->second < points.size() && points[it->second].id == id)
        return &points[it->second];
    // Fallback: linear scan (index may be stale)
    for (auto& p : points)
        if (p.id == id) return &p;
    return nullptr;
}

const PointEntity* Sketch::findPoint(EntityID id) const {
    auto it = pointIndex_.find(id);
    if (it != pointIndex_.end() && it->second < points.size() && points[it->second].id == id)
        return &points[it->second];
    for (auto& p : points)
        if (p.id == id) return &p;
    return nullptr;
}

LineEntity* Sketch::findLine(EntityID id) {
    auto it = lineIndex_.find(id);
    if (it != lineIndex_.end() && it->second < lines.size() && lines[it->second].id == id)
        return &lines[it->second];
    for (auto& l : lines)
        if (l.id == id) return &l;
    return nullptr;
}

const LineEntity* Sketch::findLine(EntityID id) const {
    auto it = lineIndex_.find(id);
    if (it != lineIndex_.end() && it->second < lines.size() && lines[it->second].id == id)
        return &lines[it->second];
    for (const auto& l : lines)
        if (l.id == id) return &l;
    return nullptr;
}

CircleEntity* Sketch::findCircle(EntityID id) {
    auto it = circleIndex_.find(id);
    if (it != circleIndex_.end() && it->second < circles.size() && circles[it->second].id == id)
        return &circles[it->second];
    for (auto& c : circles)
        if (c.id == id) return &c;
    return nullptr;
}

const CircleEntity* Sketch::findCircle(EntityID id) const {
    auto it = circleIndex_.find(id);
    if (it != circleIndex_.end() && it->second < circles.size() && circles[it->second].id == id)
        return &circles[it->second];
    for (auto& c : circles)
        if (c.id == id) return &c;
    return nullptr;
}

ArcEntity* Sketch::findArc(EntityID id) {
    auto it = arcIndex_.find(id);
    if (it != arcIndex_.end() && it->second < arcs.size() && arcs[it->second].id == id)
        return &arcs[it->second];
    for (auto& a : arcs)
        if (a.id == id) return &a;
    return nullptr;
}

const ArcEntity* Sketch::findArc(EntityID id) const {
    auto it = arcIndex_.find(id);
    if (it != arcIndex_.end() && it->second < arcs.size() && arcs[it->second].id == id)
        return &arcs[it->second];
    for (const auto& a : arcs)
        if (a.id == id) return &a;
    return nullptr;
}

EllipseEntity* Sketch::findEllipse(EntityID id) {
    auto it = ellipseIndex_.find(id);
    if (it != ellipseIndex_.end() && it->second < ellipses.size() && ellipses[it->second].id == id)
        return &ellipses[it->second];
    for (auto& e : ellipses)
        if (e.id == id) return &e;
    return nullptr;
}

const EllipseEntity* Sketch::findEllipse(EntityID id) const {
    auto it = ellipseIndex_.find(id);
    if (it != ellipseIndex_.end() && it->second < ellipses.size() && ellipses[it->second].id == id)
        return &ellipses[it->second];
    for (const auto& e : ellipses)
        if (e.id == id) return &e;
    return nullptr;
}

EllipseArcEntity* Sketch::findEllipseArc(EntityID id) {
    auto it = ellipseArcIndex_.find(id);
    if (it != ellipseArcIndex_.end() && it->second < ellipseArcs.size() && ellipseArcs[it->second].id == id)
        return &ellipseArcs[it->second];
    for (auto& e : ellipseArcs)
        if (e.id == id) return &e;
    return nullptr;
}

const EllipseArcEntity* Sketch::findEllipseArc(EntityID id) const {
    auto it = ellipseArcIndex_.find(id);
    if (it != ellipseArcIndex_.end() && it->second < ellipseArcs.size() && ellipseArcs[it->second].id == id)
        return &ellipseArcs[it->second];
    for (const auto& e : ellipseArcs)
        if (e.id == id) return &e;
    return nullptr;
}

SplineEntity* Sketch::findSpline(EntityID id) {
    auto it = splineIndex_.find(id);
    if (it != splineIndex_.end() && it->second < splines.size() && splines[it->second].id == id)
        return &splines[it->second];
    for (auto& s : splines)
        if (s.id == id) return &s;
    return nullptr;
}

const SplineEntity* Sketch::findSpline(EntityID id) const {
    auto it = splineIndex_.find(id);
    if (it != splineIndex_.end() && it->second < splines.size() && splines[it->second].id == id)
        return &splines[it->second];
    for (const auto& s : splines)
        if (s.id == id) return &s;
    return nullptr;
}

Constraint* Sketch::findConstraint(EntityID id) {
    auto it = constraintIndex_.find(id);
    if (it != constraintIndex_.end() && it->second < constraints.size() && constraints[it->second].id == id)
        return &constraints[it->second];
    for (auto& c : constraints)
        if (c.id == id) return &c;
    return nullptr;
}

EntityID Sketch::addPoint(double x, double y) {
    EntityID id = genID();
    pointIndex_[id] = points.size();
    points.push_back({id, x, y});
    dirty = true;
    return id;
}

EntityID Sketch::addLine(EntityID startPt, EntityID endPt) {
    EntityID id = genID();
    lineIndex_[id] = lines.size();
    lines.push_back({id, startPt, endPt});
    dirty = true;
    return id;
}

EntityID Sketch::addCircle(EntityID centerPt, double radius) {
    EntityID id = genID();
    circleIndex_[id] = circles.size();
    circles.push_back({id, centerPt, radius});
    dirty = true;
    return id;
}

EntityID Sketch::addArc(EntityID centerPt, EntityID startPt, EntityID endPt) {
    EntityID id = genID();
    ArcEntity arc;
    arc.id = id;
    arc.centerPt = centerPt;
    arc.startPt = startPt;
    arc.endPt = endPt;
    recomputeArcAngles(arc);
    arcIndex_[id] = arcs.size();
    arcs.push_back(arc);
    dirty = true;
    return id;
}

EntityID Sketch::addEllipse(EntityID centerPt, double semiMajor, double semiMinor, double rotation) {
    EntityID id = genID();
    EllipseEntity e;
    e.id = id;
    e.centerPt = centerPt;
    e.semiMajor = semiMajor;
    e.semiMinor = semiMinor;
    e.rotation = rotation;
    ellipseIndex_[id] = ellipses.size();
    ellipses.push_back(e);
    dirty = true;
    return id;
}

EntityID Sketch::addEllipseArc(EntityID centerPt, EntityID startPt, EntityID endPt,
                                double semiMajor, double semiMinor, double rotation) {
    EntityID id = genID();
    EllipseArcEntity e;
    e.id = id;
    e.centerPt = centerPt;
    e.startPt = startPt;
    e.endPt = endPt;
    e.semiMajor = semiMajor;
    e.semiMinor = semiMinor;
    e.rotation = rotation;
    // Compute angles from point positions
    Point2D c = getPointPos(centerPt);
    Point2D s = getPointPos(startPt);
    Point2D ep = getPointPos(endPt);
    e.startAngle = std::atan2(s.y - c.y, s.x - c.x);
    e.endAngle = std::atan2(ep.y - c.y, ep.x - c.x);
    ellipseArcIndex_[id] = ellipseArcs.size();
    ellipseArcs.push_back(e);
    dirty = true;
    return id;
}

EntityID Sketch::addSpline(const std::vector<EntityID>& controlPts, int degree, bool periodic) {
    EntityID id = genID();
    SplineEntity s;
    s.id = id;
    s.controlPtIDs = controlPts;
    s.degree = degree;
    s.periodic = periodic;
    splineIndex_[id] = splines.size();
    splines.push_back(s);
    dirty = true;
    return id;
}

EntityID Sketch::addConstraint(ConstraintType type, EntityID a, EntityID b,
                               double value, bool isAuto) {
    EntityID id = genID();
    Constraint c;
    c.id = id;
    c.type = type;
    c.entityA = a;
    c.entityB = b;
    c.value = value;
    c.isAuto = isAuto;
    constraintIndex_[id] = constraints.size();
    constraints.push_back(c);
    dirty = true;
    return id;
}

void Sketch::removeConstraint(EntityID id) {
    constraintIndex_.erase(id);
    constraints.erase(
        std::remove_if(constraints.begin(), constraints.end(),
                        [id](const Constraint& c) { return c.id == id; }),
        constraints.end());
    // Rebuild constraint indices (positions shifted)
    constraintIndex_.clear();
    for (size_t i = 0; i < constraints.size(); i++) constraintIndex_[constraints[i].id] = i;
    dirty = true;
}

void Sketch::removeConstraintsReferencing(EntityID id) {
    auto oldSize = constraints.size();
    constraints.erase(
        std::remove_if(constraints.begin(), constraints.end(),
            [id](const Constraint& c) {
                return c.entityA == id || c.entityB == id || c.entityC == id;
            }),
        constraints.end());
    if (constraints.size() != oldSize) dirty = true;
}

bool Sketch::isPointReferenced(EntityID pointID) const {
    for (const auto& line : lines) {
        if (line.startPt == pointID || line.endPt == pointID)
            return true;
    }
    for (const auto& circle : circles) {
        if (circle.centerPt == pointID)
            return true;
    }
    for (const auto& arc : arcs) {
        if (arc.centerPt == pointID || arc.startPt == pointID || arc.endPt == pointID)
            return true;
    }
    for (const auto& e : ellipses) {
        if (e.centerPt == pointID)
            return true;
    }
    for (const auto& ea : ellipseArcs) {
        if (ea.centerPt == pointID || ea.startPt == pointID || ea.endPt == pointID)
            return true;
    }
    for (const auto& sp : splines) {
        for (auto ptID : sp.controlPtIDs)
            if (ptID == pointID) return true;
    }
    return false;
}

bool Sketch::canRemovePoint(EntityID pointID) const {
    const PointEntity* pt = findPoint(pointID);
    if (pt && pt->projected) return false;
    return !isPointReferenced(pointID);
}

void Sketch::removePoint(EntityID id) {
    // Remove all lines that reference this point
    std::vector<EntityID> linesToRemove;
    for (const auto& line : lines) {
        if (line.startPt == id || line.endPt == id)
            linesToRemove.push_back(line.id);
    }
    for (EntityID lineID : linesToRemove) {
        removeLine(lineID);
    }

    // Remove all circles that reference this point
    std::vector<EntityID> circlesToRemove;
    for (const auto& circle : circles) {
        if (circle.centerPt == id)
            circlesToRemove.push_back(circle.id);
    }
    for (EntityID circleID : circlesToRemove) {
        removeCircle(circleID);
    }

    // Remove all arcs that reference this point
    std::vector<EntityID> arcsToRemove;
    for (const auto& arc : arcs) {
        if (arc.centerPt == id || arc.startPt == id || arc.endPt == id)
            arcsToRemove.push_back(arc.id);
    }
    for (EntityID arcID : arcsToRemove) {
        removeArc(arcID);
    }

    // Remove all ellipses that reference this point
    std::vector<EntityID> ellipsesToRemove;
    for (const auto& e : ellipses) {
        if (e.centerPt == id)
            ellipsesToRemove.push_back(e.id);
    }
    for (EntityID eid : ellipsesToRemove) {
        removeEllipse(eid);
    }

    // Remove all ellipse arcs that reference this point
    std::vector<EntityID> ellipseArcsToRemove;
    for (const auto& ea : ellipseArcs) {
        if (ea.centerPt == id || ea.startPt == id || ea.endPt == id)
            ellipseArcsToRemove.push_back(ea.id);
    }
    for (EntityID eaid : ellipseArcsToRemove) {
        removeEllipseArc(eaid);
    }

    // Remove all splines that reference this point
    std::vector<EntityID> splinesToRemove;
    for (const auto& sp : splines) {
        for (auto ptID : sp.controlPtIDs) {
            if (ptID == id) {
                splinesToRemove.push_back(sp.id);
                break;
            }
        }
    }
    for (EntityID spid : splinesToRemove) {
        removeSpline(spid);
    }

    // Remove constraints referencing this point
    removeConstraintsReferencing(id);

    // Remove the point itself
    points.erase(
        std::remove_if(points.begin(), points.end(),
            [id](const PointEntity& p) { return p.id == id; }),
        points.end());
    rebuildIndices();
}

void Sketch::removeLine(EntityID id) {
    LineEntity* line = findLine(id);
    if (!line) return;

    EntityID startPt = line->startPt;
    EntityID endPt = line->endPt;

    // Remove constraints referencing this line
    removeConstraintsReferencing(id);

    // Remove the line
    lines.erase(
        std::remove_if(lines.begin(), lines.end(),
            [id](const LineEntity& l) { return l.id == id; }),
        lines.end());

    // Remove orphaned points (never remove projected reference geometry)
    if (canRemovePoint(startPt)) {
        removeConstraintsReferencing(startPt);
        points.erase(
            std::remove_if(points.begin(), points.end(),
                [startPt](const PointEntity& p) { return p.id == startPt; }),
            points.end());
    }
    if (canRemovePoint(endPt)) {
        removeConstraintsReferencing(endPt);
        points.erase(
            std::remove_if(points.begin(), points.end(),
                [endPt](const PointEntity& p) { return p.id == endPt; }),
            points.end());
    }
    rebuildIndices();
}

void Sketch::removeCircle(EntityID id) {
    CircleEntity* circle = findCircle(id);
    if (!circle) return;

    EntityID centerPt = circle->centerPt;

    // Remove constraints referencing this circle
    removeConstraintsReferencing(id);

    // Remove the circle
    circles.erase(
        std::remove_if(circles.begin(), circles.end(),
            [id](const CircleEntity& c) { return c.id == id; }),
        circles.end());

    // Remove orphaned center point (never remove projected reference geometry)
    if (canRemovePoint(centerPt)) {
        removeConstraintsReferencing(centerPt);
        points.erase(
            std::remove_if(points.begin(), points.end(),
                [centerPt](const PointEntity& p) { return p.id == centerPt; }),
            points.end());
    }
    rebuildIndices();
}

void Sketch::removeArc(EntityID id) {
    ArcEntity* arc = findArc(id);
    if (!arc) return;

    EntityID cPt = arc->centerPt;
    EntityID sPt = arc->startPt;
    EntityID ePt = arc->endPt;

    removeConstraintsReferencing(id);

    arcs.erase(
        std::remove_if(arcs.begin(), arcs.end(),
            [id](const ArcEntity& a) { return a.id == id; }),
        arcs.end());

    // Remove orphaned points (never remove projected reference geometry)
    auto removeOrphan = [&](EntityID ptID) {
        if (canRemovePoint(ptID)) {
            removeConstraintsReferencing(ptID);
            points.erase(
                std::remove_if(points.begin(), points.end(),
                    [ptID](const PointEntity& p) { return p.id == ptID; }),
                points.end());
        }
    };
    removeOrphan(cPt);
    removeOrphan(sPt);
    removeOrphan(ePt);
    rebuildIndices();
}

void Sketch::removeEllipse(EntityID id) {
    EllipseEntity* e = findEllipse(id);
    if (!e) return;

    EntityID centerPt = e->centerPt;

    removeConstraintsReferencing(id);

    ellipses.erase(
        std::remove_if(ellipses.begin(), ellipses.end(),
            [id](const EllipseEntity& el) { return el.id == id; }),
        ellipses.end());

    if (canRemovePoint(centerPt)) {
        removeConstraintsReferencing(centerPt);
        points.erase(
            std::remove_if(points.begin(), points.end(),
                [centerPt](const PointEntity& p) { return p.id == centerPt; }),
            points.end());
    }
    rebuildIndices();
}

void Sketch::removeEllipseArc(EntityID id) {
    EllipseArcEntity* ea = findEllipseArc(id);
    if (!ea) return;

    EntityID cPt = ea->centerPt;
    EntityID sPt = ea->startPt;
    EntityID ePt = ea->endPt;

    removeConstraintsReferencing(id);

    ellipseArcs.erase(
        std::remove_if(ellipseArcs.begin(), ellipseArcs.end(),
            [id](const EllipseArcEntity& e) { return e.id == id; }),
        ellipseArcs.end());

    auto removeOrphan = [&](EntityID ptID) {
        if (canRemovePoint(ptID)) {
            removeConstraintsReferencing(ptID);
            points.erase(
                std::remove_if(points.begin(), points.end(),
                    [ptID](const PointEntity& p) { return p.id == ptID; }),
                points.end());
        }
    };
    removeOrphan(cPt);
    removeOrphan(sPt);
    removeOrphan(ePt);
    rebuildIndices();
}

void Sketch::removeSpline(EntityID id) {
    SplineEntity* sp = findSpline(id);
    if (!sp) return;

    std::vector<EntityID> ctrlPts = sp->controlPtIDs;

    removeConstraintsReferencing(id);

    splines.erase(
        std::remove_if(splines.begin(), splines.end(),
            [id](const SplineEntity& s) { return s.id == id; }),
        splines.end());

    for (EntityID ptID : ctrlPts) {
        if (canRemovePoint(ptID)) {
            removeConstraintsReferencing(ptID);
            points.erase(
                std::remove_if(points.begin(), points.end(),
                    [ptID](const PointEntity& p) { return p.id == ptID; }),
                points.end());
        }
    }
    rebuildIndices();
}

void Sketch::recomputeArcAngles(ArcEntity& arc) const {
    Point2D c = getPointPos(arc.centerPt);
    Point2D s = getPointPos(arc.startPt);
    Point2D e = getPointPos(arc.endPt);

    double newStart = std::atan2(s.y - c.y, s.x - c.x);
    double rawSweep = std::atan2(e.y - c.y, e.x - c.x) - newStart;
    while (rawSweep <= 0.0)     rawSweep += kTwoPiD;
    while (rawSweep > kTwoPiD)  rawSweep -= kTwoPiD;

    // A brand-new arc (addArc) has no previous sweep to stay continuous with.
    bool fresh = (arc.startAngle == 0.0 && arc.endAngle == 0.0);
    double sweep = rawSweep;
    if (!fresh) {
        double prevSweep = arc.endAngle - arc.startAngle;
        while (prevSweep <= 0.0)    prevSweep += kTwoPiD;
        while (prevSweep > kTwoPiD) prevSweep -= kTwoPiD;

        // Arcs are stored CCW with sweep in (0, 2pi], so the sweep is fully determined by the
        // three points — which means a single solve step that carries the end point across the
        // start ray silently turns a 5-degree arc into a 355-degree one. Pick whichever branch
        // is nearest the previous sweep and clamp at the degenerate floor instead, so the arc
        // collapses (recoverably) rather than inverting to its own complement.
        double alt = rawSweep - kTwoPiD;
        if (std::fabs(alt - prevSweep) < std::fabs(rawSweep - prevSweep)) sweep = alt;
        if (sweep < kMinArcSweep) sweep = kMinArcSweep;
        if (sweep > kTwoPiD)      sweep = kTwoPiD;
    }

    arc.startAngle = newStart;
    // Consumers all normalise (endAngle - startAngle) into (0, 2pi], so storing an endAngle
    // beyond +pi is both safe and what keeps the sweep unambiguous.
    arc.endAngle = newStart + sweep;
}

SketchGeometrySnapshot Sketch::captureGeometry() const {
    SketchGeometrySnapshot snap;
    snap.points      = points;
    snap.circles     = circles;
    snap.arcs        = arcs;
    snap.ellipses    = ellipses;
    snap.ellipseArcs = ellipseArcs;
    return snap;
}

void Sketch::restoreGeometry(const SketchGeometrySnapshot& snap) {
    points      = snap.points;
    circles     = snap.circles;
    arcs        = snap.arcs;
    ellipses    = snap.ellipses;
    ellipseArcs = snap.ellipseArcs;
    rebuildIndices();
}

Point2D Sketch::getPointPos(EntityID id) const {
    const PointEntity* p = findPoint(id);
    if (p) return {p->x, p->y};
    return {0.0, 0.0};
}

void Sketch::clear() {
    points.clear();
    lines.clear();
    circles.clear();
    arcs.clear();
    ellipses.clear();
    ellipseArcs.clear();
    splines.clear();
    constraints.clear();
    nextID = 1;
    dirty = true;
    pointIndex_.clear();
    lineIndex_.clear();
    circleIndex_.clear();
    arcIndex_.clear();
    ellipseIndex_.clear();
    ellipseArcIndex_.clear();
    splineIndex_.clear();
    constraintIndex_.clear();
}

// ─── Curve sampling utilities ──────────────────────────────────────

// Using kTwoPiD from Constants.h

std::vector<Point2D> sampleEllipse(Point2D center, double semiMajor, double semiMinor,
                                    double rotation, int numSamples) {
    std::vector<Point2D> pts;
    pts.reserve(numSamples + 1);
    double cosR = std::cos(rotation), sinR = std::sin(rotation);
    for (int i = 0; i <= numSamples; i++) {
        double a = kTwoPiD * i / numSamples;
        double ex = semiMajor * std::cos(a), ey = semiMinor * std::sin(a);
        pts.push_back({center.x + ex * cosR - ey * sinR,
                        center.y + ex * sinR + ey * cosR});
    }
    return pts;
}

std::vector<Point2D> sampleEllipseArc(Point2D center, double semiMajor, double semiMinor,
                                       double rotation, double startAngle, double endAngle,
                                       int numSamples) {
    double sweep = ccwSweep(startAngle, endAngle);
    int segs = std::max(8, numSamples);
    std::vector<Point2D> pts;
    pts.reserve(segs + 1);
    double cosR = std::cos(rotation), sinR = std::sin(rotation);
    for (int i = 0; i <= segs; i++) {
        double a = startAngle + sweep * i / segs;
        double ex = semiMajor * std::cos(a), ey = semiMinor * std::sin(a);
        pts.push_back({center.x + ex * cosR - ey * sinR,
                        center.y + ex * sinR + ey * cosR});
    }
    return pts;
}

std::vector<double> generateUniformKnots(int numCtrlPts, int degree) {
    int numKnots = numCtrlPts + degree + 1;
    std::vector<double> knots(numKnots);
    for (int i = 0; i <= degree; i++) knots[i] = 0.0;
    for (int i = degree + 1; i < numCtrlPts; i++)
        knots[i] = (double)(i - degree) / (numCtrlPts - degree);
    for (int i = numCtrlPts; i < numKnots; i++) knots[i] = 1.0;
    return knots;
}

Point2D evaluateBSpline(const std::vector<Point2D>& ctrlPts, const std::vector<double>& knots,
                         const std::vector<double>& weights, int degree, double t) {
    int n = (int)ctrlPts.size();
    if (n == 0) return {0, 0};
    if (n == 1) return ctrlPts[0];

    // Find knot span k such that knots[k] <= t < knots[k+1]
    int k = degree;
    for (int i = degree; i < n; i++) {
        if (t >= knots[i] && t < knots[i + 1]) { k = i; break; }
    }
    if (t >= knots[n]) k = n - 1;

    bool rational = !weights.empty();

    // De Boor's algorithm: work with (degree+1) points
    std::vector<double> dx(degree + 1), dy(degree + 1), dw(degree + 1);
    for (int j = 0; j <= degree; j++) {
        int idx = k - degree + j;
        if (idx < 0 || idx >= n) {
            dx[j] = dy[j] = 0; dw[j] = 1.0;
            continue;
        }
        double w = rational ? weights[idx] : 1.0;
        dx[j] = ctrlPts[idx].x * w;
        dy[j] = ctrlPts[idx].y * w;
        dw[j] = w;
    }

    for (int r = 1; r <= degree; r++) {
        for (int j = degree; j >= r; j--) {
            int i = k - degree + j;
            double denom = knots[i + degree - r + 1] - knots[i];
            if (std::fabs(denom) < 1e-10) continue;
            double alpha = (t - knots[i]) / denom;
            dx[j] = (1.0 - alpha) * dx[j - 1] + alpha * dx[j];
            dy[j] = (1.0 - alpha) * dy[j - 1] + alpha * dy[j];
            dw[j] = (1.0 - alpha) * dw[j - 1] + alpha * dw[j];
        }
    }

    double fw = dw[degree];
    if (std::fabs(fw) < 1e-10) fw = 1.0;
    return {dx[degree] / fw, dy[degree] / fw};
}

std::vector<Point2D> sampleSpline(const SplineEntity& sp, const Sketch& sketch, int numSamples) {
    std::vector<Point2D> ctrlPts;
    ctrlPts.reserve(sp.controlPtIDs.size());
    for (auto id : sp.controlPtIDs) ctrlPts.push_back(sketch.getPointPos(id));

    int n = (int)ctrlPts.size();
    if (n < 2) return ctrlPts;

    int deg = std::min(sp.degree, n - 1);

    std::vector<double> knots = sp.knots;
    if (knots.empty()) knots = generateUniformKnots(n, deg);

    // Validate knot vector size
    if ((int)knots.size() < n + deg + 1) knots = generateUniformKnots(n, deg);

    double tMin = knots[deg];
    double tMax = knots[n];

    std::vector<Point2D> pts;
    pts.reserve(numSamples + 1);
    for (int i = 0; i <= numSamples; i++) {
        double frac = (double)i / numSamples;
        double t = tMin + (tMax - tMin) * frac;
        // Clamp slightly before tMax to stay in valid range
        if (i == numSamples) t = tMax - 1e-6;
        pts.push_back(evaluateBSpline(ctrlPts, knots, sp.weights, deg, t));
    }
    return pts;
}

// ─── Constraint conflict detection ─────────────────────────────────

const char* constraintConflictReason(const Sketch& sketch, ConstraintType type,
                                     EntityID eA, EntityID eB, EntityID eC) {
    (void)eC;
    // Driven constraints are reference-only, so they can never contradict anything.
    auto unary = [&](ConstraintType t, EntityID e) -> bool {
        if (e == NullID) return false;
        for (const auto& c : sketch.constraints)
            if (!c.driven && c.type == t && c.entityA == e) return true;
        return false;
    };
    auto pair = [&](ConstraintType t, EntityID a, EntityID b) -> bool {
        if (a == NullID || b == NullID) return false;
        for (const auto& c : sketch.constraints) {
            if (c.driven || c.type != t) continue;
            if ((c.entityA == a && c.entityB == b) || (c.entityA == b && c.entityB == a))
                return true;
        }
        return false;
    };

    switch (type) {
        case ConstraintType::Horizontal:
            if (unary(ConstraintType::Vertical, eA))
                return "that line is already vertical";
            break;
        case ConstraintType::Vertical:
            if (unary(ConstraintType::Horizontal, eA))
                return "that line is already horizontal";
            break;

        case ConstraintType::Parallel:
            if (pair(ConstraintType::Perpendicular, eA, eB))
                return "those lines are already perpendicular";
            if (pair(ConstraintType::Angle, eA, eB))
                return "those lines already have an angle dimension";
            if ((unary(ConstraintType::Horizontal, eA) && unary(ConstraintType::Vertical, eB)) ||
                (unary(ConstraintType::Vertical, eA) && unary(ConstraintType::Horizontal, eB)))
                return "one line is horizontal and the other vertical";
            break;

        case ConstraintType::Perpendicular:
            if (pair(ConstraintType::Parallel, eA, eB))
                return "those lines are already parallel";
            if (pair(ConstraintType::Collinear, eA, eB))
                return "those lines are already collinear";
            if (pair(ConstraintType::Angle, eA, eB))
                return "those lines already have an angle dimension";
            if ((unary(ConstraintType::Horizontal, eA) && unary(ConstraintType::Horizontal, eB)) ||
                (unary(ConstraintType::Vertical, eA)   && unary(ConstraintType::Vertical, eB)))
                return "both lines already share the same direction";
            break;

        case ConstraintType::Collinear:
            if (pair(ConstraintType::Perpendicular, eA, eB))
                return "those lines are already perpendicular";
            if (pair(ConstraintType::Angle, eA, eB))
                return "those lines already have an angle dimension";
            break;

        case ConstraintType::Angle:
            if (pair(ConstraintType::Parallel, eA, eB))
                return "those lines are already parallel";
            if (pair(ConstraintType::Perpendicular, eA, eB))
                return "those lines are already perpendicular";
            if (pair(ConstraintType::Collinear, eA, eB))
                return "those lines are already collinear";
            if (pair(ConstraintType::Angle, eA, eB))
                return "those lines already have an angle dimension";
            break;

        case ConstraintType::Distance:
            if (unary(ConstraintType::Distance, eA))
                return "that line already has a length dimension";
            break;

        case ConstraintType::Radius:
        case ConstraintType::Diameter:
            if (unary(ConstraintType::Radius, eA) || unary(ConstraintType::Diameter, eA))
                return "that curve already has a radius or diameter dimension";
            break;

        case ConstraintType::EqualLength:
            if (pair(ConstraintType::EqualLength, eA, eB))
                return "those lines are already set equal";
            break;

        case ConstraintType::Concentric:
            if (pair(ConstraintType::Concentric, eA, eB))
                return "those curves are already concentric";
            break;

        default:
            break;
    }
    return nullptr;
}

} // namespace shitcad
