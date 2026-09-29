#include "FeatureHistory.h"
#include <algorithm>
#include <cmath>

namespace shitcad {

// --- ProfileSignature ---

ProfileSignature ProfileSignature::fromProfile(const ClosedProfile& profile, const Sketch& sketch) {
    ProfileSignature sig;
    sig.circleID = profile.circleID;
    for (auto id : profile.lineIDs) sig.lineIDs.insert(id);
    for (auto id : profile.ellipseIDs) sig.ellipseIDs.insert(id);
    for (auto id : profile.splineIDs) sig.splineIDs.insert(id);
    for (const auto& seg : profile.segments) {
        if (seg.type == SegmentType::Arc && seg.origCircleID != NullID)
            sig.arcIDs.insert(seg.origCircleID);
        if ((seg.type == SegmentType::Ellipse || seg.type == SegmentType::EllipseArc) && seg.origEllipseID != NullID)
            sig.ellipseIDs.insert(seg.origEllipseID);
        if (seg.type == SegmentType::Spline && seg.origSplineID != NullID)
            sig.splineIDs.insert(seg.origSplineID);
    }
    if (!profile.isCircle()) {
        Point2D c = polygonCentroid(tessellateProfile(sketch, profile));
        sig.centroidX = c.x;
        sig.centroidY = c.y;
    }
    return sig;
}

bool ProfileSignature::matches(const ClosedProfile& profile, const Sketch& sketch) const {
    // Circle match: exact
    if (circleID != NullID || profile.circleID != NullID) {
        return circleID == profile.circleID;
    }

    // First try Jaccard similarity on edge IDs (all entity types)
    std::set<EntityID> myEdges = lineIDs;
    myEdges.insert(arcIDs.begin(), arcIDs.end());
    myEdges.insert(ellipseIDs.begin(), ellipseIDs.end());
    myEdges.insert(splineIDs.begin(), splineIDs.end());

    std::set<EntityID> otherEdges(profile.lineIDs.begin(), profile.lineIDs.end());
    otherEdges.insert(profile.ellipseIDs.begin(), profile.ellipseIDs.end());
    otherEdges.insert(profile.splineIDs.begin(), profile.splineIDs.end());
    for (const auto& seg : profile.segments) {
        if (seg.type == SegmentType::Arc && seg.origCircleID != NullID)
            otherEdges.insert(seg.origCircleID);
        if ((seg.type == SegmentType::Ellipse || seg.type == SegmentType::EllipseArc) && seg.origEllipseID != NullID)
            otherEdges.insert(seg.origEllipseID);
        if (seg.type == SegmentType::Spline && seg.origSplineID != NullID)
            otherEdges.insert(seg.origSplineID);
    }

    if (!myEdges.empty() && !otherEdges.empty()) {
        std::set<EntityID> intersection;
        std::set_intersection(myEdges.begin(), myEdges.end(),
                             otherEdges.begin(), otherEdges.end(),
                             std::inserter(intersection, intersection.begin()));

        std::set<EntityID> unionSet;
        std::set_union(myEdges.begin(), myEdges.end(),
                      otherEdges.begin(), otherEdges.end(),
                      std::inserter(unionSet, unionSet.begin()));

        if (!unionSet.empty()) {
            double jaccard = (double)intersection.size() / (double)unionSet.size();
            if (jaccard > 0.7) return true;
        }
    }

    // Fallback: centroid proximity (handles projected geometry where IDs change)
    // Match if centroids are very close and segment counts are similar
    Point2D c = polygonCentroid(tessellateProfile(sketch, profile));
    double dx = c.x - centroidX;
    double dy = c.y - centroidY;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 0.5) {
        // Also check segment count similarity
        int mySeg = (int)(lineIDs.size() + arcIDs.size() + ellipseIDs.size() + splineIDs.size());
        int otherSeg = (int)(profile.lineIDs.size() + profile.ellipseIDs.size() + profile.splineIDs.size() + profile.segments.size());
        if (mySeg == 0 || otherSeg == 0) return dist < 0.1;
        double ratio = (double)std::min(mySeg, otherSeg) / std::max(mySeg, otherSeg);
        return ratio > 0.5;
    }

    return false;
}

float ProfileSignature::centroidDistTo(const ClosedProfile& profile, const Sketch& sketch) const {
    Point2D c = polygonCentroid(tessellateProfile(sketch, profile));
    double dx = c.x - centroidX;
    double dy = c.y - centroidY;
    return static_cast<float>(std::sqrt(dx * dx + dy * dy));
}

// --- FeatureHistory ---

FeatureID FeatureHistory::addSketchFeature(int planeIndex, const Sketch& snapshot, PlaneID planeID) {
    Feature f;
    f.id = nextID_++;
    f.type = FeatureType::Sketch;
    f.name = "Sketch" + std::to_string(++sketchCounter_);
    f.data = SketchFeatureData{planeIndex, planeID, snapshot};
    features_.push_back(std::move(f));
    return features_.back().id;
}

FeatureID FeatureHistory::addExtrudeFeature(const ExtrudeFeatureData& params) {
    Feature f;
    f.id = nextID_++;
    f.type = FeatureType::Extrude;
    if (params.operation == ExtrudeOperation::Cut) {
        f.name = "Cut" + std::to_string(++extrudeCounter_);
    } else {
        f.name = "Extrude" + std::to_string(++extrudeCounter_);
    }
    f.data = params;
    features_.push_back(std::move(f));
    return features_.back().id;
}

FeatureID FeatureHistory::addRevolveFeature(const RevolveFeatureData& params) {
    Feature f;
    f.id = nextID_++;
    f.type = FeatureType::Revolve;
    if (params.operation == ExtrudeOperation::Cut) {
        f.name = "RevolveCut" + std::to_string(++revolveCounter_);
    } else {
        f.name = "Revolve" + std::to_string(++revolveCounter_);
    }
    f.data = params;
    features_.push_back(std::move(f));
    return features_.back().id;
}

void FeatureHistory::updateSketchSnapshot(FeatureID id, const Sketch& snapshot) {
    Feature* f = findFeature(id);
    if (!f || f->type != FeatureType::Sketch) return;
    std::get<SketchFeatureData>(f->data).sketchSnapshot = snapshot;
}

void FeatureHistory::updateExtrudeData(FeatureID id, const ExtrudeFeatureData& data) {
    Feature* f = findFeature(id);
    if (!f || f->type != FeatureType::Extrude) return;
    std::get<ExtrudeFeatureData>(f->data) = data;
}

void FeatureHistory::updateRevolveData(FeatureID id, const RevolveFeatureData& data) {
    Feature* f = findFeature(id);
    if (!f || f->type != FeatureType::Revolve) return;
    std::get<RevolveFeatureData>(f->data) = data;
}

void FeatureHistory::updateLoftData(FeatureID id, const LoftFeatureData& data) {
    Feature* f = findFeature(id);
    if (!f || f->type != FeatureType::Loft) return;
    std::get<LoftFeatureData>(f->data) = data;
}

FeatureID FeatureHistory::addBooleanFeature(const BooleanFeatureData& params) {
    Feature f;
    f.id = nextID_++;
    f.type = FeatureType::Boolean;
    if (params.operation == BooleanOperation::Subtract) {
        f.name = "Subtract" + std::to_string(++booleanCounter_);
    } else {
        f.name = "Union" + std::to_string(++booleanCounter_);
    }
    f.data = params;
    features_.push_back(std::move(f));
    return features_.back().id;
}

void FeatureHistory::updateBooleanData(FeatureID id, const BooleanFeatureData& data) {
    Feature* f = findFeature(id);
    if (!f || f->type != FeatureType::Boolean) return;
    std::get<BooleanFeatureData>(f->data) = data;
}

FeatureID FeatureHistory::addMeshImportFeature(const MeshImportFeatureData& params,
                                               const std::string& name) {
    Feature f;
    f.id = nextID_++;
    f.type = FeatureType::MeshImport;
    // Named after the file rather than a counter: the name is what identifies
    // the surface when the model is handed to a simulation.
    f.name = name;
    f.data = params;
    features_.push_back(std::move(f));
    return features_.back().id;
}

void FeatureHistory::updateMeshImportData(FeatureID id, const MeshImportFeatureData& data) {
    Feature* f = findFeature(id);
    if (!f || f->type != FeatureType::MeshImport) return;
    std::get<MeshImportFeatureData>(f->data) = data;
}

FeatureID FeatureHistory::addCadImportFeature(const CadImportFeatureData& params,
                                              const std::string& name) {
    Feature f;
    f.id = nextID_++;
    f.type = FeatureType::CadImport;
    f.name = name; // the file stem, like a mesh import
    f.data = params;
    features_.push_back(std::move(f));
    return features_.back().id;
}

void FeatureHistory::updateCadImportData(FeatureID id, const CadImportFeatureData& data) {
    Feature* f = findFeature(id);
    if (!f || f->type != FeatureType::CadImport) return;
    std::get<CadImportFeatureData>(f->data) = data;
}

FeatureID FeatureHistory::addLoftFeature(const LoftFeatureData& params) {
    Feature f;
    f.id = nextID_++;
    f.type = FeatureType::Loft;
    f.name = "Loft" + std::to_string(++loftCounter_);
    f.data = params;
    features_.push_back(std::move(f));
    return features_.back().id;
}

FeatureID FeatureHistory::findSketchFeatureForPlane(int planeIndex) const {
    for (const auto& f : features_) {
        if (f.type == FeatureType::Sketch) {
            const auto& sd = std::get<SketchFeatureData>(f.data);
            if (sd.sketchPlaneIndex == planeIndex) return f.id;
        }
    }
    return NullFeatureID;
}

std::vector<FeatureID> FeatureHistory::getDependents(FeatureID id) const {
    std::vector<FeatureID> deps;
    for (const auto& f : features_) {
        if (f.type == FeatureType::Extrude) {
            const auto& ed = std::get<ExtrudeFeatureData>(f.data);
            if (ed.sourceSketchFeature == id) deps.push_back(f.id);
        } else if (f.type == FeatureType::Revolve) {
            const auto& rd = std::get<RevolveFeatureData>(f.data);
            if (rd.sourceSketchFeature == id) deps.push_back(f.id);
        } else if (f.type == FeatureType::Loft) {
            const auto& ld = std::get<LoftFeatureData>(f.data);
            for (const auto& sec : ld.sections) {
                if (sec.sourceSketchFeature == id) { deps.push_back(f.id); break; }
            }
        }
    }
    return deps;
}

void FeatureHistory::suppressFeature(FeatureID id) {
    Feature* f = findFeature(id);
    if (f) f->suppressed = true;
}

void FeatureHistory::unsuppressFeature(FeatureID id) {
    Feature* f = findFeature(id);
    if (f) f->suppressed = false;
}

void FeatureHistory::deleteFeature(FeatureID id) {
    // Cascade: delete dependents first
    auto deps = getDependents(id);
    for (auto depID : deps) deleteFeature(depID);

    features_.erase(
        std::remove_if(features_.begin(), features_.end(),
                       [id](const Feature& f) { return f.id == id; }),
        features_.end());
}

void FeatureHistory::removeFeature(FeatureID id) {
    features_.erase(
        std::remove_if(features_.begin(), features_.end(),
                       [id](const Feature& f) { return f.id == id; }),
        features_.end());
}

void FeatureHistory::insertFeatureAt(int index, const Feature& feat) {
    if (index < 0) index = 0;
    if (index > (int)features_.size()) index = (int)features_.size();
    features_.insert(features_.begin() + index, feat);
    // Ensure nextID_ stays ahead of any re-inserted feature
    if (feat.id >= nextID_) nextID_ = feat.id + 1;
}

void FeatureHistory::renameFeature(FeatureID id, const std::string& newName) {
    Feature* f = findFeature(id);
    if (f) f->name = newName;
}

void FeatureHistory::setRollbackPos(int pos) {
    rollbackPos_ = pos;
}

bool FeatureHistory::isRolledBack(int featureIndex) const {
    if (rollbackPos_ < 0) return false;
    return featureIndex > rollbackPos_;
}

Feature* FeatureHistory::findFeature(FeatureID id) {
    for (auto& f : features_) {
        if (f.id == id) return &f;
    }
    return nullptr;
}

const Feature* FeatureHistory::findFeature(FeatureID id) const {
    for (const auto& f : features_) {
        if (f.id == id) return &f;
    }
    return nullptr;
}

int FeatureHistory::featureIndex(FeatureID id) const {
    for (int i = 0; i < (int)features_.size(); i++) {
        if (features_[i].id == id) return i;
    }
    return -1;
}

} // namespace shitcad
