#pragma once
#include "SketchData.h"
#include "SketchPlane.h"
#include "ProfileDetector.h"
#include "ExtrudeTool.h"
#include "MeshImport.h"
#include <vector>
#include <string>
#include <variant>
#include <set>
#include <cstdint>

namespace shitcad {

using FeatureID = uint32_t;
constexpr FeatureID NullFeatureID = 0;

enum class FeatureType : uint8_t {
    Sketch,
    Extrude,
    Revolve,
    Loft,
    Boolean,
    MeshImport,
};

// Signature for matching profiles across sketch edits
struct ProfileSignature {
    std::set<EntityID> lineIDs;
    std::set<EntityID> arcIDs;
    std::set<EntityID> ellipseIDs;
    std::set<EntityID> splineIDs;
    EntityID circleID = NullID;
    EntityID ellipseID = NullID; // for full-ellipse profiles (future)
    double centroidX = 0; // boundary centroid for disambiguation
    double centroidY = 0;

    static ProfileSignature fromProfile(const ClosedProfile& profile, const Sketch& sketch);
    bool matches(const ClosedProfile& profile, const Sketch& sketch) const;
    float centroidDistTo(const ClosedProfile& profile, const Sketch& sketch) const;
};

struct SketchFeatureData {
    int sketchPlaneIndex = -1;
    PlaneID sketchPlaneID = NullPlaneID; // stable ID (survives reordering)
    Sketch sketchSnapshot; // full copy after edit session
};

struct ExtrudeFeatureData {
    FeatureID sourceSketchFeature = NullFeatureID;
    std::vector<ProfileSignature> profileSigs;
    std::vector<int> profileIndicesFallback;
    float height = 10.0f;
    float offset = 0.0f;
    ExtrudeOperation operation = ExtrudeOperation::NewBody;
    ExtrudeDirection direction = ExtrudeDirection::OneSide;
};

struct RevolveFeatureData {
    FeatureID sourceSketchFeature = NullFeatureID;
    std::vector<ProfileSignature> profileSigs;
    std::vector<int> profileIndicesFallback;
    EntityID axisLineID = NullID; // sketch line used as revolution axis
    float angleDeg = 360.0f;
    ExtrudeOperation operation = ExtrudeOperation::NewBody;
};

// Loft connects profiles on different sketch planes into a smooth solid.
// Each section is identified by a sketch plane + profile signature.
struct LoftSection {
    FeatureID sourceSketchFeature = NullFeatureID;
    ProfileSignature profileSig;
    int profileIndexFallback = 0;
};

struct LoftFeatureData {
    std::vector<LoftSection> sections; // >= 2 sections
    ExtrudeOperation operation = ExtrudeOperation::NewBody;
    bool solid = true; // true = solid, false = shell (hollow)
};

// BooleanOperation is declared in ExtrudeTool.h (alongside ExtrudeOperation)

struct BooleanFeatureData {
    BooleanOperation operation = BooleanOperation::Union;
    // Body identification by feature ID chain: we store the feature IDs that
    // created the bodies, which is stable across replays (body indices change).
    std::vector<FeatureID> targetBodyFeatures;  // features that produced the target body
    std::vector<FeatureID> toolBodyFeatures;     // features that produced the tool body
    // Fallback: body indices at commit time (used if feature matching fails)
    int targetBodyIndex = -1;
    int toolBodyIndex = -1;
};

// A triangle mesh referenced from disk (STL) rather than copied into the
// project. Replay re-reads the file, so an updated export (e.g. from Onshape)
// is picked up without re-importing, and the file is never re-triangulated.
// The body it produces is mesh-only: Body3D::shape is null.
struct MeshImportFeatureData {
    std::string sourcePath;   // absolute path to the STL
    std::string unit = "mm";  // unit of the file's coordinates (a kUnits name)
    MeshTransform transform;  // placement in the model, applied after unit scaling
};

struct Feature {
    FeatureID id = NullFeatureID;
    FeatureType type = FeatureType::Sketch;
    std::string name;
    bool suppressed = false;
    std::variant<SketchFeatureData, ExtrudeFeatureData, RevolveFeatureData, LoftFeatureData,
                 BooleanFeatureData, MeshImportFeatureData> data;
    bool hasError = false;
    std::string errorMsg;
};

class FeatureHistory {
public:
    FeatureID addSketchFeature(int planeIndex, const Sketch& snapshot, PlaneID planeID = NullPlaneID);
    FeatureID addExtrudeFeature(const ExtrudeFeatureData& params);
    FeatureID addRevolveFeature(const RevolveFeatureData& params);
    FeatureID addLoftFeature(const LoftFeatureData& params);
    FeatureID addBooleanFeature(const BooleanFeatureData& params);
    FeatureID addMeshImportFeature(const MeshImportFeatureData& params, const std::string& name);

    void updateSketchSnapshot(FeatureID id, const Sketch& snapshot);
    void updateExtrudeData(FeatureID id, const ExtrudeFeatureData& data);
    void updateRevolveData(FeatureID id, const RevolveFeatureData& data);
    void updateLoftData(FeatureID id, const LoftFeatureData& data);
    void updateBooleanData(FeatureID id, const BooleanFeatureData& data);
    void updateMeshImportData(FeatureID id, const MeshImportFeatureData& data);
    FeatureID findSketchFeatureForPlane(int planeIndex) const;

    std::vector<FeatureID> getDependents(FeatureID id) const;

    void suppressFeature(FeatureID id);
    void unsuppressFeature(FeatureID id);
    void deleteFeature(FeatureID id); // cascades to dependents
    void removeFeature(FeatureID id); // remove single feature, no cascade (for undo)
    void insertFeatureAt(int index, const Feature& feat); // re-insert at position (for undo)
    void renameFeature(FeatureID id, const std::string& newName);

    int rollbackPos() const { return rollbackPos_; }
    void setRollbackPos(int pos);
    bool isRolledBack(int featureIndex) const;

    const std::vector<Feature>& features() const { return features_; }
    std::vector<Feature>& features() { return features_; }
    Feature* findFeature(FeatureID id);
    const Feature* findFeature(FeatureID id) const;
    int featureIndex(FeatureID id) const;

    bool empty() const { return features_.empty(); }
    size_t size() const { return features_.size(); }

    // Accessors for serialization
    FeatureID nextID() const { return nextID_; }
    void setNextID(FeatureID id) { nextID_ = id; }
    int sketchCounter() const { return sketchCounter_; }
    void setSketchCounter(int c) { sketchCounter_ = c; }
    int extrudeCounter() const { return extrudeCounter_; }
    void setExtrudeCounter(int c) { extrudeCounter_ = c; }
    int revolveCounter() const { return revolveCounter_; }
    void setRevolveCounter(int c) { revolveCounter_ = c; }
    int loftCounter() const { return loftCounter_; }
    void setLoftCounter(int c) { loftCounter_ = c; }
    int booleanCounter() const { return booleanCounter_; }
    void setBooleanCounter(int c) { booleanCounter_ = c; }

    void clear() {
        features_.clear();
        nextID_ = 1;
        rollbackPos_ = -1;
        sketchCounter_ = 0;
        extrudeCounter_ = 0;
        revolveCounter_ = 0;
        loftCounter_ = 0;
        booleanCounter_ = 0;
    }

private:
    std::vector<Feature> features_;
    FeatureID nextID_ = 1;
    int rollbackPos_ = -1; // -1 = all visible
    int sketchCounter_ = 0;
    int extrudeCounter_ = 0;
    int revolveCounter_ = 0;
    int loftCounter_ = 0;
    int booleanCounter_ = 0;
};

} // namespace shitcad
