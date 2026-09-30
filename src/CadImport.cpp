#include "CadImport.h"
#include "Utf8Path.h"

#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <IGESCAFControl_Reader.hxx>
#include <IGESData_GlobalSection.hxx>
#include <IGESData_IGESModel.hxx>
#include <Quantity_Color.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFPrs_DocumentExplorer.hxx>
#include <gp_Mat.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <map>
#include <memory>

namespace shitcad {

bool isCadFile(const std::string& path) {
    auto dot = path.rfind('.');
    if (dot == std::string::npos) return false;
    std::string ext = path.substr(dot);
    for (auto& ch : ext) ch = (char)std::tolower((unsigned char)ch);
    return ext == ".step" || ext == ".stp" || ext == ".igs" || ext == ".iges";
}

static bool isIges(const std::string& path) {
    auto dot = path.rfind('.');
    std::string ext = dot == std::string::npos ? "" : path.substr(dot);
    for (auto& ch : ext) ch = (char)std::tolower((unsigned char)ch);
    return ext == ".igs" || ext == ".iges";
}

double tessellationDeflection(const double lo[3], const double hi[3]) {
    const double dx = hi[0] - lo[0], dy = hi[1] - lo[1], dz = hi[2] - lo[2];
    const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
    return std::max(0.01, diag * 0.0002);
}

namespace {

// Angular tolerance of an import's display tessellation. Looser than the 0.1
// rad Scene3D uses for modelled parts: exported parts are often covered in
// small holes, and at 0.1 each one took 63 segments (a 1.3 m sparger meshed
// into a million vertices in 12 s). Large curves are held by the linear
// tolerance, which is the stricter of the two for them.
constexpr double kAngularDeflection = 0.25;

struct CachedCad {
    FileStamp stamp;
    std::string error;        // the read failed: parsing it again gives the same answer
    std::vector<CadPart> parts;
    CadFileInfo info;
};

// Each entry holds every part's B-rep and its triangulation, which is large for
// a real assembly, so the cache is bounded: the kMaxCachedFiles most recently
// used files stay, the rest are re-read when asked for again. Replay touches
// every import in a project each time, so a project stays cached as long as it
// has no more imports than that.
constexpr size_t kMaxCachedFiles = 8;

struct CacheSlot {
    std::shared_ptr<const CachedCad> entry;
    uint64_t lastUse = 0;
};

std::map<std::string, CacheSlot>& cache() {
    static std::map<std::string, CacheSlot> c;
    return c;
}

void evictOldest(std::map<std::string, CacheSlot>& c, const std::string& keep) {
    while (c.size() > kMaxCachedFiles) {
        auto oldest = c.end();
        for (auto it = c.begin(); it != c.end(); ++it)
            if (it->first != keep && (oldest == c.end() || it->second.lastUse < oldest->second.lastUse)) oldest = it;
        if (oldest == c.end()) return;
        c.erase(oldest);
    }
}

std::string labelName(const TDF_Label& label) {
    if (label.IsNull()) return {};
    Handle(TDataStd_Name) name;
    if (!label.FindAttribute(TDataStd_Name::GetID(), name)) return {};
    // replaceNonAscii = 0 converts to UTF-8
    return TCollection_AsciiString(name->Get()).ToCString();
}

std::string lowerUnit(std::string s) {
    for (auto& ch : s) ch = (char)std::tolower((unsigned char)ch);
    if (s == "millimetre" || s == "millimeter" || s == "mm") return "mm";
    if (s == "centimetre" || s == "centimeter" || s == "cm") return "cm";
    if (s == "metre" || s == "meter" || s == "m") return "m";
    if (s == "inch" || s == "in") return "inch";
    if (s == "foot" || s == "ft") return "ft";
    return s;
}

// Split one leaf of the file into the bodies it becomes: each solid, each
// shell outside a solid, and every face outside a shell grouped as one body.
// Curves and points make no body; they are only counted.
void splitIntoParts(const TopoDS_Shape& shape, const std::string& name, const XCAFPrs_Style& style,
                    std::vector<CadPart>& out, CadFileInfo& info) {
    std::vector<TopoDS_Shape> pieces;
    int solids = 0, surfaces = 0;
    for (TopExp_Explorer e(shape, TopAbs_SOLID); e.More(); e.Next()) {
        pieces.push_back(e.Current());
        solids++;
    }
    for (TopExp_Explorer e(shape, TopAbs_SHELL, TopAbs_SOLID); e.More(); e.Next()) {
        pieces.push_back(e.Current());
        surfaces++;
    }
    BRep_Builder builder;
    TopoDS_Compound loose;
    bool anyLoose = false;
    for (TopExp_Explorer e(shape, TopAbs_FACE, TopAbs_SHELL); e.More(); e.Next()) {
        if (!anyLoose) builder.MakeCompound(loose);
        builder.Add(loose, e.Current());
        anyLoose = true;
    }
    if (anyLoose) {
        pieces.push_back(loose);
        surfaces++;
    }
    if (pieces.empty() && TopExp_Explorer(shape, TopAbs_EDGE).More()) info.skippedWires++;

    info.solids += solids;
    info.surfaces += surfaces;
    for (size_t i = 0; i < pieces.size(); i++) {
        CadPart part;
        part.shape = pieces[i];
        part.name = name;
        if (pieces.size() > 1 && !name.empty()) part.name += " (" + std::to_string(i + 1) + ")";
        if (style.IsSetColorSurf()) {
            double r, g, b;
            style.GetColorSurf().Values(r, g, b, Quantity_TOC_sRGB);
            part.hasColor = true;
            part.color[0] = (float)r; part.color[1] = (float)g; part.color[2] = (float)b;
        }
        out.push_back(std::move(part));
    }
}

bool readFile(const std::string& path, CachedCad& out, std::string& error) {
    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) doc;
    app->NewDocument("MDTV-XCAF", doc);
    // Closed on every way out, including an OCCT exception unwinding to
    // loadCadFile: the document holds the whole transferred model.
    struct DocGuard {
        Handle(XCAFApp_Application) app;
        Handle(TDocStd_Document) doc;
        bool open = true;
        void close() { if (open) { app->Close(doc); open = false; } }
        ~DocGuard() { close(); }
    } guard{app, doc};
    // Everything in SHITcad is mm; the readers convert from the file's unit
    // to the document's.
    XCAFDoc_DocumentTool::SetLengthUnit(doc, 1.0, UnitsMethods_LengthUnit_Millimeter);

    // OCCT's const char* file functions take UTF-8 (see Utf8Path.h).
    if (isIges(path)) {
        out.info.format = "IGES";
        IGESCAFControl_Reader reader;
        reader.SetColorMode(true);
        reader.SetNameMode(true);
        reader.SetLayerMode(false);
        if (reader.ReadFile(path.c_str()) != IFSelect_RetDone) {
            error = "Not a readable IGES file: " + path;
            return false;
        }
        Handle(IGESData_IGESModel) model = reader.IGESModel();
        if (!model.IsNull() && !model->GlobalSection().UnitName().IsNull())
            out.info.fileUnit = lowerUnit(model->GlobalSection().UnitName()->ToCString());
        if (!reader.Transfer(doc)) {
            error = "Could not convert the IGES file to shapes: " + path;
            return false;
        }
    } else {
        out.info.format = "STEP";
        STEPCAFControl_Reader reader;
        reader.SetColorMode(true);
        reader.SetNameMode(true);
        reader.SetLayerMode(false);
        reader.SetPropsMode(false);
        if (reader.ReadFile(path.c_str()) != IFSelect_RetDone) {
            error = "Not a readable STEP file: " + path;
            return false;
        }
        NCollection_Sequence<TCollection_AsciiString> lengths, angles, solidAngles;
        reader.ChangeReader().FileUnits(lengths, angles, solidAngles);
        if (!lengths.IsEmpty()) out.info.fileUnit = lowerUnit(lengths.First().ToCString());
        if (!reader.Transfer(doc)) {
            error = "Could not convert the STEP file to shapes: " + path;
            return false;
        }
    }

    // Leaves of the assembly tree, each with its absolute placement, so an
    // assembly that uses one part several times gets every instance.
    for (XCAFPrs_DocumentExplorer expl(doc, XCAFPrs_DocumentExplorerFlags_OnlyLeafNodes); expl.More(); expl.Next()) {
        const XCAFPrs_DocumentNode& node = expl.Current();
        TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(node.RefLabel);
        if (shape.IsNull()) continue;
        shape = shape.Located(node.Location);
        std::string name = labelName(node.RefLabel);
        if (name.empty()) name = labelName(node.Label);
        splitIntoParts(shape, name, node.Style, out.parts, out.info);
    }
    guard.close(); // the shapes outlive it; meshing below does not need it

    if (out.parts.empty()) {
        error = out.info.skippedWires
            ? "The file has only curves or points - nothing that makes a body: " + path
            : "The file contains no shapes: " + path;
        return false;
    }

    // Tessellate once, sized to the whole model; replays share it.
    Bnd_Box box;
    for (const auto& p : out.parts) BRepBndLib::Add(p.shape, box);
    if (box.IsVoid()) {
        error = "The file's shapes have no extent: " + path;
        return false;
    }
    double lo[3], hi[3];
    box.Get(lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]);
    out.info.deflection = tessellationDeflection(lo, hi);
    // All parts in one call, so OCCT can mesh faces in parallel. It returns
    // when done: nothing runs past this call, so the app stays single-threaded.
    BRep_Builder builder;
    TopoDS_Compound all;
    builder.MakeCompound(all);
    for (const auto& p : out.parts) builder.Add(all, p.shape);
    BRepMesh_IncrementalMesh mesher(all, out.info.deflection, false, kAngularDeflection, true);
    mesher.Perform();
    cadPlacedBounds(out.parts, MeshTransform{}, out.info.lo, out.info.hi);
    return true;
}

} // namespace

bool loadCadFile(const std::string& path, std::vector<CadPart>& parts, CadFileInfo& info,
                 std::string& error) {
    FileStamp stamp;
    if (!stampFile(path, stamp, error)) return false;
    static uint64_t tick = 0;
    auto& c = cache();
    auto it = c.find(path);
    if (it == c.end() || !(it->second.entry->stamp == stamp)) {
        auto entry = std::make_shared<CachedCad>();
        entry->stamp = stamp;
        // OCCT reports malformed files by throwing; nothing above this line does.
        try {
            if (!readFile(path, *entry, entry->error) && entry->error.empty())
                entry->error = "Failed to read " + path;
        } catch (const Standard_Failure& e) {
            entry->error = std::string("Failed to read ") + path + ": " + e.GetMessageString();
        }
        // Failures are cached too, until the file changes: replay asks on every
        // edit and the Place panel on every frame, and re-parsing a broken
        // 50 MB STEP each time froze the app.
        if (!entry->error.empty()) entry->parts.clear();
        it = c.insert_or_assign(path, CacheSlot{entry, 0}).first;
    }
    it->second.lastUse = ++tick;
    const std::shared_ptr<const CachedCad> hit = it->second.entry; // survives the eviction below
    evictOldest(c, path);
    if (!hit->error.empty()) {
        error = hit->error;
        return false;
    }
    parts = hit->parts;
    info = hit->info;
    return true;
}

TopoDS_Shape placeCadShape(const TopoDS_Shape& shape, const MeshTransform& xf) {
    if (xf.isIdentity()) return shape;
    // Through a quaternion so the location is a pure rotation: a matrix
    // composed of many turns is orthonormal only to ~1e-16, and OCCT treats a
    // location with a scale factor of 0.9999999999999998 as scaled.
    const double* r = xf.r;
    gp_Mat m(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
    gp_Trsf trsf;
    trsf.SetRotation(gp_Quaternion(m));
    trsf.SetTranslationPart(gp_Vec(xf.t[0], xf.t[1], xf.t[2]));
    return shape.Moved(TopLoc_Location(trsf));
}

bool cadPlacedBounds(const std::vector<CadPart>& parts, const MeshTransform& xf,
                     double lo[3], double hi[3]) {
    Bnd_Box box;
    for (const auto& p : parts) {
        // From the triangulation, without the shapes' tolerance gap, so "drop
        // to ground" puts the lowest point at exactly zero.
        BRepBndLib::AddOptimal(placeCadShape(p.shape, xf), box, true, false);
    }
    if (box.IsVoid()) return false;
    box.Get(lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]);
    return true;
}

} // namespace shitcad
