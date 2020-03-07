#ifndef UTILITIES_OPTIONS_H
#define UTILITIES_OPTIONS_H

#include <boost/program_options.hpp>
#include <string>
#include "d3derr.h"

const char* const VERSION_STRING = "D3D_version = DEV-2.22";

namespace po = boost::program_options;

namespace d3d {

struct IOOptions {
    std::string input;
    std::string outputFolder;
    std::string outputPath;
    int verboseLevel;
    po::options_description getCommandLine();
};

struct HealingOptions {
    int coordinate_precision = 4;  // controls the precision for written output
    bool binary = false;

    bool verbose =
        false;  // if set to true, there will be more output on screen
    bool remesh_intersections =
        false;  // if set to true, this happens before anything else. This is an
                // alternative provided by the polygonica support team to their
                // general self-intersection fixing function
    bool remove_noise_shells =
        true;  // during healing, eliminate small disconnected mesh pieces
    bool make_manifold = true;  // make solid manifold
    bool make_closed = true;    // close solid if necessary
    bool fix_orientation =
        true;  // check and fix the orientation of all faces of the solid
    bool fix_intersections =
        true;  // use Polygonica's self-intersections fixing

    bool useJournaling = false;
    std::string debugOutputLog;

    po::options_description getCommandLine();
};

struct OffsetOptions {
    float offsetValue = 0.0;
    std::string offsetFile = "";
    float voxelSize = 1.0;

    po::options_description getCommandLine();
};

struct TetraOptions {
    double mquality = 1.2;  //  radius - edge ratio
    double volumeBound = -1.0;
    int optimizationLevel = 2;

    po::options_description getCommandLine();
};

enum class FreezeConstraints {
    Ignore,
    NonDesignPID,
    IsosurfaceDistances,
    ByEuclideanDistance,
    MarkedNodesTxt,
    SubsetNodesFEM,
    NonDesignInterface,
    FreezeAndMerge
};
std::istream& operator>>(std::istream& in,
                         FreezeConstraints& freezeConstraints);
std::ostream& operator<<(std::ostream& out,
                         const FreezeConstraints& freezeConstraints);

struct ConstraintOptions {
    std::string freezeFile;
    FreezeConstraints freezeMethod;
    int freezePID = -1;
    bool disableShrinkPatch = false;
};
enum class Smoother {
    Laplacian = 1,
    Taubin,
    Bilaplacian,
    MCF,
    TaubinMCF,
    BilaplacianMCF,
    Membrane,
    ThinPlate,
    MinimumVariation,
    Hybrid,
    Repair
};

std::istream& operator>>(std::istream& in, Smoother& smoothingMethod);
std::ostream& operator<<(std::ostream& out, const Smoother& smoothingMethod);

struct SmoothOptions {
    Smoother smoothMethod;
    double msize = -1.0;  // -1.0: compute size; 0: skip remeshing
    double msizescale = 1.1;
    int nstep = 4;
    ConstraintOptions constraints;

    po::options_description getCommandLine();
};

struct IsosurfaceOptions {
    std::string level;
    std::string densityField;
    std::string densityDomainDimension;
    bool heal = false;
    bool makeClosedIsosurface = true;
    std::string nonDesignHandling;

    po::options_description getCommandLine();
};

struct RemeshOptions {
    double msize = -1.0;  // -1.0: compute size; 0: skip remeshing
    double msizescale = 2.0620569;
    double msizescale_min = 0.1;
    double msizescale_max = 1.33;
    ConstraintOptions constraints;

    po::options_description getCommandLine();
};

struct ThickerOptions {
    double thinBoneThreshold;
    double thinRegionThreshold;
    double hausdorffStartTol_;
    double hausdorffFinalTol_;
    bool doRemeshing;
    bool doSmoothing;
    bool doCleanUp;
    bool doModification;
    int nthickerIterations_;

    po::options_description getCommandLine();
};

}  // namespace d3d

struct CarpenterOptions {
    std::string dataFileName;
    std::string nonDesignFile;
    std::string outputFormat = "bdf";
    int verboseLevel;
};

#endif
