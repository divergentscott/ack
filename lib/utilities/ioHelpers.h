#ifndef UTILITIES_IOHELPERS_H
#define UTILITIES_IOHELPERS_H

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <string>
#include "d3derr.h"
#include "options.h"

namespace po = boost::program_options;

namespace d3d {
namespace io {

D3D_status getOutputPaths(IOOptions &options_in,
                          boost::filesystem::path &outputPath_out,
                          boost::filesystem::path &outputFolder_out);

D3D_status getInputPath(const d3d::IOOptions &ioOptions_in,
                        boost::filesystem::path &inputFilePath_out);

D3D_status parseCommandLineInput(int argc, char *argv[],
                                 po::options_description options_description_in,
                                 po::variables_map &vm_out);
}  // namespace io
}  // namespace d3d
#endif
