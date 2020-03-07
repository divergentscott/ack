#include "ioHelpers.h"
#include <boost/filesystem.hpp>
#include <iostream>
//#include "generated/git.h"

namespace d3d {

namespace io {
namespace {
auto validExtensions = {"stl", "bdf", "fem", "obj", "vtk"};
}

D3D_status getOutputPaths(IOOptions& options,
                          boost::filesystem::path& outputPath,
                          boost::filesystem::path& outputFolder) {
    auto inputPath = boost::filesystem::path(options.input);

    std::string extension;
    for (auto ext : validExtensions) {
        if (ext == options.outputPath) {
            extension = options.outputPath;
            options.outputPath = "";
        }
    }

    outputPath = options.outputPath;

    if (options.outputFolder.empty()) {
        if (options.outputPath.empty())
            outputFolder = inputPath.parent_path();
        else
            outputFolder = outputPath.parent_path();
    } else {
        outputFolder = options.outputFolder;
    }

    if (options.outputPath.empty()) {
        outputPath = (outputFolder / (inputPath.stem().string() + "_out" +
                                      inputPath.extension().string()));
    }

    if (!extension.empty()) {
        outputPath.replace_extension("." + extension);
    }

    if (boost::filesystem::create_directories(outputFolder))
        return D3D_status::SUCCESS;
    else
        return D3D_status::FAIL;
}

D3D_status parseCommandLineInput(int argc, char* argv[],
                                 po::options_description options_description_in,
                                 po::variables_map& vm_out) {
    try {
        auto parsed = po::command_line_parser(argc, argv)
                          .options(options_description_in)
                          .style(po::command_line_style::unix_style |
                                 po::command_line_style::long_case_insensitive)
                          .run();
        // unix_style includes the following: allow_short | short_allow_adjacent
        // | short_allow_next | allow_long | long_allow_adjacent |
        // long_allow_next | allow_sticky | allow_guessing |
        // allow_dash_for_short
        po::store(parsed, vm_out);
        po::notify(vm_out);
    } catch (boost::program_options::error& err) {
        // catch the specific exception and say which option is wrong
        std::cout << "Wrong options!\n Use --help or -h for usage.\n";
        std::cout << "Error message: " << err.what() << "\n";
        return D3D_status::UNKNOWN_OPTIONS;
    } catch (...) {
        // catch the specific exception and say which option is wrong
        std::cout << "Wrong options!\n Use --help or -h for usage."
                  << std::endl;
        return D3D_status::UNKNOWN_OPTIONS;
    }

    if (vm_out.count("help") || argc == 1) {
        std::cout << options_description_in << "\n";
        return D3D_status::SUCCESS_AND_DONE;
    }

    if (vm_out.count("version")) {
        //std::cout << VERSION_STRING << std::endl;
        return D3D_status::SUCCESS_AND_DONE;
    }
    if (!vm_out.count("input")) {
        std::cout << "Missing input file\n";
        return D3D_status::MISSING_INPUT;
    }

    return D3D_status::SUCCESS;
}

D3D_status getInputPath(const d3d::IOOptions& ioOptions_in,
                        boost::filesystem::path& inputFilePath_out) {
    inputFilePath_out = boost::filesystem::path(ioOptions_in.input);
    auto validExtensions = {".stl", ".bdf", ".fem", ".dat"};
    if (std::none_of(validExtensions.begin(), validExtensions.end(),
                     [&inputFilePath_out](std::string s) {
                         return inputFilePath_out.extension() == s;
                     })) {
        printf("\n please include file extension .stl, .fem, .dat or .bdf!");
        return D3D_status::MISSING_FILE_EXTENSION;
    }
    if (!boost::filesystem::exists(inputFilePath_out)) {
        std::cout << "Missing input file path\n";
        return D3D_status::CANNOT_OPEN_FILE;
    }

	return D3D_status::SUCCESS;
}
}  // namespace io
}  // namespace d3d
