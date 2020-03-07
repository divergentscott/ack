import os
from conans import ConanFile, CMake


class AckConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    # comma-separated list of requirements
    requires = "boost/1.70.0@conan/stable", "VTK-mini/1.3@d3d/testing"
    #
#    "TBB/2019_U4@conan/stable", \
#            "boost_graph/1.69.0@d3d/testing", \
#            "tetgen/1.5.1@d3d/testing", \
#            "boost/1.69.0@d3d/testing"
    generators = "cmake"

    def config_options(self):
        if self.settings.os == "Windows":
            self.options["TBB"].shared = True

    def imports(self):
        self.copy("*.lib", dst="lib", keep_path=False)
        self.copy("*.dll", dst="bin", keep_path=False)
        self.copy("*.dylib*", dst="lib", keep_path=False)
        self.copy("*.so*", dst="lib", keep_path=False)
        self.copy("*.a", dst="lib", keep_path=False)