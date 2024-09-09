from conan import ConanFile
from conan.tools.cmake import cmake_layout


class City3DRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps", "VirtualRunEnv"

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        self.requires("scip/9.0.1")
        self.requires("boost/1.84.0", override=True)
        self.requires("cgal/[>=5.5.1 <=5.6.1]")
        self.requires("eigen/3.4.0")
        self.requires("gmp/6.3.0")
        self.requires("laslib/2.0.2")
        self.requires("laszip/3.4.4")
        self.requires("opencv/4.10.0")
        self.requires("rply/1.1.4")
        self.requires("soplex/7.0.1")
        self.requires("llvm-openmp/18.1.8", transitive_headers=True, transitive_libs=True)

    def build_requirements(self):
        self.tool_requires("boost/1.84.0", override=True)
    #     self.tool_requires("cmake/3.22.6")

    def configure(self):
        self.options["opencv/*"].parallel = "openmp"
        self.options["opencv/*"].fPIC = True
        # we only need the opencv main modules
        self.options["opencv/*"].neon = False
        self.options["opencv/*"].with_eigen = False
        self.options["opencv/*"].with_ffmpeg = False
        self.options["opencv/*"].with_flatbuffers = False
        self.options["opencv/*"].with_jpeg = False
        self.options["opencv/*"].with_jpeg2000 = False
        self.options["opencv/*"].with_msmf = False
        self.options["opencv/*"].with_msmf_dxva = False
        self.options["opencv/*"].with_openexr = False
        self.options["opencv/*"].with_png = False
        self.options["opencv/*"].with_protobuf = False
        self.options["opencv/*"].with_quirc = False
        self.options["opencv/*"].with_tesseract = False
        self.options["opencv/*"].with_tiff = False
        self.options["opencv/*"].with_wayland = False
        self.options["opencv/*"].with_webp = False