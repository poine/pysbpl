import setuptools
from Cython.Distutils import build_ext

##
# workaround this bug:
# http://stackoverflow.com/questions/8106258/cc1plus-warning-command-line-option-wstrict-prototypes-is-valid-for-ada-c-o
import distutils, Cython
class my_build_ext(Cython.Distutils.build_ext):
    def build_extensions(self):
        distutils.sysconfig.customize_compiler(self.compiler)
        try:
            self.compiler.compiler_so.remove("-Wstrict-prototypes")
        except (AttributeError, ValueError):
            pass
        build_ext.build_extensions(self)
##
# extra_compile_args=["-std=c++11", "-Wno-cpp"] is for removing the deprecation warning
#  because defining NPY_NO_DEPRECATED_API to NPY_1_7_API_VERSION breaks compilation

ext = setuptools.Extension(
    "pysbpl.sbpl",          # name of extension
    ["pysbpl/sbpl.pyx"],      # filename of our Pyrex/Cython source
    language="c++",    # this causes Pyrex/Cython to create C++ source
    include_dirs=[],
    extra_compile_args=["-std=c++11", "-Wno-cpp"],
    libraries=["sbpl"],
    library_dirs=[],
    runtime_library_dirs=[],
)

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name = 'pysbpl',
    ext_modules = [ext],
    cmdclass = {'build_ext': my_build_ext},
    version="0.0.1",
    author="Edited by: Matt Schmittle",
    author_email="schmttle@cs.washington.edu",
    description="Python bindings for SBPL",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/schmittlema/pysbpl",
    packages=setuptools.find_packages(),
    include_package_data=True,
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD3 License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
