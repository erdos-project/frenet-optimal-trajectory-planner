from setuptools import Distribution
import setuptools.command.build_ext as _build_ext
from distutils.core import setup, Extension
import sys
import os
import subprocess


# For running external build
class build_ext(_build_ext.build_ext):
    def run(self):
        command = ["./build.sh", "-p", sys.executable]
        subprocess.check_call(command)


class BinaryDistribution(Distribution):
    def has_ext_modules(self):
        return True


def main():
    os.environ['CC'] = 'g++'
    os.environ['CXX'] = 'g++'
    pwd = os.path.dirname(os.path.realpath(__file__))

    # Path to Eigen might differ, replace path if necessary
    CFLAGS = [
        '-std=c++11', '-fPIC', '-shared', '-I', '-O3', '-pthread', '-Wall',
        '-I/usr/include/eigen3'
    ]

    LDFLAGS = []
    LDFLAGS += ['-Xlinker', '-export-dynamic']
    LDFLAGS += ['-W','-Wno-undef', '-lstdc++', '-static-libstdc++']

    # Link static library libPackageFrenetOptimalTrajectory.a
    LDFLAGS += [
        '-L' + os.path.join(pwd, 'build'), '-lPackageFrenetOptimalTrajectory'
    ]
    LDFLAGS += [
        '-I', '/usr/include/x86_64-linux-gnu/qt5/QtCore', '-l', 'Qt5Core'
    ]
    print("Linker Flag:", LDFLAGS)

    module = Extension(
        'fot_planner',
        sources=['src/FrenetOptimalTrajectory/planner_package.cpp'],
        language='C++',
        include_dirs=[
            'src', 'src/CubicSpline', 'src/Polynomials',
            'src/FrenetOptimalTrajectory', 'src/Obstacle', 'src/Car'
        ],
        extra_compile_args=CFLAGS,
        extra_link_args=LDFLAGS)

    setup(name="fot_planner",
          version="1.0.0",
          description="FOT Planner",
          author="ERDOS Project",
          url="https://github.com/erdos-project/",
          ext_modules=[module])


if __name__ == "__main__":
    main()
