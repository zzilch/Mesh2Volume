cmake_minimum_required(VERSION 3.15)
project(mesh2volume)

option(DEBUG_RELEASE_BUILD "Debug release build in VS" OFF)
if (MSVC AND DEBUG_RELEASE_BUILD)
    add_compile_options(/Z7)
    add_link_options(/INCREMENTAL:NO /DEBUG /OPT:REF /OPT:ICF)
endif()

if (DEFINED ENV{CONDA_PREFIX})
    # build with conda-build
    file(TO_CMAKE_PATH $ENV{CONDA_PREFIX} CONDA_PREFIX)
else()
    # build from source
    file(TO_CMAKE_PATH $ENV{PREFIX} CONDA_PREFIX)
endif()
message("Conda Prefix:${CONDA_PREFIX}")

if (WIN32)
    # cmake won't search this path in Windows
    file(TO_CMAKE_PATH ${CONDA_PREFIX}/Library/lib/cmake/OpenVDB OPENVDB_MODULE_PATH)
else()
    file(TO_CMAKE_PATH ${CONDA_PREFIX}/lib/cmake/OpenVDB OPENVDB_MODULE_PATH)
endif()
message("OpenVDB Module Path:${OPENVDB_MODULE_PATH}")
list(APPEND CMAKE_MODULE_PATH ${OPENVDB_MODULE_PATH})

find_package(OpenVDB REQUIRED)
find_package(Eigen3 REQUIRED eigen3)
include_directories( ${EIGEN3_INCLUDE_DIRS} )
find_package(pybind11 REQUIRED)

pybind11_add_module(mesh2volume MODULE module.cpp)
target_link_libraries(mesh2volume PRIVATE OpenVDB::openvdb)
target_link_libraries(mesh2volume PRIVATE TBB::tbb)
