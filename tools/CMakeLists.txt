cmake_minimum_required(VERSION 3.15.0)
project(mesh2volume VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(WITH_INDEX_VOLUME "Get index volume" OFF)

if (DEFINED ENV{CONDA_PREFIX})
    file(TO_CMAKE_PATH $ENV{CONDA_PREFIX} CONDA_PREFIX)
else()
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

include_directories(include)

find_package(OpenVDB REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

add_executable(mesh2volume ./src/mesh2volume.cpp)
add_executable(vdb2vtk ./src/vdb2vtk.cpp)
add_executable(vdb2mesh ./src/vdb2mesh.cpp)

target_link_libraries(mesh2volume PRIVATE OpenVDB::openvdb)
target_link_libraries(mesh2volume PRIVATE Boost::boost Boost::filesystem)
target_link_libraries(mesh2volume PRIVATE TBB::tbb)

target_link_libraries(vdb2vtk PRIVATE OpenVDB::openvdb)
target_link_libraries(vdb2vtk PRIVATE Boost::boost Boost::filesystem)
target_link_libraries(vdb2vtk PRIVATE TBB::tbb)

target_link_libraries(vdb2mesh PRIVATE OpenVDB::openvdb)
target_link_libraries(vdb2mesh PRIVATE Boost::filesystem)

if (WIN32)
  file(TO_CMAKE_PATH ${CONDA_PREFIX}/Library/bin EXE_INSTALL_PATH)
else()
  file(TO_CMAKE_PATH ${CONDA_PREFIX}/bin EXE_INSTALL_PATH)
endif()
install(TARGETS mesh2volume vdb2vtk vdb2mesh RUNTIME DESTINATION ${EXE_INSTALL_PATH})