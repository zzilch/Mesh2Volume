cmake_minimum_required(VERSION 3.12.0)
project(mesh2volume VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(WITH_INDEX_VOLUME "Get index volume" OFF)

if(WITH_INDEX_VOLUME)
  add_definitions(-DUSE_INDEX_VOLUME) 
endif()

if(CMAKE_PREFIX_PATH)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_PREFIX_PATH}/lib/cmake/OpenVDB")
elseif(OPENVDB_ROOT)
  list(APPEND CMAKE_MODULE_PATH "${OPENVDB_ROOT}/lib/cmake/OpenVDB")
endif()

message("CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
message("OPENVDB_ROOT: ${OPENVDB_ROOT}")

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