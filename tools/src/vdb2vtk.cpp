#include <iostream>
#include <fstream>
#include <vector>

#include "boost/filesystem.hpp"
#include "boost/endian/conversion.hpp"

#include "tbb/blocked_range.h"
#include "tbb/parallel_for.h"

#include "openvdb/openvdb.h"
#include "openvdb/io/Stream.h"
#include "openvdb/tools/MeshToVolume.h"
#include "openvdb/tools/Dense.h"

#include "CLI11.hpp"

int main(int argc, char **argv)
{
    boost::filesystem::path input_path, output_path;
    std::vector<float> bbox{-0.5, -0.5, -0.5, 0.5, 0.5, 0.5};
    float dim = 256;
    bool flag_cube = 0, flag_silent = 0;

    CLI::App app("vdb2vtk");
    app.add_option("input", input_path, "Input file path")
        ->check(CLI::ExistingFile)
        ->required();
    app.add_option("output", output_path, "Output file path");
    auto opt_dim = app.add_option("--dim", dim, "Crop dim")
                       ->check(CLI::PositiveNumber)
                       ->default_val(256);
    auto opt_bbox = app.add_option("--bbox", bbox, "Crop bbox")
                        ->expected(6);
    opt_bbox->excludes(opt_dim);
    app.add_flag("--silent", flag_silent, "Silent mode");
    CLI11_PARSE(app, argc, argv);

    openvdb::initialize();

    openvdb::io::File ifile(input_path.string());
    ifile.open();
    openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>((*ifile.getGrids())[0]);
    ifile.close();
    if (!flag_silent)
    {
        grid->print(std::cout);
    }

    // Set volume size and voxel size
    openvdb::math::Transform::Ptr transform = grid->transformPtr();
    openvdb::Vec3d voxel_size = grid->voxelSize();

    openvdb::BBoxd bbox_user(
        openvdb::Vec3d(bbox[0], bbox[1], bbox[2]),
        openvdb::Vec3d(bbox[3], bbox[4], bbox[5]));
    openvdb::Vec3d center;
    openvdb::Vec3d extent;

    if (!app.count("--bbox"))
    {
        openvdb::CoordBBox bbox_grid = grid->evalActiveVoxelBoundingBox();
        openvdb::Coord bmin = bbox_grid.min();
        openvdb::Coord bmax = bbox_grid.max();
        center = (transform->indexToWorld(bmin) + transform->indexToWorld(bmax)) / 2;
        extent = (dim - 1) * voxel_size;
    }
    else
    {
        center = bbox_user.getCenter();
        extent = bbox_user.extents();
    }

    // Set output file
    std::string ext = ".vtk";
    if (!app.count("output"))
    {
        output_path = input_path;
        output_path.concat(ext);
    }
    else
    {
        if (output_path.extension() != ext)
            output_path.concat(ext);
    }
    std::ofstream ofile(output_path.string(), std::ios_base::binary);
    if (!ofile.is_open())
    {
        std::cout << "Fail to open file " << output_path.string() << '\n';
        exit(1);
    }

    // Sparse to dense, use LayoutXYZ required by the legacy .vtk file
    openvdb::Vec3d vmin = center - extent / 2;
    openvdb::Vec3d vmax = center + extent / 2;
    openvdb::tools::Dense<float, openvdb::tools::LayoutXYZ> dense(
        transform->worldToIndexCellCentered({vmin, vmax}));
    openvdb::tools::copyToDense(*grid, dense);
    if (!flag_silent)
    {
        dense.print();
    }

    // Convert to big endian required by the legacy .vtk file
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, dense.valueCount()),
        [&](const tbb::blocked_range<size_t> &r) {
            for (size_t i = r.begin(); i != r.end(); ++i)
                boost::endian::native_to_big_inplace(dense.data()[i]);
        });

    openvdb::Coord dim_dense = dense.bbox().dim();
    ofile << "# vtk DataFile Version 5.0\n";
    ofile << output_path.filename().string() << "\n";
    ofile << "BINARY\n";
    ofile << "DATASET STRUCTURED_POINTS\n";
    ofile << "DIMENSIONS " << dim_dense.x() << " " << dim_dense.y() << " " << dim_dense.z() << "\n";
    ofile << "ORIGIN " << vmin.x() << " " << vmin.y() << " " << vmin.z() << "\n";
    ofile << "SPACING " << voxel_size.x() << " " << voxel_size.y() << " " << voxel_size.z() << "\n";
    ofile << "POINT_DATA " << dense.valueCount() << "\n";
    ofile << "SCALARS values float \n";
    ofile << "LOOKUP_TABLE default\n";
    ofile.write((char *)dense.data(), dense.valueCount() * sizeof(float));
    ofile.close();
}