#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>

#include "boost/filesystem.hpp"
#include "boost/endian.hpp"

#include "tbb/blocked_range.h"
#include "tbb/parallel_for.h"

#include "openvdb/openvdb.h"
#include "openvdb/io/Stream.h"
#include "openvdb/tools/MeshToVolume.h"
#include "openvdb/tools/Dense.h"

#include "CLI11.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

struct Mesh
{
    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> faces;
    openvdb::BBoxd bbox;
};

void read_obj(std::string filepath, Mesh &mesh)
{
    mesh.points.clear();
    mesh.faces.clear();

    tinyobj::ObjReaderConfig reader_config;
    reader_config.triangulate = true;
    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(filepath, reader_config))
    {
        if (!reader.Error().empty())
        {
            std::cerr << "TinyObjReader: " << reader.Error();
        }
        exit(1);
    }
    if (!reader.Warning().empty())
    {
        std::cout << "TinyObjReader: " << reader.Warning();
    }

    auto &attrib = reader.GetAttrib();
    auto &shapes = reader.GetShapes();
    auto &materials = reader.GetMaterials();

    for (size_t i = 0; i < attrib.vertices.size(); i += 3)
    {
        tinyobj::real_t vx = attrib.vertices[i + 0];
        tinyobj::real_t vy = attrib.vertices[i + 1];
        tinyobj::real_t vz = attrib.vertices[i + 2];
        openvdb::Vec3s v = openvdb::Vec3s(vx, vy, vz);
        mesh.bbox.expand(v);
        mesh.points.push_back(v);
    }

    for (size_t s = 0; s < shapes.size(); s++)
    {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++)
        {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            size_t i0 = shapes[s].mesh.indices[index_offset + 0].vertex_index;
            size_t i1 = shapes[s].mesh.indices[index_offset + 1].vertex_index;
            size_t i2 = shapes[s].mesh.indices[index_offset + 2].vertex_index;
            mesh.faces.push_back(openvdb::Vec3I(i0, i1, i2));
            index_offset += fv;
        }
    }
}

int main(int argc, char **argv)
{
    auto start = std::chrono::steady_clock::now();

    boost::filesystem::path input_path, output_path;
    int dim = 256;
    float bw = 3.0, expand = 1.57735;
    std::vector<float> bbox{-0.5, -0.5, -0.5, 0.5, 0.5, 0.5};
    bool flag_silent = 0, flag_fill = 0, flag_full = 0, flag_world = 0,
         flag_dense = 0, flag_unsigned = 0, flag_index = 0, flag_cube = 1, flag_cell = 0;

    CLI::App app("mesh2volume");
    app.add_option("input", input_path, "Input file path")
        ->check(CLI::ExistingFile)
        ->required();
    app.add_option("output", output_path, "Output file path");

    app.add_option("--dim", dim, "Grid size of the longest axis")
        ->check(CLI::PositiveNumber)
        ->default_val(256);
    app.add_option("--bw", bw, "Bandwith of the narrow-band")
        ->check(CLI::PositiveNumber)
        ->default_val(3.0);
    app.add_flag("--world", flag_world, "Use bandwith in the world space unit");
    app.add_flag("--fill", flag_fill, "[Signed] Fill interior");
    app.add_flag("--full", flag_full, "[Signed] Fill interior and exterior");

    app.add_flag("--dense", flag_dense, "Save as dense volume");
    app.add_option("--expand", expand, "[Dense] Expansion ratio for mesh bbox")
        ->check(CLI::PositiveNumber)
        ->default_val(1.57735);
    app.add_option("--bbox", bbox, "[Dense] User specified bbox")
        ->expected(6);
    app.add_option("--cube", flag_cube, "[Dense] Use a cubic bbox")
        ->default_val(true);

    app.add_flag("--silent", flag_silent, "Silent mode");
    app.add_flag("--unsigned", flag_unsigned, "Compute unsiged distance field");
    app.add_flag("--cell", flag_cell, "Cell centered");

#ifdef USE_INDEX_VOLUME
    app.add_flag("--index", flag_index, "[Todo] Save index volume");
#endif

    CLI11_PARSE(app, argc, argv);

    openvdb::initialize();

    std::chrono::steady_clock::time_point t0, t1;
    if (!flag_silent)
    {
        std::cout << "==========Read mesh==========\n";
        t0 = std::chrono::steady_clock::now();
    }

    Mesh mesh;
    read_obj(input_path.string(), mesh);

    if (!flag_silent)
    {
        t1 = std::chrono::steady_clock::now();
        std::cout << "Mesh:\n";
        std::cout << "\tPoints: " << mesh.points.size() << "\n";
        std::cout << "\tFaces: " << mesh.faces.size() << "\n";
        std::cout << "\tBBox: " << mesh.bbox << "\n";
        std::cout << "Time: " << std::chrono::duration<double>(t1 - t0).count()
                  << "s\n\n";
    }

    if (!flag_silent)
    {
        std::cout << "==========Mesh to volume==========\n";
        t0 = std::chrono::steady_clock::now();
    }

    // Set volume size and voxel size
    openvdb::BBoxd bbox_user(
        openvdb::Vec3d(bbox[0], bbox[1], bbox[2]),
        openvdb::Vec3d(bbox[3], bbox[4], bbox[5]));
    openvdb::Vec3d center;
    openvdb::Vec3d extent;
    if (!app.count("--bbox"))
    {
        center = mesh.bbox.getCenter();
        extent = mesh.bbox.extents() * expand;
    }
    else
    {
        center = bbox_user.getCenter();
        extent = bbox_user.extents();
    }

    float length = openvdb::math::Max(
        extent.x(),
        extent.y(),
        extent.z());

    float voxel_size = length / (dim - 1);
    if (voxel_size < 1e-5)
    {
        std::cout << "The voxel size (" << voxel_size << ") is too small.";
        exit(1);
    }

    openvdb::Vec3d volume_size;
    if (flag_cube)
    {
        volume_size.init(length, length, length);
    }
    else
    {
        volume_size = extent;
    }

    // Set narrow-band width
    float exbw = bw, inbw = bw;
    if (flag_full)
    {
        exbw = dim;
    }
    if (flag_world)
    {
        exbw = exbw / voxel_size;
        inbw = inbw / voxel_size;
    }
    if (flag_fill || flag_full)
    {
        inbw = std::numeric_limits<float>::max();
    }

    // // Transform to local grid space (cell-centered)
    t0 = std::chrono::steady_clock::now();
    openvdb::math::Transform::Ptr transform =
        openvdb::math::Transform::createLinearTransform(voxel_size);
    if(flag_cell) 
    {
        transform->postTranslate({voxel_size / 2, voxel_size / 2, voxel_size / 2});
    }

#ifdef USE_INDEX_VOLUME
    std::vector<openvdb::Vec3s> verts_local(mesh.points.size());
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, mesh.points.size()),
        openvdb::tools::mesh_to_volume_internal::TransformPoints<openvdb::Vec3s>(
            &mesh.points[0], &verts_local[0], *transform));
    openvdb::tools::QuadAndTriangleDataAdapter<openvdb::Vec3s, openvdb::Vec3I>
        adapter(verts_local, mesh.faces);
    // Mesh to volume conversion
    openvdb::Int32Grid::Ptr idx_grid;
    if (flag_index)
    {
        idx_grid.reset(new openvdb::Int32Grid(0));
    }

    openvdb::FloatGrid::Ptr grid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
        adapter, *transform, exbw, inbw, flag_unsigned, idx_grid.get());
#else
    openvdb::FloatGrid::Ptr grid;
    if (flag_unsigned)
    {
        grid = openvdb::tools::meshToUnsignedDistanceField<openvdb::FloatGrid>(*transform, mesh.points, mesh.faces, std::vector<openvdb::Vec4I>(), exbw);
    }
    else
    {
        grid = openvdb::tools::meshToSignedDistanceField<openvdb::FloatGrid>(*transform, mesh.points, mesh.faces, std::vector<openvdb::Vec4I>(), exbw, inbw);
    }
#endif

    if (!flag_silent)
    {
        t1 = std::chrono::steady_clock::now();
        grid->print(std::cout);
        std::cout << "Time: " << std::chrono::duration<double>(t1 - t0).count() << "s\n\n";
    }

    // Set output file
    std::string ext = !flag_dense ? ".vdb" : ".vtk";
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
    if (!flag_dense)
    {
        if (!flag_silent)
        {
            std::cout << "==========Write VDB==========\n";
            t0 = std::chrono::steady_clock::now();
        }
        std::string name = grid->getName();
        openvdb::io::Stream(ofile).write({grid});
        if (!flag_silent)
        {
            t1 = std::chrono::steady_clock::now();
            std::cout << "Time: " << std::chrono::duration<double>(t1 - t0).count() << "s\n\n";
        }
    }
    else
    {
        // Save volmume
        if (!flag_silent)
        {
            std::cout << "==========Sparse to dense==========\n";
            t0 = std::chrono::steady_clock::now();
        }

        // Sparse to dense, use LayoutXYZ required by the legacy .vtk file
        openvdb::Vec3d vmin = center - volume_size / 2;
        openvdb::Vec3d vmax = center + volume_size / 2;
        openvdb::tools::Dense<float, openvdb::tools::LayoutXYZ> dense(
            flag_cell ? 
            transform->worldToIndexCellCentered({vmin, vmax}) : 
            transform->worldToIndexNodeCentered({vmin, vmax}));
        openvdb::tools::copyToDense(*grid, dense);

        if (!flag_silent)
        {
            t1 = std::chrono::steady_clock::now();
            dense.print();
            std::cout << "Time: " << std::chrono::duration<double>(t1 - t0).count() << "s\n\n";
        }

        if (!flag_silent)
        {
            std::cout << "==========Write VTK==========\n";
            t0 = std::chrono::steady_clock::now();
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
        ofile << "SPACING " << voxel_size << " " << voxel_size << " " << voxel_size << "\n";
        ofile << "POINT_DATA " << dense.valueCount() << "\n";
        ofile << "SCALARS values float \n";
        ofile << "LOOKUP_TABLE default\n";
        ofile.write((char *)dense.data(), dense.valueCount() * sizeof(float));

        if (!flag_silent)
        {
            t1 = std::chrono::steady_clock::now();
            std::cout << "Time: " << std::chrono::duration<double>(t1 - t0).count() << "s\n\n";
        }
    }
    ofile.close();
    if (!flag_silent)
    {
        auto end = std::chrono::steady_clock::now();
        std::cout << "Total Time: " << std::chrono::duration<double>(end - start).count() << "s\n";
    }
    // @Todo: Save index volume
}
