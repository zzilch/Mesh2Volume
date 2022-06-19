#include <iostream>
#include <fstream>
#include <vector>

#include "boost/filesystem.hpp"

#include "openvdb/openvdb.h"
#include "openvdb/io/Stream.h"
#include "openvdb/tools/VolumeToMesh.h"

#include "CLI11.hpp"

struct Mesh
{
    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> triangles;
    std::vector<openvdb::Vec4I> quads;
};

void write_obj(const std::string &filepath, const Mesh &mesh)
{
    std::ofstream ofile(filepath);
    if (!ofile.is_open())
    {
        std::cout << "Fail to open file " << filepath << '\n';
        exit(1);
    }
    ofile << "# Created by vdb2mesh\n";
    for (auto &v : mesh.points)
        ofile << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
    for (auto &f : mesh.triangles)
        ofile << "f " << f[2] + 1 << " " << f[1] + 1 << " " << f[0] + 1 << "\n";
    for (auto &f : mesh.quads)
        ofile << "f " << f[3] + 1 << " " << f[2] + 1 << " " << f[1] + 1 << " " << f[0] + 1 << "\n";
    ofile.close();
}

int main(int argc, char **argv)
{
    boost::filesystem::path input_path, output_path;
    bool flag_silent = 0, flag_relax = true;
    double iso = 0, adapt = 0;

    CLI::App app("vdb2vtk");
    app.add_option("input", input_path, "Input file path")
        ->check(CLI::ExistingFile)
        ->required();
    app.add_option("output", output_path, "Output file path");
    app.add_option("--iso", iso, "Determines which isosurface to mesh");
    app.add_option("--adapt", adapt, "Adaptivity threshold [0 to 1]")
        ->default_val(0);
    app.add_option("--relax", flag_relax, "Toggle relaxing disoriented triangles during adaptive meshing")
        ->default_val(true);
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
        std::cout << "\n";
    }

    if (!app.count("--iso"))
    {
        openvdb::Vec3d vs = grid->voxelSize();
        iso = openvdb::math::Min(vs.x(), vs.y(), vs.z()) / 2;
    }

    Mesh mesh;
    openvdb::tools::volumeToMesh(*grid, mesh.points, mesh.triangles, mesh.quads, iso, adapt, flag_relax);
    if (!flag_silent)
    {   
        std::cout << "Isovalue: " << iso << "\n";
        std::cout << "Adaptivity: " << adapt << "\n";
        std::cout << "Mesh:\n";
        std::cout << "\tPoints: " << mesh.points.size() << "\n";
        std::cout << "\tFaces: Triangles(" << mesh.triangles.size()
                  << ") + Quads(" << mesh.quads.size()
                  << ") = " << mesh.triangles.size() + mesh.quads.size();
        std::cout << "\n\n";
    }
    // Set output file
    std::string ext = ".obj";
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
    write_obj(output_path.string(), mesh);
}