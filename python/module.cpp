#include <iostream>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "tbb/blocked_range.h"
#include "tbb/parallel_for.h"

#include "openvdb/openvdb.h"
#include "openvdb/io/Stream.h"
#include "openvdb/tools/MeshToVolume.h"
#include "openvdb/tools/VolumeToMesh.h"
#include "openvdb/tools/Dense.h"
#include "openvdb/tools/Interpolation.h"

namespace py = pybind11;

struct Volume
{
    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> faces;
    openvdb::BBoxd bbox;
    int dim;

    openvdb::Vec3d extents;
    float length;
    float delta;
    openvdb::math::Transform::Ptr transform;
    openvdb::FloatGrid::Ptr grid;

    Volume(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::VectorXd& B, int D) :bbox(B.data()), dim(D)
    {
        int nV = V.rows();
        int nF = F.rows();
        points.resize(nV);
        faces.resize(nF);
        tbb::parallel_for(
            tbb::blocked_range<int>(0, nV),
            [&](const tbb::blocked_range<int>& r)
            {
                for (int i = r.begin(); i != r.end(); ++i)
                {
                    points[i] = openvdb::Vec3s(V(i, 0), V(i, 1), V(i, 2));
                }
            }
        );
        tbb::parallel_for(
            tbb::blocked_range<int>(0, nF),
            [&](const tbb::blocked_range<int>& r)
            {
                for (int i = r.begin(); i != r.end(); ++i)
                {
                    faces[i] = openvdb::Vec3I(F(i, 0), F(i, 1), F(i, 2));
                }
            }
        );
        extents = bbox.extents();
        length = extents.length();
        delta = length / (dim - 1);
        transform = openvdb::math::Transform::createLinearTransform(delta);
        grid = openvdb::tools::meshToSignedDistanceField<openvdb::FloatGrid>(
            *transform, 
            points, 
            faces, 
            std::vector<openvdb::Vec4I>(), 
            dim, 
            std::numeric_limits<float>::max());
    }

    auto sample(const Eigen::MatrixXd& P,int mode)
    {
        int nP = P.rows();
        Eigen::VectorXd sdf(nP);
        openvdb::FloatGrid::ConstAccessor accessor = grid->getConstAccessor();
        if (mode == 1)
        {
            // why: we need instance a template outside the lambda to compile sucessfully
            openvdb::FloatGrid::ConstAccessor accessor = grid->getConstAccessor();
            openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::BoxSampler> sampler(accessor, grid->transform());
            tbb::parallel_for(
                tbb::blocked_range<int>(0, nP),
                [&](const tbb::blocked_range<int>& r)
                {
                    openvdb::FloatGrid::ConstAccessor accessor_local = grid->getConstAccessor();
                    openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::BoxSampler> sampler_local(accessor_local, grid->transform());
                    for (int i = r.begin(); i != r.end(); ++i)
                    {
                        sdf[i] = sampler_local.wsSample(openvdb::Vec3d(P(i, 0), P(i, 1), P(i, 2)));
                    }
                }
            );
        }
        else if (mode == 2)
        {
            // why: we need instance a template outside the lambda to compile sucessfully
            openvdb::FloatGrid::ConstAccessor accessor = grid->getConstAccessor();
            openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::QuadraticSampler> sampler(accessor, grid->transform());
            tbb::parallel_for(
                tbb::blocked_range<int>(0, nP),
                [&](const tbb::blocked_range<int>& r)
                {
                    openvdb::FloatGrid::ConstAccessor accessor_local = grid->getConstAccessor();
                    openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::QuadraticSampler> sampler_local(accessor_local, grid->transform());
                    for (int i = r.begin(); i != r.end(); ++i)
                    {
                        sdf[i] = sampler_local.wsSample(openvdb::Vec3d(P(i, 0), P(i, 1), P(i, 2)));
                    }
                }
            );
        }
        else
        {
            {
                // why: we need instance a template outside the lambda to compile sucessfully
                openvdb::FloatGrid::ConstAccessor accessor = grid->getConstAccessor();
                openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::PointSampler> sampler(accessor, grid->transform());
                tbb::parallel_for(
                    tbb::blocked_range<int>(0, nP),
                    [&](const tbb::blocked_range<int>& r)
                    {
                        openvdb::FloatGrid::ConstAccessor accessor_local = grid->getConstAccessor();
                        openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::PointSampler> sampler_local(accessor_local, grid->transform());
                        for (int i = r.begin(); i != r.end(); ++i)
                        {
                            sdf[i] = sampler_local.wsSample(openvdb::Vec3d(P(i, 0), P(i, 1), P(i, 2)));
                        }
                    }
                );
            }
        }
        return sdf;
    }

    auto to_dense()
    {
        openvdb::Vec3d center = bbox.getCenter();
        openvdb::BBoxd bbox_dense = openvdb::BBoxd(center - length/2, center + length / 2);
        openvdb::tools::Dense<float> dense(transform->worldToIndexNodeCentered(bbox_dense));
        openvdb::tools::copyToDense(*grid, dense);
        openvdb::Coord dim_dense = dense.bbox().dim();

        py::array data = py::array(py::buffer_info(
            dense.data(),
            sizeof(float),
            py::format_descriptor<float>::format(),
            3,
            { dim_dense.x(), dim_dense.y(), dim_dense.z() },
            { sizeof(float) * dim_dense.y() * dim_dense.z(), sizeof(float) * dim_dense.z(),  sizeof(float) }));
        Eigen::Vector3d origin(bbox_dense.min().x(), bbox_dense.min().y(), bbox_dense.min().z());
        Eigen::Vector3d spacing(delta, delta, delta);
        return py::make_tuple(data, origin, spacing);
    }

    auto to_mesh(float iso,float adapt,bool flag_relax)
    {
        std::vector<openvdb::Vec3s> v;
        std::vector<openvdb::Vec3I> f;
        std::vector<openvdb::Vec4I> q;
        openvdb::tools::volumeToMesh(*grid, v,f,q, iso, adapt, flag_relax);
        Eigen::MatrixXd V(v.size(),3);
        Eigen::MatrixXi F(f.size(),3);
        Eigen::MatrixXi Q(q.size(),4);
        tbb::parallel_for(
            tbb::blocked_range<int>(0, v.size()),
            [&](const tbb::blocked_range<int>& r)
            {
                for (int i = r.begin(); i != r.end(); ++i)
                {
                    V(i, 0) = v[i][0];
                    V(i, 1) = v[i][1];
                    V(i, 2) = v[i][2];
                }
            }
        );
        tbb::parallel_for(
            tbb::blocked_range<int>(0, f.size()),
            [&](const tbb::blocked_range<int>& r)
            {
                for (int i = r.begin(); i != r.end(); ++i)
                {
                    F(i, 0) = f[i][0];
                    F(i, 1) = f[i][1];
                    F(i, 2) = f[i][2];
                }
            }
        );
        tbb::parallel_for(
            tbb::blocked_range<int>(0, q.size()),
            [&](const tbb::blocked_range<int>& r)
            {
                for (int i = r.begin(); i != r.end(); ++i)
                {
                    Q(i, 0) = q[i][0];
                    Q(i, 1) = q[i][1];
                    Q(i, 2) = q[i][2];
                    Q(i, 3) = q[i][3];
                }
            }
        );
        return py::make_tuple(V, F, Q);
    }

};

PYBIND11_MODULE(pymesh2volume, m) {
    py::class_<Volume>(m, "Volume")
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&, const Eigen::MatrixXd&, int>(),
            py::arg("V"), py::arg("F"), py::arg("B"), py::arg("D") = 128)
        .def("sample", &Volume::sample, py::arg("P"), py::arg("mode") = 1)
        .def("to_dense", &Volume::to_dense)
        .def("to_mesh",&Volume::to_mesh,py::arg("iso") = 1/128, py::arg("adapt") = 0, py::arg("flag_relax") = true);
}