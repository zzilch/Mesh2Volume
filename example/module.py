import numpy as np
import trimesh
import pymesh2volume
import pyvista as pv
import time

mesh = trimesh.load('./assets/plane.obj')
B = mesh.bounds.reshape(-1)
V = mesh.vertices
F = mesh.faces.astype('int32')
D = 128
P = mesh.bounding_box.sample_volume(10000)

scale = mesh.scale
sphere = pv.Sphere(scale/2,B.mean(0))

s = time.time()
volume = pymesh2volume.Volume(V,F,B,D)
e = time.time()
print("Mesh2Grid(dim=128):",e-s,"s")

s = time.time()
sdf,origin,spacing = volume.to_dense()
e = time.time()
print("Grid2Dense(dim=128):",e-s,"s")
dense = pv.wrap(sdf)
dense.spacing = spacing
dense.origin = origin
dense.save('./assets/plane.vtk')

s = time.time()
# mode:
# 0. nearest
# 1. trilinear
# 2. quadratic
sdf = volume.sample(P,mode=1)
e = time.time()
print("Sample(n=1e4):",e-s,"s")

pl = pv.Plotter(shape=(1,3))
pl.subplot(0,0)
pl.add_mesh(mesh,opacity=0.5)
pl.add_title("Mesh",font_size=10)
pl.subplot(0,1)
pl.add_mesh(dense.contour(),opacity=0.5,show_scalar_bar=False,cmap='jet')
pl.add_title("Dense(dim=128)",font_size=10)
pl.subplot(0,2)
pl.add_mesh(P,scalars=sdf,show_scalar_bar=False,cmap='jet')
pl.add_title("Sample(n=1e4)",font_size=10)
pl.link_views()
pl.view_isometric()
pl.show()