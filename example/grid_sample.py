import numpy as np
import torch
import torch.nn.functional as F
import pyvista as pv

def vtk_read(filepath):
    vtk = pv.read(filepath)
    print("Dimensions:",vtk.dimensions)
    print("Spacing:",vtk.spacing)
    print("Origin:",vtk.origin)
    print("Center:",vtk.center)
    bbox = np.array(vtk.bounds).reshape(-1,2)
    print("BBox:\n",bbox)
    return vtk

def vtk_plot(vtk):
    spacing = vtk.spacing[0]
    pl = pv.Plotter()
    pl.add_mesh(vtk.threshold((-spacing*3,spacing*3)),opacity=0.5)
    pl.add_mesh(pv.Box(bounds=vtk.bounds),style='wireframe')
    pl.add_axes()
    pl.show()
    
def vtk_clip(vtk,bbox,dimensions):
    # basic info
    D = len(dimensions)
    origin = bbox[:,0]
    center = bbox.sum(-1)/2
    extent = (bbox[:,1]-bbox[:,0])/2
    spacing = extent*2/(dimensions-1)
    bbox_norm = (bbox-center[:,None])/extent[:,None]
    
    # create grid points
    coords = [
        torch.linspace(bbox_norm[i,0],bbox_norm[i,1],dimensions[i])
        for i in reversed(range(D)) # XZY->ZXY
    ]
    grid =  torch.meshgrid(coords,indexing='ij')
    grid = grid[::-1] # ZXY->XYZ
    grid = torch.stack(grid,dim=-1)
    
    # grid sample
    input = torch.tensor(vtk['values'].reshape(vtk.dimensions)).float()
    output = F.grid_sample(
        input.reshape((1,1,)+input.shape),
        grid.reshape((1,)+grid.shape)
    ).squeeze()
    
    # create vtk
    vtk_clipped = pv.UniformGrid(dimensions,spacing,origin)
    vtk_clipped.point_arrays['values'] = output.numpy().reshape(-1)
    return vtk_clipped
    
    
if __name__ == "__main__":
    vtk = vtk_read('./assets/bunny.obj.vtk')
    
    vtk_plot(vtk)
    
    bbox = np.array([
        [-1,1],
        [0,2],
        [-1,1]
    ])
    dimensions = np.array([64,64,64])
    vtk_clipped = vtk_clip(vtk,bbox,dimensions)
    vtk_plot(vtk_clipped)