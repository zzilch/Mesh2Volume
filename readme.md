# Mesh2Volume

Tools for converting mesh to signed distance field volume (Mesh2SDF) using [OpenVDB](https://github.com/AcademySoftwareFoundation/openvdb).


## Tools

- **mesh2volume**: Convert Wavefront *.obj* mesh to sparse and narrow-band *.vdb* volume or dense *.vtk* volume.
- **vdb2vtk**: Convert sparse and narrow-band *.vdb* volume to dense *.vtk* volume.
- **vdb2mesh**: Convert sparse and narrow-band *.vdb* volume to Wavefront *.obj* mesh.


## Usage

Run `target --help` to check more options.

1. Mesh2VDB: The result can be visualized by Houdini, Maya and `vdb_view` from OpenVDB. 


    ```bash
    mesh2volume ./assets/bunny.obj # ./assets/bunny.obj.vdb --dim 256 --bw 3
    ```
    
    ![bunny.obj.vdb](./assets/bunny.obj.vdb.png)

2. Mesh2VTK: The result can be visualized by VTK, Paraview.

    ```bash
    mesh2volume ./assets/bunny.obj --full --dense # ./assets/bunny.obj.vtk` --dim 256
    ```

    ![bunny.obj.vtk](./assets/bunny.obj.vtk.png)

3. VDB2VTK: Specify dimension or bounding box to crop the *.vdb* volume.


    ```bash
    vdb2vtk ./assets/bunny.obj.vdb # ./assets/bunny.obj.vdb.vtk --dim 256
    ```

    ![bunny.obj.vdb.vtk](./assets/bunny.obj.vdb.vtk.png)

4. VDB2Mesh: The result is a mesh mixed with quad and triangle faces.

    ```bash
    vdb2mesh ./assets/bunny.obj.vtk # ./assets/bunny.obj.vtk.obj --iso 0 --adapt 0
    ```

    ![bunny.obj.vdb.obj](./assets/bunny.obj.vdb.obj.png)

5. VTK2Mesh: read as numpy array in python with vtk wrapper or just and run marching cube


## Build

### Windows

1. Install [vcpkg](https://github.com/microsoft/vcpkg)

2. Install dependencies

    ```bash
    # Linux & MacOS
    vcpkg install openvdb
    vcpkg install boost-filesystem
    ```

3. Build

    ```bash
    mkdir build
    cd build
    # windows
    cmake -DCMAKE_TOOLCHAIN_FILE="<PATH_TO_VCPKG>/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows -A x64 ..
    cmake --build . --parallel 4 --config Release --target ALL_BUILD
    ```

    You can also build and install the latest version of [OpenVDB](https://github.com/AcademySoftwareFoundation/openvdb) from source and run

    ```bash
    cmake -DCMAKE_TOOLCHAIN_FILE="<PATH_TO_VCPKG>/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows -A x64 -DOPENVDB_ROOT="<PATH_TO_OPENVDB_INSTALL_PATH>" ..
    ```

4. The targets are in `./build/Release`.

### Linux

1. Install [Miniconda](https://docs.conda.io/en/latest/miniconda.html) or [Anaconda](https://www.anaconda.com/)

2. Create a virtual environment and install dependencies

    ```bash
    # dependencies of openvdb-core
    # openexr>=2.2, tbb>=2018.0 (2020 is ok), blosc>=1.5.0 (1.15.0 is ok),  zlib>=1.2.7, boost>=1.61 conda 
    sudo apt install zstd
    create -n openvdb -c conda-forge cmake openexr tbb-devel=2018.0 blosc=1.15.0 zlib boost-cpp=1.75
    conda activate openvdb

    # [optional] dependencies of openvdb-tools, better to use library in the system
    # conda install -c conda-forge jemalloc glfw 

    # build and install openvdb in the conda environment
    git clone https://github.com/AcademySoftwareFoundation/openvdb
    cd openvdb
    cmake .. -DCMAKE_PREFIX_PATH=$CONDA_PREFIX -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX #-DOPENVDB_BUILD_VDB_VIEW=ON
    cmake --build . --parallel 4 --config Release --target install

3. Build (in the root of this repo)
    ```
    mkdir build && cd build
    cmake .. -DCMAKE_PREFIX_PATH=$CONDA_PREFIX
    cmake --build . --parallel 4 --config Release --target all
    ``` 

4. The targets are in `./build`