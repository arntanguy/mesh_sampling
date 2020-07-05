mesh_sampling
==

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![CI of mesh_sampling](https://github.com/arntanguy/mesh_sampling/workflows/CI%20of%20mesh_sampling/badge.svg)](https://github.com/arntanguy/mesh_sampling/actions?query=workflow%3A%22CI+of+mesh_sampling%22)
[![Package mesh_sampling](https://github.com/arntanguy/mesh_sampling/workflows/Package%20mesh_sampling/badge.svg)](https://github.com/arntanguy/mesh_sampling/actions?query=workflow%3A%22Package%20mesh_sampling%22)

C++ Implementation of pointcloud generation from mesh sampling methods.

![Sampling example](https://raw.githubusercontent.com/arntanguy/mesh_sampling/master/sample/sampling_example.png)

So far, the following samplers have been implemented:

- Weighted random sampling: generates a given number of points uniformely distributed according to triangle areas.
  See [this blog post](https://medium.com/@daviddelaiglesiacastro/3f-point-cloud-generation-from-3f-triangular-mesh-bbb602ecf238) for details on the method.

It is provided as-is, and could probably be optimized should the need arise. Feel free to submit merge requests.

Installation
==

Requirements:
- cmake >3.11
- Eigen3
- PCL 1.7

If you do not already have a recent cmake installation (>3.11), you will need to install it. On Ubuntu bionic, this can be done by adding the official [Kitware PPA](https://apt.kitware.com/), and updating cmake

For bionic:

```sh
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
sudo apt-get update
```

For xenial:

```sh
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ xenial-rc main'
sudo apt-get update
```

Then install cmake
```sh
sudo apt install cmake
```


You can now build and install this package

```
git clone --recursive https://github.com/arntanguy/mesh_sampling.git
cd mesh_sampling
mkdir build && cd build
cmake ..
make
sudo make install
```

Example
==

```bash
./src/example <path_to_model>.<supported_format>
pcl_viewer /tmp/example_normal.pcd -normals_scale 5 -normals 1
pcl_viewer /tmp/example_xyz.pcd
pcl_viewer /tmp/example_rgb.pcd
pcl_viewer /tmp/example_rgb_normal.pcd
```