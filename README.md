mesh_sampling
==

C++ Implementation of pointcloud generation from mesh sampling methods.

![Sampling example](https://raw.githubusercontent.com/arntanguy/mesh_sampling/master/sample/sampling_example.png)

So far, the following samplers have been implemented:

- Weighted random sampling: generates a given number of points uniformely distributed according to triangle areas.
  See [this blog post](https://medium.com/@daviddelaiglesiacastro/3f-point-cloud-generation-from-3f-triangular-mesh-bbb602ecf238) for details on the method.

It is provided as-is, and could probably be optimized should the need arise. Feel free to submit merge requests.


Example
==

```bash
./src/example <path_to_model>.<supported_format>
pcl_viewer /tmp/example_normal.pcd -normals_scale 5 -normals 1
pcl_viewer /tmp/example_xyz.pcd
pcl_viewer /tmp/example_rgb.pcd
pcl_viewer /tmp/example_rgb_normal.pcd
```
