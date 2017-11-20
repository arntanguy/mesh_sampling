mesh_sampling
==

C++ Implementation of pointcloud generation from mesh sampling methods.
So far, the following samplers have been implemented:

- Weighted random sampling: generates a given number of points uniformely distributed according to triangle areas.
  See [this blog post](https://medium.com/@daviddelaiglesiacastro/3f-point-cloud-generation-from-3f-triangular-mesh-bbb602ecf238) for details on the method.

It is provided as-is, and could probably be optimized should the need arise. Feel free to submit merge requests. 


Example
==

```cpp
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <mesh_sampling/assimp_scene.h>
#include <mesh_sampling/weighted_random_sampling.h>

using namespace mesh_sampling;

int main(int /* argc */, char** /* argv */)
{
  // Model in a format supported by assimp
  std::string model_path = "../../getafe.stl";

  std::unique_ptr<ASSIMPScene> mesh = nullptr;
  try
  {
    mesh = std::unique_ptr<ASSIMPScene>(new ASSIMPScene(model_path));
  }
  catch (std::runtime_error& e)
  {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  // Point type
  WeightedRandomSampling<pcl::PointXYZRGB> sampling(mesh->scene());
  // Sample 10000 points on your scene
  auto cloud = sampling.weighted_random_sampling(10000);

  pcl::io::savePCDFileASCII("/tmp/test_pcd.pcd", *cloud);
  pcl::io::savePLYFileASCII("/tmp/test.ply", *cloud);
}
```
