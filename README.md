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
  // Model in a format supported by assimp
  std::string model_path = "your_model.stl";

  // Create an instance of the Importer class
  Assimp::Importer importer;
  // And have it read the given file with some example postprocessing
  // Usually - if speed is not the most important aspect for you - you'll
  // propably to request more postprocessing than we do in this example.
  const aiScene* scene = importer.ReadFile( model_path,
        aiProcess_Triangulate            |
        aiProcess_JoinIdenticalVertices  |
        aiProcess_SortByPType);

  // If the import failed, report it
  if( !scene)
  {
    std::cerr << importer.GetErrorString() << std::endl; ;
    return false;
  }

  WeightedRandomSampling sampling(scene);
  // Sample 10000 points on your scene
  auto cloud = sampling.weighted_random_sampling(10000);

  pcl::io::savePCDFileASCII ("/tmp/test_pcd.pcd", *cloud);
```
