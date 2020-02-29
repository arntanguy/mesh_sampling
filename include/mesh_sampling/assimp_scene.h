#pragma once

#include <assimp/Importer.hpp> // C++ importer interface
#include <assimp/postprocess.h> // Post processing flags
#include <assimp/scene.h> // Output data structure
#include <stdexcept>

namespace mesh_sampling
{

class ASSIMPScene
{
  // Create an instance of the Importer class
  // The importer will automatically delete the scene
  Assimp::Importer importer;
  const aiScene * scene_;

public:
  ASSIMPScene(const std::string & model_path)
  : // And have it read the given file with some example postprocessing
    // Usually - if speed is not the most important aspect for you - you'll
    // propably to request more postprocessing than we do in this example.
    scene_(importer.ReadFile(model_path,
                             aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_GenNormals
                                 | aiProcess_FixInfacingNormals))
  {
    // If the import failed, report it
    if(!scene_)
    {
      throw std::runtime_error(importer.GetErrorString());
    }
  }

  /**
   * @brief Do not store the returned pointer, only use it
   *
   * @return
   */
  const aiScene * scene() const
  {
    return scene_;
  }
};

} // namespace mesh_sampling
