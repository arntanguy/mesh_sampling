#pragma once

#include <assimp/Importer.hpp> // C++ importer interface
#include <assimp/postprocess.h> // Post processing flags
#include <assimp/scene.h> // Output data structure
#include <stdexcept>

namespace mesh_sampling
{

class ASSIMPScene
{
  // The importer will automatically delete the scene
  Assimp::Importer importer;
  const aiScene * scene_;
  std::string modelPath_;

public:
  ASSIMPScene(const std::string & model_path) : modelPath_(model_path)
  {
    loadScene();
  }

  ASSIMPScene(const std::string & model_path, float scale) : modelPath_(model_path)
  {
    importer.SetPropertyFloat(AI_CONFIG_GLOBAL_SCALE_FACTOR_KEY, scale);
    loadScene();
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

protected:
  void loadScene()
  {
    scene_ =
        importer.ReadFile(modelPath_, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_GenNormals
                                          | aiProcess_FixInfacingNormals | aiProcess_GlobalScale);

    // If the import failed, report it
    if(!scene_)
    {
      throw std::runtime_error(importer.GetErrorString());
    }
  }
};

} // namespace mesh_sampling
