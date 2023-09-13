#include "Mesh.h"

#include <Corrade/Containers/Pair.h>

namespace mc_rtc::magnum
{

Mesh::Mesh(Object3D * parent,
           SceneGraph::DrawableGroup3D * group,
           ImportedMesh & data,
           Shaders::PhongGL & colorShader,
           Shaders::PhongGL & textureShader,
           Color4 color)
: CommonDrawable(parent, group)
{
  if(data.scene_)
  {
    Containers::Array<Object3D *> objects{std::size_t(data.scene_->mappingBound())};
    Containers::Array<Containers::Pair<UnsignedInt, Int>> parents = data.scene_->parentsAsArray();
    for(const Containers::Pair<UnsignedInt, Int> & parent : parents) { objects[parent.first()] = new Object3D{}; }
    for(const Containers::Pair<UnsignedInt, Int> & parent : parents)
    {
      objects[parent.first()]->setParent(parent.second() == -1 ? this : objects[parent.second()]);
    }
    for(const Containers::Pair<UnsignedInt, Matrix4> & transformation : data.scene_->transformations3DAsArray())
    {
      if(Object3D * object = objects[transformation.first()]) { object->setTransformation(transformation.second()); }
    }
    for(const Containers::Pair<UnsignedInt, Containers::Pair<UnsignedInt, Int>> & meshMaterial :
        data.scene_->meshesMaterialsAsArray())
    {
      Object3D * object = objects[meshMaterial.first()];
      Containers::Optional<GL::Mesh> & mesh = data.meshes_[meshMaterial.second().first()];
      if(!object || !mesh) continue;

      Int materialId = meshMaterial.second().second();

      /* Material not available / not loaded, use a default material */
      if(materialId == -1 || !data.materials_[materialId])
      {
        drawables_.push_back(new ColoredDrawable{object, group, colorShader, *mesh, color});
      }
      /* Textured material, if the texture loaded correctly */
      else if(data.materials_[materialId]->hasAttribute(Trade::MaterialAttribute::DiffuseTexture)
              && data.textures_[data.materials_[materialId]->diffuseTexture()])
      {
        drawables_.push_back(new TexturedDrawable{object, group, textureShader, *mesh,
                                                  *data.textures_[data.materials_[materialId]->diffuseTexture()]});
      }
      /* Color-only material */
      else
      {
        Containers::Optional<Color4> ambient = Containers::NullOpt;
        auto diffuse = color;
        if(data.materials_[materialId]->hasAttribute(Trade::MaterialAttribute::DiffuseColor))
        {
          diffuse = data.materials_[materialId]->diffuseColor();
          if(diffuse == 0xffffffff_rgbaf) { diffuse = color; }
        }
        if(data.materials_[materialId]->hasAttribute(Trade::MaterialAttribute::AmbientColor))
        {
          ambient = data.materials_[materialId]->ambientColor();
        }
        drawables_.push_back(new ColoredDrawable{object, group, colorShader, *mesh, diffuse, ambient});
      }
    }
  }
  else if(!data.meshes_.isEmpty() && data.meshes_[0])
  {
    drawables_.push_back(new ColoredDrawable(this, group, colorShader, *data.meshes_[0], color));
  }
}

void Mesh::draw_(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera)
{
  for(auto & d : drawables_) { d->draw_(transformationMatrix, camera); }
}

} // namespace mc_rtc::magnum
