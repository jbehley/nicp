#include <iostream>
#include <GL/gl.h>

#include "image_map_node.h"
#include <gl_helpers/opengl_primitives.h>

namespace map_core {
  
  using namespace std;
  using namespace boss;

  ImageMapNode::ImageMapNode(const Eigen::Isometry3f& t, 
			     BaseCameraInfo* cam,
			     const std::string& topic_,
			     int seq_,
			     int id,
			     IdContext* context) : MapNode(t, id, context){
    _topic = topic_;
    _camera_info = cam;
    _seq = seq_;
  }


  void ImageMapNode::serialize(ObjectData& data, IdContext& context){
    MapNode::serialize(data,context);
    data.setPointer("camera_info", _camera_info);
    data.setString("topic", _topic);
    data.setInt("seq",_seq);
  }

  void ImageMapNode::deserialize(ObjectData& data, IdContext& context){
    MapNode::deserialize(data,context);
    data.getReference("camera_info").bind(_camera_info);
    _topic = data.getString("topic");
    _seq = data.getInt("seq");
  }

  void ImageMapNode::draw(DrawAttributesType attributes, int name) {
    if (! (attributes&ATTRIBUTE_SHOW))
      return;
    if (name>-1)
      glPushName(name);

    glPushMatrix();
    gl_helpers::glMultMatrix(_transform);
    glScalef(0.1, 0.1, 0.1);
    gl_helpers::drawReferenceSystem();    
    glPopMatrix();
    
    glPushMatrix();
    Eigen::Isometry3f cameraPose = _transform*_camera_info->offset();
    gl_helpers::glMultMatrix(cameraPose);
    glColor3f(0.56f, 0.0f, 1.0f);
    gl_helpers::drawPyramidWireframe(0.02, 0.01);
    glPopMatrix();
    if (name>-1)
      glPopName();
  }


  BOSS_REGISTER_CLASS(ImageMapNode);

}
