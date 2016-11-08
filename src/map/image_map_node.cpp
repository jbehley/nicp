#include <iostream>
#include <GL/gl.h>

#include "image_map_node.h"
#include "globals/opengl_primitives.h"

namespace nicp {
  
  using namespace std;

  ImageMapNode::ImageMapNode(const Eigen::Isometry3f& t, 
			     BaseCameraInfo* cam,
			     const std::string& topic_,
			     int seq_,
			     int id,
			     nicp::IdContext* context) : MapNode(t, id, context){
    _topic = topic_;
    _camera_info = cam;
    _seq = seq_;
  }


  void ImageMapNode::serialize(nicp::ObjectData& data, nicp::IdContext& context){
    MapNode::serialize(data,context);
    data.setPointer("camera_info", _camera_info);
    data.setString("topic", _topic);
    data.setInt("seq",_seq);
  }

  void ImageMapNode::deserialize(nicp::ObjectData& data, nicp::IdContext& context){
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
    Eigen::Isometry3f cameraPose = _transform*_camera_info->offset();
    nicp::glMultMatrix(cameraPose);
    nicp::drawPyramidWireframe(/*float pyrH = */0.02, /*float pyrW = */0.01);
    glPopMatrix();
    if (name>-1)
      glPopName();
  }


  BOSS_REGISTER_CLASS(ImageMapNode);

}
