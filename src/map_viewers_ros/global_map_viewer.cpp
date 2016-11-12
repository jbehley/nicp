#include "global_map_viewer.h"
#include "mapping/MapUpdateMsg.h"
#include <stdexcept>

namespace map_viewers_ros {

  using namespace mapping;
  using namespace gl_helpers;
  using namespace map_core;
  
  GlobalMapViewer::GlobalMapViewer(boss::IdContext* context):
    LocalMapViewer(context){
    setOriginFrameId("odom");
    _delta_pose.setIdentity();
  }

  void GlobalMapViewer::updateCallback(const MapUpdateMsgConstPtr& msg) {
    for (size_t i = 0; i<msg->updates.size(); i++){
      const MapNodeUpdateMsg& update = msg->updates[i];
      boss::Identifiable* o = _context->getById(update.node_id);
      if (! o)
	continue;
      MapNode* n = dynamic_cast<MapNode*>(o);
      if (!n) {
	throw std::runtime_error("invalid type");
      }
      n->setTransform(pose2eigen(update.transform));
    }
  }


  void GlobalMapViewer::onNewNode(MapNode* node) {
    LocalMapViewer::onNewNode(node);
    // at each new node we determine the offset between the global map 
    // and the tracker map
    tf::StampedTransform transform;
    std::string from = "odom";
    std::string to = "closer_origin_frame_id";

    try{
      _tf_listener->waitForTransform(from, 
				     to, 
				     ros::Time(node->timestamp()), 
				     ros::Duration(0.1) );

      _tf_listener->lookupTransform (from, 
				     to, 
				     ros::Time(node->timestamp()), 
				     transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    _delta_pose = tfTransform2eigen(transform);
  }

 void GlobalMapViewer::draw() {
    // determine the offset between the last local map (according to the tracker)
    // and the actual position of the last local map

    glPushMatrix();
    glMultMatrix(_ref_pose);
    glColor3f(0.5, 0.5, 0.5);
    if (_show_clouds)
      _ref_cloud.draw();
    glPopMatrix();

    glPushMatrix();
    glMultMatrix(_curr_pose);
    glPushMatrix();
    glColor3f(0.2, 0.2, 1.0);
    if (_show_clouds)
      _curr_cloud.draw();
    glScalef(0.2, 0.2, 0.2);
    drawReferenceSystem();
    glPopMatrix();
    glPopMatrix();


    glPushMatrix();
    glMultMatrix(_delta_pose);
    for (std::list<MapNode*>::iterator it = _temp_nodes.begin(); it!=_temp_nodes.end(); it++){
      (*it)->draw();
    }
    glPopMatrix();

    TrajectoryViewer::draw();
    _need_redraw = false;
  }

  void GlobalMapViewer::init(ros::NodeHandle& n, tf::TransformListener* tf_listener) {
    LocalMapViewer::init(n, tf_listener);
    _updates_sub = n.subscribe<MapUpdateMsg>("/global_optimizer/map_updates", 10, 
					     boost::bind(&GlobalMapViewer::updateCallback, this, _1));
  }
 
}
