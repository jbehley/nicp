#pragma once

#include "local_map_viewer.h"
#include "mapping/MapUpdateMsg.h"

namespace map_viewers_ros {

  class GlobalMapViewer: public LocalMapViewer{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GlobalMapViewer(boss::IdContext* context = 0);
    void updateCallback(const mapping::MapUpdateMsgConstPtr& msg);
    void init(ros::NodeHandle& n, tf::TransformListener* tf_listener);
    virtual void draw();
    virtual void onNewNode(map_core::MapNode* node);

  protected:
    ros::Subscriber _updates_sub;
    Eigen::Isometry3f _delta_pose; // transform between the global map and the local mapper one
    // required to draw correclty the unassigned trajectory chunks
  };
}
