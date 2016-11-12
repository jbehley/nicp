#pragma once

#include "map_ros/map_msgs_ros.h"
#include "mapping/StampedCloudMsg.h"
#include <ros/ros.h>
#include <txt_io/message_writer.h>
#include <tf/transform_listener.h>
#include <gl_helpers/simple_viewer.h>
#include <qapplication.h>
#include "map_viewers/trajectory_viewer.h"
#include <gl_helpers/simple_viewer.h>
#include <gl_helpers/opengl_primitives.h>
#include "map_ros/local_map_listener.h"
#include <tf/transform_listener.h>
#include <qevent.h>

namespace map_viewers_ros {  

  class LocalMapViewer: public map_ros::LocalMapListener, public map_viewers::TrajectoryViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
    LocalMapViewer(boss::IdContext* context_=0);

    virtual void draw();
    virtual void onNewLocalMap(map_core::LocalMap* lmap);
    virtual void onNewNode(map_core::MapNode * n);
    virtual void onNewRelation(map_core::BinaryNodeRelation * r);
    void cloudCallback(const mapping::StampedCloudMsgConstPtr& msg, Eigen::Isometry3f* pose, map_core::Cloud* dest);

    inline void setOriginFrameId(const std::string frame_id) {_origin_frame_id = frame_id;}
    inline const std::string& originFrameId() const {return _origin_frame_id;}
    
    void keyPressEvent(QKeyEvent *e);

    void setShowCurrentClouds(bool f);
    inline bool showCurrentClouds() const { return _show_clouds; }
    inline bool needRedraw() const {return _need_redraw;}
    void init(ros::NodeHandle& n, tf::TransformListener* tf_listener);

  protected:
    map_core::Cloud _reference;
    map_core::Cloud _current;
    bool _need_redraw;
    ros::NodeHandle * _n;
    std::string _origin_frame_id;
    ros::Subscriber _curr_sub, _ref_sub;
    Eigen::Isometry3f _curr_pose, _ref_pose;
    map_core::Cloud _curr_cloud, _ref_cloud;
    tf::TransformListener* _tf_listener;
    std::list<map_core::MapNode*> _temp_nodes; // nodes not yet in any of the local maps
    bool _show_clouds;
  };

}
