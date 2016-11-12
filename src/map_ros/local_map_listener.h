#pragma once

#include <ros_wrappers/ros_utils.h>
#include "map_msgs_ros.h"

namespace map_ros {

  class LocalMapListener {
  public:
    LocalMapListener(boss::IdContext* context_=0);

    void init(ros::NodeHandle& n);
    virtual void onNewLocalMap(map_core::LocalMap* lmap);
    virtual void onNewNode(map_core::MapNode* node);
    virtual void onNewRelation(map_core::BinaryNodeRelation* rel);
    virtual void onNewCameraInfo(map_core::BaseCameraInfo* cam);

  protected:
    void pinholeCameraInfoCallback(const mapping::PinholeCameraInfoMsgConstPtr& msg);
    void multiCameraInfoCallback(const mapping::MultiCameraInfoMsgConstPtr& msg);
    void imageMapNodeCallback(const mapping::ImageMapNodeMsgConstPtr& msg);
    void multiImageMapNodeCallback(const mapping::MultiImageMapNodeMsgConstPtr& msg);
    void relationsCallback(const mapping::BinaryNodeRelationMsgConstPtr& msg);
    void localMapCallback(const mapping::LocalMapMsgConstPtr& msg);
    
    void processPendingCameraInfos();
    void processPendingImageNodes();
    void processPendingRelations();
    void processPendingLocalMaps();
    void processPendingMsgs();
    
    std::list<mapping::ImageMapNodeMsg> _pending_image_node_msgs;
    std::list<mapping::MultiImageMapNodeMsg> _pending_multi_image_node_msgs;
    std::list<mapping::PinholeCameraInfoMsg> _pending_camera_info_msgs;
    std::list<mapping::MultiCameraInfoMsg> _pending_multi_camera_info_msgs;
    std::list<mapping::LocalMapMsg> _pending_local_map_msgs;
    std::list<mapping::BinaryNodeRelationMsg> _pending_relations_msgs;

    ros::Subscriber _sub_camera_info, _sub_multi_camera_info;
    ros::Subscriber _sub_image_map_node, _sub_multi_image_map_node;
    ros::Subscriber _sub_local_map;
    ros::Subscriber _sub_relations;
    std::set<boss::Identifiable*> _objects;
    boss::IdContext* _context;
  };

}
