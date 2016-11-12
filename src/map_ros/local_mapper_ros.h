#pragma once

#include "nicp/depth_utils.h"
#include "nicp/nn_aligner.h"
#include "nicp/projective_aligner.h"
#include "map_tracker/tracker.h"
#include "map_tracker/base_triggers.h"
#include "local_mapper/local_mapper.h"
#include "map_tracker/multi_tracker.h"
#include <txt_io/tf_overrider_trigger.h>
#include <ros_wrappers/image_message_listener.h>
#include <txt_io/message_enlister_trigger.h>
#include <txt_io/message_dumper_trigger.h>
#include <txt_io/pinhole_image_message.h>
#include "map_msgs_ros.h"
#include "mapping/IdsSrv.h"
#include "mapping/LocalMapByIdSrv.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <vector>
#include <fstream>
#include <iostream>

namespace map_ros {

  class LocalMapperRos : public local_mapper::LocalMapper {
  public:
  LocalMapperRos(map_tracker::Tracker* tracker,
		 int priorory = 10,
		 boss::Serializer* ser=0,
		 boss::IdContext * context=0) :    
    local_mapper::LocalMapper(tracker,
			      map_tracker::Tracker::TRACK_GOOD|
			      map_tracker::Tracker::TRACK_BROKEN|
			      map_tracker::Tracker::REFERENCE_FRAME_RESET|
			      map_tracker::Tracker::TRACKING_DONE|
			      map_tracker::Tracker::NEW_CAMERA_ADDED,
			      priorory,
			      ser) {
      _context = context;
      std::cerr << "local mapper ros" << std::endl;
    }
  
    void init(ros::NodeHandle& nh);
    virtual void onCameraInfoCreated(map_core::BaseCameraInfo* camera_info);
    virtual void onLocalMapCreated(map_core::LocalMap* lmap);
    virtual void onNewNodeCreated(map_core::MapNode* node, map_core::BinaryNodeRelation* rel);
    virtual void onRelationCreated(map_core::BinaryNodeRelation* rel);

    virtual bool srvLocalMapIds(mapping::IdsSrv::Request& req, mapping::IdsSrv::Response& resp);
    virtual bool srvGetLocalMap(mapping::LocalMapByIdSrv::Request& req, mapping::LocalMapByIdSrv::Response& resp);

  protected:
    void serializeCameras();
    void serializeCamera(map_core::BaseCameraInfo* cam);
    void serializeTrajectory(map_core::MapNodeList& nodes);
    void serializeNode(map_core::MapNode* n);
    void serializeRelation(map_core::BinaryNodeRelation* rel);
    std::vector<map_core::BaseCameraInfo*> _camera_infos;
    ros::Publisher _image_node_pub;
    ros::Publisher _multi_image_node_pub;
    ros::Publisher _camera_info_pub;
    ros::Publisher _multi_camera_info_pub;
    ros::Publisher _local_map_pub;
    ros::Publisher _relations_pub;
    ros::ServiceServer _local_map_ids_srv;
    ros::ServiceServer _get_local_map_srv;
    boss::IdContext* _context;

    std::map<int, mapping::LocalMapMsg*> _savedLocalMapMsgs;
  };
  
}
