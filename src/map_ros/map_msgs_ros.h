#include <ros_wrappers/ros_utils.h>
#include <boss/id_context.h>

#include "map_core/local_map.h"
#include "map_core/map_node.h"
#include "map_core/image_map_node.h"
#include "map_core/multi_image_map_node.h"
#include "map_core/binary_node_relation.h"
#include "map_core/base_camera_info.h"
#include "map_core/pinhole_camera_info.h"
#include "map_core/multi_camera_info.h"

#include "mapping/RichPointMsg.h"
#include "mapping/CloudMsg.h"
#include "mapping/PinholeCameraInfoMsg.h"
#include "mapping/MultiCameraInfoMsg.h"
#include "mapping/MapNodeMsg.h"
#include "mapping/ImageMapNodeMsg.h"
#include "mapping/MultiImageMapNodeMsg.h"
#include "mapping/LocalMapMsg.h"
#include "mapping/BinaryNodeRelationMsg.h"

namespace map_ros {

  void msg2cloud(map_core::Cloud& dest,    const mapping::CloudMsg& src);  
  void cloud2msg(mapping::CloudMsg& dest,  const map_core::Cloud& src);

  map_core::PinholeCameraInfo* msg2pinholeCameraInfo(const mapping::PinholeCameraInfoMsg& msg, boss::IdContext* context);
  mapping::PinholeCameraInfoMsg pinholeCameraInfo2msg(map_core::PinholeCameraInfo* src, boss::IdContext*  context);

  map_core::MultiCameraInfo* msg2multiCameraInfo(const mapping::MultiCameraInfoMsg& msg, boss::IdContext* context);
  mapping::MultiCameraInfoMsg multiCameraInfo2msg(map_core::MultiCameraInfo* src, boss::IdContext*  context);

  map_core::MapNode* msg2MapNode(const mapping::MapNodeMsg& msg, boss::IdContext* context);
  mapping::MapNodeMsg mapNode2msg(map_core::MapNode* src, boss::IdContext* context);

  map_core::ImageMapNode* msg2imageMapNode(const mapping::ImageMapNodeMsg& msg, boss::IdContext* context);
  mapping::ImageMapNodeMsg imageMapNode2msg(map_core::ImageMapNode* src, boss::IdContext* context);

  map_core::MultiImageMapNode* msg2multiImageMapNode(const mapping::MultiImageMapNodeMsg& msg, boss::IdContext* context);
  mapping::MultiImageMapNodeMsg multiImageMapNode2msg(map_core::MultiImageMapNode* src, boss::IdContext* context);

  map_core::LocalMap* msg2localMap(const mapping::LocalMapMsg& msg, boss::IdContext* context);
  mapping::LocalMapMsg localMap2msg(map_core::LocalMap* src, boss::IdContext* context);
  
  map_core::BinaryNodeRelation* msg2binaryNodeRelation(const mapping::BinaryNodeRelationMsg& msg, boss::IdContext* context);
  mapping::BinaryNodeRelationMsg binaryNodeRelation2msg(map_core::BinaryNodeRelation* src, boss::IdContext* context);
  
}
