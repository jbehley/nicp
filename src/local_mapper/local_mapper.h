#pragma once

#include "map_core/local_map.h"
#include "map_core/binary_node_relation.h"
#include <boss/serializer.h>
#include "map_tracker/tracker.h"

namespace local_mapper {

  class TrajectoryMakerTrigger: public map_tracker::Tracker::Trigger{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    TrajectoryMakerTrigger(map_tracker::Tracker* tracker,
			   int events = 
			   map_tracker::Tracker::TRACK_GOOD|
			   map_tracker::Tracker::REFERENCE_FRAME_RESET|
			   map_tracker::Tracker::NEW_CAMERA_ADDED,
			   int priorory = 10);
    virtual void action(map_tracker::Tracker::TriggerEvent e);
    map_core::MapNodeList* nodes() {return _nodes;}
    map_core::BinaryNodeRelationSet* relations() { return _relations;}
    void setNodes(map_core::MapNodeList* nodes_) {_nodes = nodes_;}

    // min translation between two trajectory nodes
    inline float trajectoryMinTranslation()  const { return _trajectory_min_translation; }
    inline void setTrajectoryMinTranslation(float v)  { _trajectory_min_translation = v; }

    // min orientation between two trajectory nodes
    inline float trajectoryMinOrientation()  const { return _trajectory_min_orientation; }
    inline void setTrajectoryMinOrientation(float v)  { _trajectory_min_orientation = v; }
    virtual void onNewNodeCreated(map_core::MapNode* node, map_core::BinaryNodeRelation* rel=0);
    virtual void onCameraInfoCreated(map_core::BaseCameraInfo* camera_info);
  protected:
    Eigen::Isometry3f _last_global_pose;
    map_core::MapNodeList* _nodes;
    map_core::BinaryNodeRelationSet* _relations;
    float _trajectory_min_translation;
    float _trajectory_min_orientation;
  };


  class LocalMapper: public TrajectoryMakerTrigger {
  public:
    LocalMapper(map_tracker::Tracker* tracker,
		int events = 
		map_tracker::Tracker::TRACK_GOOD|
		map_tracker::Tracker::TRACK_BROKEN|
		map_tracker::Tracker::REFERENCE_FRAME_RESET|
		map_tracker::Tracker::TRACKING_DONE|
		map_tracker::Tracker::NEW_CAMERA_ADDED,
		int priorory = 10,
		boss::Serializer* ser=0);
    virtual void action(map_tracker::Tracker::TriggerEvent e);
    virtual void onLocalMapCreated(map_core::LocalMap* lmap);
    virtual void onRelationCreated(map_core::BinaryNodeRelation* rel);

    
    inline boss::Serializer* serializer() const {return _serializer;}
    inline void setSerializer(boss::Serializer* ser) {_serializer = ser;}

    inline map_core::MapNodeList* localMaps() { return _local_maps; }
    inline void setLocalMaps(map_core::MapNodeList* local_maps) { _local_maps = local_maps; }

    inline map_core::BinaryNodeRelationSet* localMapsRelations() { return _local_maps_relations; }
    inline void setLocalMapsRelations(map_core::BinaryNodeRelationSet* local_maps_relations) { _local_maps_relations = local_maps_relations; }


    // max size of the bounding box of the trajectory translation, after which a new local map is created
    inline float trajectoryMaxTranslation()  const { return _trajectory_max_translation; }
    inline void setTrajectoryMaxTranslation(float v)  { _trajectory_max_translation = v; }

    // max size of the bounding box of the trajectory orientation, after which a new local map is created
    inline float trajectoryMaxOrientation()  const { return _trajectory_max_orientation; }
    inline void setTrajectoryMaxOrientation(float v)  { _trajectory_max_orientation = v; }


    inline float clippingDistance() const {return _clipping_distance;}
    void setClippingDistance(float clipping_distance_) {_clipping_distance=clipping_distance_;}
    bool isTrajectoryBoundReached();

  protected:
    map_core::LocalMap* makeLocalMap();
    void saveLocalMap(map_core::LocalMap& lmap);
    void saveCameras(nicp::CameraInfoManager& manager);

    std::tr1::shared_ptr<map_core::LocalMap> _last_local_map, _previous_local_map;
    std::tr1::shared_ptr<map_core::BinaryNodeRelation> _last_relation;
    bool _enable;
    boss::Serializer* _serializer;
    float _trajectory_max_translation;
    float _trajectory_max_orientation;
    float _clipping_distance;
    map_core::MapNodeList* _local_maps;
    map_core::BinaryNodeRelationSet* _local_maps_relations;
  };

}

