#pragma once

#include <qapplication.h>
#include <qglviewer.h>

#include "local_mapper/local_map_triggers.h"

#include "map_tracker/tracker.h"
#include "trajectory_viewer.h"

namespace nicp {
  
  class LocalMapperViewer : public TrajectoryViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    LocalMapperViewer(local_mapper::LocalMapTrigger* _trigger);

    virtual void draw();

  protected:
    map_tracker::Tracker* _tracker;
    local_mapper::LocalMapTrigger* _trigger;
    bool _modelTainted;
    map_core::MapNodeList* _local_map_trajectory;

  };
  
}
