#pragma once

#include <qapplication.h>
#include <qglviewer.h>

#include "local_mapper/local_mapper.h"

#include "map_tracker/tracker.h"
#include "trajectory_viewer.h"

namespace map_viewers {
  
  class LocalMapperViewer : public TrajectoryViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    LocalMapperViewer(local_mapper::LocalMapper* _local_mapper);

    virtual void draw();

  protected:
    map_tracker::Tracker* _tracker;
    local_mapper::LocalMapper* _local_mapper;
    bool _modelTainted;
    map_core::MapNodeList* _local_map_trajectory;

  };
  
}
