#pragma once

#include <qapplication.h>
#include <qglviewer.h>

#include "local_mapper/local_map_triggers.h"

#include "tracker/tracker.h"
#include "trajectory_viewer.h"

namespace nicp {
  
  class LocalMapperViewer : public TrajectoryViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    LocalMapperViewer(LocalMapTrigger* _trigger);

    virtual void draw();

  protected:
    Tracker* _tracker;
    LocalMapTrigger* _trigger;
    bool _modelTainted;
    MapNodeList* _local_map_trajectory;

  };
  
}
