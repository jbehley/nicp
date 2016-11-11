#pragma once

#include <qapplication.h>
#include <qglviewer.h>

#include "map_tracker/tracker.h"
#include <gl_helpers/simple_viewer.h>

namespace nicp {

  class TrackerViewer : public gl_helpers::SimpleViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    TrackerViewer(map_tracker::Tracker* _tracker);

    virtual void draw();

    inline bool followCamera() const {return _follow_camera;}
    inline void setFollowCamera(bool follow_camera) { _follow_camera = follow_camera;}

  protected:
    map_tracker::Tracker* _tracker;
    bool _modelTainted;
    bool _follow_camera;
    
  };

}
