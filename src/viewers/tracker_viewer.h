#pragma once

#include <qapplication.h>
#include <qglviewer.h>

#include "tracker/tracker.h"
#include "globals/simple_viewer.h"

namespace nicp {

  class TrackerViewer : public SimpleViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    TrackerViewer(Tracker* _tracker);

    virtual void draw();
    inline bool followCamera() const {return _follow_camera;}
    inline void setFollowCamera(bool follow_camera) { _follow_camera = follow_camera;}

  protected:
    Tracker* _tracker;
    bool _modelTainted;
    bool _follow_camera;
    
  };

}
