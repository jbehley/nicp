#pragma once

#include <set>

#include "map_core/cloud.h"
#include <gl_helpers/simple_viewer.h>

namespace nicp {

  class CloudViewer : public gl_helpers::SimpleViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::map<const map_core::Cloud*, Eigen::Isometry3f, std::less<const map_core::Cloud*>,
		     Eigen::aligned_allocator<std::pair<const map_core::Cloud*, Eigen::Isometry3f> > > CloudIsometryMap;
    CloudViewer();
    enum Mode {MoveCamera=0x0, MoveObject=0x1};

    virtual void draw();
    virtual void drawWithNames();
    virtual void keyPressEvent(QKeyEvent *e);

    void addCloud(map_core::Cloud* c, const Eigen::Isometry3f& iso = Eigen::Isometry3f::Identity());
    void eraseCloud(map_core::Cloud* c);

  protected:
    CloudIsometryMap _clouds;
    Mode _mode;
    std::map<int, const map_core::Cloud*> _names_map;
    virtual void postSelection(const QPoint& point);
    std::set<const map_core::Cloud*> _selected_objects;
    bool _is_orthographic;
    
  };

}
