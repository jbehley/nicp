#pragma once

#include <set>

#include "core/cloud.h"
#include "globals/simple_viewer.h"

namespace nicp {

  class CloudViewer : public SimpleViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::map<const Cloud*, Eigen::Isometry3f, std::less<const Cloud*>,
		     Eigen::aligned_allocator<std::pair<const Cloud*, Eigen::Isometry3f> > > CloudIsometryMap;
    CloudViewer();
    enum Mode {MoveCamera=0x0, MoveObject=0x1};

    virtual void draw();
    virtual void drawWithNames();
    virtual void keyPressEvent(QKeyEvent *e);

    void addCloud(Cloud* c, const Eigen::Isometry3f& iso = Eigen::Isometry3f::Identity());
    void eraseCloud(Cloud* c);

  protected:
    CloudIsometryMap _clouds;
    Mode _mode;
    std::map<int, const Cloud*> _names_map;
    virtual void postSelection(const QPoint& point);
    std::set<const Cloud*> _selected_objects;
    bool _is_orthographic;
    
  };

}
