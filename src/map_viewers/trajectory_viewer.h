#pragma once

#include <gl_helpers/simple_viewer.h>
#include "map_core/map_node_list.h"
#include "map_core/binary_node_relation.h"

namespace map_viewers {

  class TrajectoryViewer: public gl_helpers::SimpleViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    virtual void draw();
    virtual void drawWithNames();

    virtual void keyPressEvent(QKeyEvent *e);

    map_core::MapNodeList nodes;
    map_core::BinaryNodeRelationSet relations;

  protected:
    std::map<int, map_core::MapNode*> _names_map;
    virtual void postSelection(const QPoint& point);
    std::set<map_core::MapNode*> _selected_objects;

  };
 
}
