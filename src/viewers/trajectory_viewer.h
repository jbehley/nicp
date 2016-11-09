#pragma once

#include "globals/simple_viewer.h"
#include "map/map_node_list.h"
#include "map/binary_node_relation.h"

namespace nicp {

  class TrajectoryViewer: public nicp::SimpleViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    virtual void draw();
    virtual void drawWithNames();

    virtual void keyPressEvent(QKeyEvent *e);

    MapNodeList nodes;
    BinaryNodeRelationSet relations;

  protected:
    std::map<int, MapNode*> _names_map;
    virtual void postSelection(const QPoint& point);
    std::set<MapNode*> _selected_objects;

  };
 
}
