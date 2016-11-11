#pragma once

#include "map/binary_node_relation.h"
#include "map/map_node.h"
#include "map/map_node_list.h"

namespace g2o {
  class SparseOptimizer;
  class VertexSE3;
  class EdgeSE3;
}

namespace nicp {

  class G2OBridge {
  public:
    G2OBridge();
    ~G2OBridge();

    void optimize(int iterations_ = 10);
    void quietOptimize(int iterations_ = 10);

    void psToG2o(BinaryNodeRelationSet& relations,
		 MapNodeList& local_maps);
  
    void g2oToPs(MapNodeList& local_maps);

  protected:
    std::map<const g2o::VertexSE3*, MapNode*> _nodes_g2o_ps_map;
    std::map<const g2o::EdgeSE3*, BinaryNodeRelation*> _edges_g2o_ps_map;
    std::map<const MapNode*, g2o::VertexSE3* > _nodes_ps_g2o_map;
    std::map<const BinaryNodeRelation*, g2o::EdgeSE3*> _edges_ps_g2o_map;
    g2o::SparseOptimizer * graph;

    g2o::SparseOptimizer * g2oInit();

  };

}
