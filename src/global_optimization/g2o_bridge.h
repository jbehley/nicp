#pragma once

#include "map_core/binary_node_relation.h"
#include "map_core/map_node.h"
#include "map_core/map_node_list.h"

namespace g2o {
  class SparseOptimizer;
  class VertexSE3;
  class EdgeSE3;
}

namespace global_optimization {

  class G2OBridge {
  public:
    G2OBridge();
    ~G2OBridge();

    void optimize(int iterations_ = 10);
    void quietOptimize(int iterations_ = 10);

    void psToG2o(map_core::BinaryNodeRelationSet& relations,
		 map_core::MapNodeList& local_maps);
  
    void g2oToPs(map_core::MapNodeList& local_maps);

  protected:
    std::map<const g2o::VertexSE3*, map_core::MapNode*> _nodes_g2o_ps_map;
    std::map<const g2o::EdgeSE3*, map_core::BinaryNodeRelation*> _edges_g2o_ps_map;
    std::map<const map_core::MapNode*, g2o::VertexSE3* > _nodes_ps_g2o_map;
    std::map<const map_core::BinaryNodeRelation*, g2o::EdgeSE3*> _edges_ps_g2o_map;
    g2o::SparseOptimizer * graph;

    g2o::SparseOptimizer * g2oInit();

  };

}
