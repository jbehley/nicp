#include <cstring>

#include <qevent.h>

#include <gl_helpers/opengl_primitives.h>

#include "local_mapper_viewer.h"

namespace map_viewers{

  using namespace std;
  using namespace Eigen;
  using namespace gl_helpers;
  using namespace map_core;
  using namespace local_mapper;
  
  LocalMapperViewer::LocalMapperViewer(LocalMapper* t) {
    _local_mapper = t;
    _tracker = t->tracker();
    _local_mapper->setLocalMaps(&nodes);
    _local_mapper->setLocalMapsRelations(&relations);
    _modelTainted = true;
    _local_map_trajectory = _local_mapper->nodes();
  }
  
  void LocalMapperViewer::draw(){
    if (!_tracker->currentModel() || !_tracker->referenceModel() )
      return;

    glPushMatrix();
    glMultMatrix( _tracker->globalT() );

    glPushMatrix();
    glScalef(0.2, 0.2, 0.2);
    drawReferenceSystem();
    glPopMatrix();

    // draw the current after applying the epsilon T from aligner
    glPushMatrix();
    glMultMatrix(_tracker->aligner().T().inverse() );
    if(_tracker->currentModel()) {
      glColor3f(0.3, 0.3, 0.8);
      _tracker->currentModel()->draw();
    }
    glPopMatrix();

    glPushMatrix();
    glMultMatrix(_tracker->lastCamera()->offset());
    drawPyramidWireframe(0.1, 0.05);
    glPopMatrix();

    // draw the reference
    if(_tracker->referenceModel()) {
      glColor3f(0.5, 0.5, 0.5);
      _tracker->referenceModel()->draw();
    }
    glPopMatrix();
            
    if(_local_map_trajectory) {
      for(MapNodeList::iterator it = _local_map_trajectory->begin(); it != _local_map_trajectory->end(); it++) {
    	(*it)->draw();
      }
    }

    for(BinaryNodeRelationSet::iterator it = relations.begin(); it != relations.end(); ++it) {
      // BinaryNodeRelation* rel = dynamic_cast<BinaryNodeRelation*>(*it);
      // if(rel) {
      // 	LocalMap* from = dynamic_cast<LocalMap*>(rel->from());
      // 	LocalMap* to = dynamic_cast<LocalMap*>(rel->to());      
      // 	if(from && to) { 
      // 	  std::cerr << "Ecchela" << std::endl;
      // 	}
      // }
    }
    
    TrajectoryViewer::draw();
  }

}

