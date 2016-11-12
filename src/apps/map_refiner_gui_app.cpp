#include <fstream>
#include <stdexcept>

#include <qevent.h>
#include <qapplication.h>

#include <boss/deserializer.h>
#include <boss/serializer.h>
#include <boss/trusted_loaders.h>

#include "map_core/cloud.h"
#include "nicp/nn_aligner.h"
#include "map_global_optimization/g2o_bridge.h"
#include <globals/system_utils.h>
#include "map_core/image_map_node.h"
#include "map_core/local_map.h"
#include "map_viewers/trajectory_viewer.h"

using namespace std;
using namespace boss;
using namespace system_utils;
using namespace map_core;
using namespace map_global_optimization;
using namespace nicp;
using namespace map_viewers;

// Help objects to force linking 
BaseCameraInfo cinfo;
ImageMapNode tnode;
LocalMap lmap;
NNAligner aligner;
BinaryNodeRelation rel;

BinaryNodeRelation* matchLocalMaps(LocalMap& reference, LocalMap& current){
  cerr << "Matching" << endl;
  // step 1, determine the transformation between reference and current
  Eigen::Isometry3f T = reference.transform().inverse() * current.transform();
  T.translation().setZero();
  // step 2 prepare the aligner

  Cloud tempRef(*reference.cloud());
  Cloud tempCurr(*current.cloud());
  voxelize(tempRef, 0.05);
  voxelize(tempCurr, 0.05);
  aligner.finder().setPointsDistance(1.);
  aligner.finder().setNormalAngle(M_PI/4);
  aligner.setIterations(10);
  aligner.setReferenceModel(&tempRef);
  aligner.setCurrentModel(&tempCurr);
  aligner.align(T);
  current.setTransform(reference.transform()*aligner.T());
  BinaryNodeRelation* rel= new BinaryNodeRelation(&reference, &current, aligner.T(), Matrix6f::Identity());
  return rel;
}

G2OBridge bridge;

class RefinerViewer: public TrajectoryViewer{
public:
  RefinerViewer(std::string output_filename_ = "", std::list<Serializable*>* serializable_objects_ = 0) { 
    _output_filename = output_filename_; 
    _serializable_objects = serializable_objects_;
  }
  void keyPressEvent(QKeyEvent *e)
  {
    // Defines the Alt+R shortcut.
    if ((e->key() == Qt::Key_M))
      {
	if (_selected_objects.size()!=2) {
	  cerr << "to do the matching you need to have exactly twom objects, I do nothing" << endl;
	  return;
	}
	LocalMap* reference, *current;
	std::set<MapNode*>::iterator it = _selected_objects.begin();
	reference = dynamic_cast<LocalMap*>(*it);
	it++;
	current = dynamic_cast<LocalMap*>(*it);
	if (! reference || ! current)
	if (_selected_objects.size()!=2) {
	  cerr << "invalid object types, I do nothing" << endl;
	  return;
	}
	if (_last_relation)
	  _last_relation.reset();
	_last_relation = std::tr1::shared_ptr<BinaryNodeRelation>(matchLocalMaps(*reference, *current));
	update(); // Refresh display
      }
    else if ((e->key() == Qt::Key_O))
      {
	bridge.psToG2o(relations, nodes);
	bridge.optimize();
	bridge.g2oToPs(nodes);
	update(); // Refresh display
      }
    else if ((e->key() == Qt::Key_A))
      {
	if (_last_relation) {
	  if(_serializable_objects) { _serializable_objects->push_back(_last_relation.get()); }	
	  relations.insert(_last_relation);
	  _last_relation = std::tr1::shared_ptr<BinaryNodeRelation>();
	}
	update(); // Refresh display
      }
    else if ((e->key() == Qt::Key_C))
      {
	_selected_objects.clear();
	update(); // Refresh display
      }
    else if ((e->key() == Qt::Key_V))
      {
	for (MapNodeList::iterator it = nodes.begin(); it!=nodes.end(); it++) {
	  _selected_objects.insert(it->get());
	}
	update(); // Refresh display
      }
    // Remove edge
    else if ((e->key() == Qt::Key_R))
      {
	if (_selected_objects.size() != 2) {
	  cerr << "to an edge you need to have exactly two objects, I do nothing" << endl;
	  return;
	}
	LocalMap* reference, *current;
	std::set<MapNode*>::iterator it = _selected_objects.begin();
	reference = dynamic_cast<LocalMap*>(*it);
	it++;
	current = dynamic_cast<LocalMap*>(*it);
	if (! reference || ! current) {
	  cerr << "invalid object types, I do nothing" << endl;
	  return;
	}

	for(BinaryNodeRelationSet::iterator it = relations.begin(); it != relations.end(); ++it) {
	  const std::tr1::shared_ptr<BinaryNodeRelation>& r = *it;
	  if(r->from() == reference && r->to() == current ||
	     r->from() == current && r->to() == reference) { 	    
	    relations.erase(it); 
	    if(_serializable_objects) { _serializable_objects->remove(it->get()); }				    
	  }
	}

	update(); // Refresh display
      }
    // Save current graph
    else if ((e->key() == Qt::Key_S))
      {
	if(_output_filename == "" || !_serializable_objects) { 
	  std::cerr << "[WARNING]: output filename not given in the command line when this was started, I do nothing" << std::endl; 
	  return;
	}
	
	Serializer ser;
	ser.setFilePath(_output_filename);
	ser.setBinaryPath(_output_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>");
	for(std::list<Serializable*>::iterator it = _serializable_objects->begin(); 
	    it != _serializable_objects->end(); 
	    ++it) {
	  Serializable* s = *it;
	  ser.writeObject(*s);
	}
      }
    else
      QGLViewer::keyPressEvent(e);    
  }    

  std::string _output_filename;
  std::tr1::shared_ptr<BinaryNodeRelation> _last_relation;
  std::list<Serializable*>* _serializable_objects;
};

const char* banner[] = {
  "map_refiner_gui_app: allows manual loop closure",
  "usage:",
  " map_refiner_app [options] <boss log file>",  
  " where: ",
  " -h          [], prints this help",
  " -o    [string], output filename where to write the refined graph",
  "once the gus starts",
  " 1: toggles/untoggles the current view (and saves a lot of bandwidth)",
  " shift + left click on a node: highlights the local map of the node",
  " M: matches the local maps (if there are only two higlighted)",
  " A: accepts the most recent match (press any other key to discard)",
  " O: optimizes the network",
  " R: remove the edges between the local maps (if there are only two higlighted)",
  " S: save the current graph",
  0
};

int main (int argc, char** argv) {
  int c = 1;
  std::string output_filename = "";
  std::string input_filename = "";		
  while (c<argc) {
    if (! strcmp(argv[c], "-h")) {
      printBanner(banner);
      return 0;
    } 
    else if (! strcmp(argv[c], "-o")) {
      c++;
      output_filename = std::string(argv[c]);
    }
    else {
      input_filename = std::string(argv[c]);
    }
    c++;
  }
  std::list<Serializable*> objects;
  Deserializer des;
  des.setFilePath(input_filename);
  Serializable* o;
  QApplication app(argc, argv);
  RefinerViewer viewer(output_filename, &objects);
  while ( (o = des.readObject()) ){
    LocalMap* lmap = dynamic_cast<LocalMap*>(o);
    if (lmap)
      viewer.nodes.addElement(lmap);
    BinaryNodeRelation* rel = dynamic_cast<BinaryNodeRelation*> (o);
    if (rel)
      viewer.relations.insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel));

    objects.push_back(o);
  }
  cerr << "Read: " << objects.size() << " elements" << endl;
  cerr << "Read: " << viewer.nodes.size() << " local maps" << endl;
  
  viewer.show();
  app.exec();
  
  return 0;
}
