#include <fstream>
#include <stdexcept>

#include <qapplication.h>

#include "boss/deserializer.h"
#include "boss/trusted_loaders.h"
#include "core/cloud.h"
#include "globals/system_utils.h"
#include "map/image_map_node.h"
#include "map/local_map.h"
#include "viewers/trajectory_viewer.h"

using namespace std;
using namespace nicp;

// Help objects to force linking 
BaseCameraInfo cinfo;
ImageMapNode tnode;
LocalMap lmap;
BinaryNodeRelation rel;

const char* banner[] = {
  "nicp_trajectory_loader_gui_app: example on how to load and show a set of boss objects constituting a boss map",
  "usage:",
  " nicp_trajectory_loader_gui_app <boss log file>",
  0
};

int main(int argc, char** argv) {
  if(argc < 2 || !strcmp(argv[1], "-h")) {
    nicp::printBanner(banner);
    return 0 ;
  }
  
  std::list<Serializable*> objects;
  Deserializer des;
  des.setFilePath(argv[1]);
  Serializable* o;
  MapNodeList lmaps;
  while((o = des.readObject()) ){
    LocalMap* lmap = dynamic_cast<LocalMap*>(o);
    if(lmap) {
      lmaps.addElement(lmap);
    }
    objects.push_back(o);
  }
  cerr << "Read: " << objects.size() << " elements" << endl;
  cerr << "Read: " << lmaps.size() << " local maps" << endl;

  QApplication app(argc, argv);
  TrajectoryViewer viewer;
  viewer.nodes = lmaps;
  viewer.show();
  app.exec();

  return 0;
}
