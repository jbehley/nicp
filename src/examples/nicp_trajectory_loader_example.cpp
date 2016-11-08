#include <fstream>
#include <stdexcept>

#include <qapplication.h>

#include "boss/deserializer.h"
#include "boss/trusted_loaders.h"
#include "globals/system_utils.h"
#include "map/image_map_node.h"
#include "map/local_map.h"

using namespace std;
using namespace nicp;

// Help objects to force linking 
BaseCameraInfo cinfo;
ImageMapNode tnode;
LocalMap lmap;
BinaryNodeRelation rel;

const char* banner[] = {
  "nicp_trajectory_loader_app: example on how to load a set of boss objects",
  "usage:",
  " nicp_trajectory_loader_app <boss log file>",
  0
};

int main(int argc, char** argv) {
  if(argc < 2 || !strcmp(argv[1],"-h")) {
    nicp::printBanner(banner);
    return 0 ;
  }
  
  std::list<Serializable*> objects;
  Deserializer des;
  des.setFilePath(argv[1]);
  Serializable* o;
  while((o = des.readObject())) {
    objects.push_back(o);
  }
  cerr << "Read: " << objects.size() << " elements" << endl;

  return 0;
}
