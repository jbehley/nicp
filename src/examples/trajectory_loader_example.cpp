#include <fstream>
#include <stdexcept>

#include <boss/deserializer.h>
#include <boss/trusted_loaders.h>
#include <globals/system_utils.h>
#include "map_core/image_map_node.h"
#include "map_core/local_map.h"

using namespace std;
using namespace system_utils;
using namespace boss;
using namespace map_core;

// Help objects to force linking 
BaseCameraInfo cinfo;
ImageMapNode tnode;
LocalMap lmap;
BinaryNodeRelation rel;

const char* banner[] = {
  "trajectory_loader_app: example on how to load a set of boss objects constituting a boss map",
  "usage:",
  " trajectory_loader_app <boss log file>",
  0
};

int main(int argc, char** argv) {
  if(argc < 2 || !strcmp(argv[1],"-h")) {
    printBanner(banner);
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
