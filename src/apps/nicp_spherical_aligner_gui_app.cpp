#include <iostream>
#include <fstream>
#include <limits>
#include <deque>
#include <queue>
#include <vector>

#include <Eigen/Core>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <qapplication.h>
#include <qevent.h>
#include <qglviewer.h>

#include "nicp/depth_utils.h"
#include "nicp/projective_aligner.h"
#include "nicp/spherical_projector.h"
#include <globals/system_utils.h>
#include "viewers/aligner_viewer.h"

using namespace std;
using namespace Eigen;
using namespace system_utils;
using namespace map_core;
using namespace nicp;

const char* banner[] = {
  "nicp_spherical_aligner_gui_app",
  "allows to align a set of clouds",
  "usage:",
  " nicp_spherical_aligner_gui_app <cloud1.dat> <cloud2.dat> ...",
  "",
  " once the GUI has started",
  " shift + left click selects a cloud",
  " M toggles the move cloud mode with cursor and page up/down keys (+ctrl to rotate)",
  " X aligns two clouds",
  " P prints the relative transforms of the clouds",
  0
};

int main(int argc, char** argv) {
  std::list<Cloud*> clouds;
  if(argc < 2 || !strcmp(argv[1],"-h")) {
    printBanner(banner);
    return 0;
  }

  SphericalProjector* projector = new SphericalProjector();

  ProjectiveAligner aligner(projector);
  aligner.projector().setMaxDistance(100.0);
  aligner.setDefaultConfig("1Level");
  aligner.solver().setDamping(0);
  aligner.solver().setMaxError(0.01);
  aligner.finder().setPointsDistance(1.0);
    
  int c = 1;
  while(c < argc) {
    Cloud* cloud = new Cloud;
    ifstream is(argv[c]);
    cloud->read(is);
    clouds.push_back(cloud);
    cerr << "loaded cloud [" << argv[c] << "] with " << cloud->size() << "] points" << endl;
    c++;
  }

  QApplication app(argc, argv);
  AlignerViewer viewer((BaseAligner*)&aligner);
  for(std::list<Cloud*>::iterator it = clouds.begin(); it != clouds.end(); ++it) {
    viewer.addCloud(*it);
  }
  viewer.show();
  app.exec();

  delete projector;
  for(std::list<Cloud*>::iterator it = clouds.begin(); it != clouds.end(); ++it) {
    delete *it;
  }
    
  return 0;
}
