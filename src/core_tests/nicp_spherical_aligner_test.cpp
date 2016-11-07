#include "core/projective_aligner.h"
#include "core/depth_utils.h"
#include "core/spherical_projector.h"

#include <fstream>

using namespace std;
using namespace nicp;

int main(int argc, char** argv) {
  if (argc<4) {
    cerr << "usage: " << argv[0] << "<model1.dat> <model2.dat> <output.dat>" << endl;
    return 0;
  }

  ifstream is1(argv[1]);
  if(! is1) {
    cerr << "unable to load file " << argv[1] << endl;
    return 0;
  }

  ifstream is2(argv[2]);
  if(! is2) {
    cerr << "unable to load file " << argv[2] << endl;
    return 0;
  }

  cerr << "loading models" << endl;
  Cloud reference;
  reference.read(is1);
  cerr << "reference has " << reference.size() << " points" << endl;
  
  Cloud current;
  current.read(is2);
  cerr << "current has " << current.size() << " points" << endl;

  SphericalProjector* projector=new SphericalProjector();

  ProjectiveAligner aligner(projector);
  aligner.finder().setPointsDistance(1.0);
  aligner.solver().setMaxError(0.01);
  aligner.setReferenceModel(&reference);
  aligner.setCurrentModel(&current);
  aligner.setDefaultConfig("1Level");
  aligner.solver().setDamping(0);
  aligner.align();
  
  float factor = 255.0f/6;

  FloatImage img = aligner.finder().referenceZBuffer()-aligner.finder().zBuffer();
  cv::imwrite("differences.png", img*factor);
  cv::imwrite("current.png", aligner.finder().zBuffer()*factor);
  cv::imwrite("reference.png", aligner.finder().referenceZBuffer()*factor);

  cerr << "T: " << endl << aligner.T().matrix() << endl;
  
  current.transformInPlace(aligner.T());
  reference.add(current);
  ofstream os(argv[3]);
  reference.write(os);

  return 0;

}
