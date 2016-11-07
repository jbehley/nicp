#include "core/projective_aligner.h"
#include "core/depth_utils.h"

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
  
  ProjectiveAligner aligner;
  aligner.projector().setMaxDistance(4);
  aligner.setReferenceModel(&reference);
  aligner.setCurrentModel(&current);
  aligner.setDefaultConfig("Xtion320x240");
  aligner.solver().setDamping(0);
  aligner.align();
  
  float factor = 255.0f/6;

  FloatImage img = aligner.finder().referenceZBuffer()-aligner.finder().zBuffer();
  cv::imwrite("difference.png", img*factor);
  cv::imwrite("current.png", aligner.finder().zBuffer()*factor);
  cv::imwrite("reference.png", aligner.finder().referenceZBuffer()*factor);
  
  cerr << "T: " << endl << aligner.T().matrix() << endl;
  
  current.transformInPlace(aligner.T());
  reference.add(current);
  ofstream os(argv[3]);
  reference.write(os);

  return 0;

}
