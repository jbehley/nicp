#include "core/nn_aligner.h"
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
  Cloud reference_orig = reference;
  
  voxelize(reference, 0.05);
  Cloud current;
  current.read(is2);
  cerr << "current has " << current.size() << " points" << endl;
  voxelize(current, 0.05);
  Cloud current_orig = current;
    
  NNAligner aligner;
  aligner.setReferenceModel(&reference);
  aligner.setCurrentModel(&current);
  aligner.setIterations(10);
  aligner.align(Eigen::Isometry3f::Identity());

  cerr << "T: " << endl << aligner.T().matrix() << endl;

  current_orig.transformInPlace(aligner.T());
  reference_orig.add(current_orig);
  ofstream os(argv[3]);
  reference_orig.write(os);

  return 0;

}
