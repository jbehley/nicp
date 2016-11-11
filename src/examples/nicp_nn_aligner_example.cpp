#include <fstream>

#include "nicp/nn_aligner.h"
#include <globals/system_utils.h>

using namespace std;
using namespace map_core;
using namespace nicp;

const char* banner[] = {
  "nicp_nn_aligner_example: example on how to register two point clouds using a nearest neighbor KD-tree based aligner",
  "usage:",
  " nicp_nn_aligner_example <model1.dat> <model2.dat> <output.dat>",
  0
};

int main(int argc, char** argv) {
  if(argc < 4) {
    system_utils::printBanner(banner);
    return 0;
  }

  ifstream is1(argv[1]);
  if(!is1) {
    cerr << "unable to load file " << argv[1] << endl;
    return 0;
  }

  ifstream is2(argv[2]);
  if(!is2) {
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
  aligner.setIterations(10);
  aligner.solver().setDamping(0);
  aligner.solver().setMaxError(0.01);
  aligner.setReferenceModel(&reference);
  aligner.setCurrentModel(&current);
  aligner.align(Eigen::Isometry3f::Identity());

  cerr << "T: " << endl << aligner.T().matrix() << endl;

  current_orig.transformInPlace(aligner.T());
  reference_orig.add(current_orig);
  ofstream os(argv[3]);
  reference_orig.write(os);

  return 0;

}
