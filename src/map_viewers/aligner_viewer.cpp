#include <iostream>

#include "aligner_viewer.h"

namespace map_viewers {

  using namespace std;
  using namespace map_core;
  using namespace nicp;
  
  AlignerViewer::AlignerViewer(BaseAligner* aligner, float voxelResolution) : CloudViewer() {
    _voxelResolution = voxelResolution;
    _aligner = aligner;
  }

  void AlignerViewer::keyPressEvent(QKeyEvent* e) {
    if(e->key() == Qt::Key_X) {
      align();
      return;
    }
    if(e->key() == Qt::Key_P) {
      cerr << "transforms: " << endl;
      for(CloudIsometryMap::iterator it = _clouds.begin(); it != _clouds.end(); ++it) {
	cerr << it->first << " -> " << t2v(it->second).transpose() << endl;
      }
      return;
    }
    CloudViewer::keyPressEvent(e);
  }
  
  void AlignerViewer::align() {
    if(_selected_objects.size() != 2) {
      cerr << "the objects should be exactly two" << std::endl;
      return;
    }
    
    std::set<const Cloud*>::iterator it = _selected_objects.begin();
    Cloud reference = *(*it);
    Eigen::Isometry3f& reference_transform = _clouds[*it];
    it++;
    Cloud current = *(*it);
    Cloud curr_cloud = *(*it);
    Eigen::Isometry3f& current_transform = _clouds[*it];

    if(_voxelResolution > 0.0) {
      voxelize(reference, _voxelResolution);
      voxelize(current, _voxelResolution);
    }
    
    // determine the transform between reference and current
    Eigen::Isometry3f dt = reference_transform.inverse() * current_transform;
    
    _aligner->setReferenceModel(&reference);
    _aligner->setCurrentModel(&current);
    _aligner->align(dt);

    current_transform = reference_transform * _aligner->T();
    updateGL();

    // curr_cloud.transformInPlace(current_transform);
    // ofstream os("refined.dat");
    // curr_cloud.add(reference);
    // curr_cloud.write(os);
    // printErrorStats();
  }

  void AlignerViewer::printErrorStats() {
    const std::vector<float>& errors = _aligner->solver().errors();
    int inliers = 0;
    int outliers = 0;
    double inliers_error_sum = 0;
    double outliers_error_sum = 0;
    for(size_t i = 0; i < errors.size(); ++i) {
      if(errors [i] < 0) {
	outliers++;
	outliers_error_sum -= errors[i];
	continue;
      }
      else {
	inliers++;
	inliers_error_sum += errors[i];
      }
    }
    cerr << "solver.max_error: " << _aligner->solver().maxError() << endl;
    cerr << "inliers : " << inliers << endl;
    cerr << "outliers: " << outliers << endl;
    cerr << "error/inliers: " << inliers_error_sum / inliers << endl;
    cerr << "error/outliers: " << outliers_error_sum / outliers << endl;
  }
  
}
