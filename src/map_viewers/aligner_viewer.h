#pragma once

#include "nicp/base_aligner.h"
#include "map_viewers/cloud_viewer.h"

namespace map_viewers {
  
  class AlignerViewer : public CloudViewer {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    AlignerViewer(nicp::BaseAligner* aligner, float voxelResolution = 0.0);

    virtual void keyPressEvent(QKeyEvent* e);

    void align();

    void printErrorStats();

    inline float voxelResolution() const { return _voxelResolution; }
    inline nicp::BaseAligner* aligner() { return _aligner; }

    inline void setVoxelResolution(float voxelResolution) { _voxelResolution = voxelResolution; }
    inline void setAligner(nicp::BaseAligner* aligner) { _aligner = aligner; }
    
  protected:
    float _voxelResolution;
    nicp::BaseAligner* _aligner;

  };

}
