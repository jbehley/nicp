#pragma once

#include "nicp/base_aligner.h"
#include "viewers/cloud_viewer.h"

namespace nicp {
  
  class AlignerViewer : public CloudViewer {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    AlignerViewer(BaseAligner* aligner, float voxelResolution = 0.0);

    virtual void keyPressEvent(QKeyEvent* e);

    void align();

    void printErrorStats();

    inline float voxelResolution() const { return _voxelResolution; }
    inline BaseAligner* aligner() { return _aligner; }

    inline void setVoxelResolution(float voxelResolution) { _voxelResolution = voxelResolution; }
    inline void setAligner(BaseAligner* aligner) { _aligner = aligner; }
    
  protected:
    float _voxelResolution;
    BaseAligner* _aligner;

  };

}
