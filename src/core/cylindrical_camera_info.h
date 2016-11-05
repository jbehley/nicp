#pragma once
#include "base_camera_info.h"

namespace nicp {
  class CylindricalCameraInfo : public BaseCameraInfo {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    CylindricalCameraInfo(const std::string& topic_ = "none",
			  const std::string& frame_id = "",
			  float horizontal_res=M_PI/180,
			  float horizontal_fov=M_PI/2,
			  float vertical_res=M_PI/180,
			  float vertical_fov=M_PI/4,
			  const Eigen::Isometry3f&offset_ = Eigen::Isometry3f::Identity(),
			  float depth_scale_ = 1e-3,
			  int id=-1,
			  nicp::IdContext* context=0);

    
    virtual BaseCameraInfo* scale(float s);
    virtual void serialize(nicp::ObjectData& data, nicp::IdContext& context);
    virtual void deserialize(nicp::ObjectData& data, nicp::IdContext& context);
    
  protected:
    float _horizontal_fov;
    float _horizontal_res;
    float _vertical_fov;
    float _vertical_res;
  };

}
