#include "spherical_camera_info.h"

namespace nicp {

  using namespace std;
 
  SphericalCameraInfo::
    SphericalCameraInfo(const std::string& topic_,
			const std::string& frame_id,
			const Eigen::Vector4f& camera_matrix_,
			const Eigen::Isometry3f&offset_,
			float depth_scale_,
			int id,
			nicp::IdContext* context):
      BaseCameraInfo(topic_, frame_id, offset_, depth_scale_, id,context){
    _camera_matrix=camera_matrix_;
   
  }

  BaseCameraInfo* SphericalCameraInfo::scale(float s) {
    Eigen::Vector4f cm=_camera_matrix;
    cm(2)*=s;
    cm(3)*=s;
    return new SphericalCameraInfo(_topic,
				   _frame_id,
				   cm,
				   _offset,
				   _depth_scale);
  }
    
  void SphericalCameraInfo::serialize(nicp::ObjectData& data, nicp::IdContext& context){
    BaseCameraInfo::serialize(data,context);
    _camera_matrix.toBOSS(data, "camera_matrix");
  }

  void SphericalCameraInfo::deserialize(nicp::ObjectData& data, nicp::IdContext& context){
    BaseCameraInfo::deserialize(data,context);
    _camera_matrix.fromBOSS(data, "camera_matrix");
  }

  BOSS_REGISTER_CLASS(SphericalCameraInfo);
}


