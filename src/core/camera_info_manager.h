#pragma once
#include "base_camera_info.h"
#include <set>

namespace nicp {

  class CameraInfoManager: public nicp::Identifiable {
  public:
    CameraInfoManager(int id=-1,
		      nicp::IdContext* context=0);

    ~CameraInfoManager() ;

    BaseCameraInfo* getCamera(const std::string& topic) ;

    BaseCameraInfo* hasCamera(BaseCameraInfo* cam);
    
    void addCamera(BaseCameraInfo* cam);

    virtual void serialize(nicp::ObjectData& data, nicp::IdContext& context);
    virtual void deserialize(nicp::ObjectData& data, nicp::IdContext& context);
    
    std::vector<BaseCameraInfo*>& cameras() {return _camera_info_vector; }
  protected:
    std::vector<BaseCameraInfo*> _camera_info_vector;
    std::map<std::string, BaseCameraInfo*> _camera_info_map;
    std::set<BaseCameraInfo*> _camera_info_set;
  };

}
