#pragma once
#include "map_core/base_camera_info.h"
#include <set>

namespace nicp {

  class CameraInfoManager: public boss::Identifiable {
  public:
    CameraInfoManager(int id=-1,
		      boss::IdContext* context=0);

    ~CameraInfoManager() ;

    map_core::BaseCameraInfo* getCamera(const std::string& topic) ;

    map_core::BaseCameraInfo* hasCamera(map_core::BaseCameraInfo* cam);
    
    void addCamera(map_core::BaseCameraInfo* cam);

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    
    std::vector<map_core::BaseCameraInfo*>& cameras() {return _camera_info_vector; }
  protected:
    std::vector<map_core::BaseCameraInfo*> _camera_info_vector;
    std::map<std::string, map_core::BaseCameraInfo*> _camera_info_map;
    std::set<map_core::BaseCameraInfo*> _camera_info_set;
  };

}
