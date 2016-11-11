#pragma once

#include <txt_io/sensor_message_sorter.h>
#include <txt_io/message_seq_synchronizer.h>
#include <txt_io/pinhole_image_message.h>

#include "tracker.h"

namespace map_tracker {
  
  class CallTrackerTrigger: public txt_io::SensorMessageSorter::Trigger{
  public:
    CallTrackerTrigger(txt_io::SensorMessageSorter* sorter,
		       int priority,
		       Tracker* tracker_,
		       std::vector<txt_io::MessageSeqSynchronizer>* synchronizers_=0);
    virtual void action(std::tr1::shared_ptr<txt_io::BaseSensorMessage> msg);
    
    inline Tracker* tracker() {return _tracker;}
  protected:
    Tracker* _tracker;
    std::vector<txt_io::MessageSeqSynchronizer>* _synchronizers;
    txt_io::PinholeImageMessage* _depth_img, *_rgb_img;
  };
}
