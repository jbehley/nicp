#pragma once
#include "globals/sensor_message_sorter.h"
#include "globals/message_seq_synchronizer.h"
#include "globals/pinhole_image_message.h"

#include "tracker.h"
namespace nicp {
  
  class CallTrackerTrigger: public SensorMessageSorter::Trigger{
  public:
    CallTrackerTrigger(SensorMessageSorter* sorter,
		       int priority,
		       Tracker* tracker_,
		       std::vector<nicp::MessageSeqSynchronizer>* synchronizers_=0);
    virtual void action(std::tr1::shared_ptr<BaseSensorMessage> msg);
    
    inline Tracker* tracker() {return _tracker;}
  protected:
    Tracker* _tracker;
    std::vector<MessageSeqSynchronizer>* _synchronizers;
    PinholeImageMessage* _depth_img, *_rgb_img;
  };
}
