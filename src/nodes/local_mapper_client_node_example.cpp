#include "map_ros/map_msgs_ros.h"
#include "mapping/StampedCloudMsg.h"
#include "map_ros/cloud_publisher_trigger.h"
#include <ros/ros.h>
#include <txt_io/message_writer.h>
#include "map_ros/local_map_listener.h"

using namespace std;
using namespace boss;
using namespace map_ros;
using namespace map_core;
  
class LocalMapListenerExample : public LocalMapListener {
public:
  // override this and do what you want with the local map
  virtual void onNewLocalMap(LocalMap* lmap) {
    cerr << "got local map" << endl;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_mapper_client_node");
  LocalMapListenerExample* example = new LocalMapListenerExample;
  ros::NodeHandle n;
  example->init(n);
  ros::spin();
}
