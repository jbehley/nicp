#include <globals/system_utils.h>
#include "map_ros/map_msgs_ros.h"
#include "mapping/StampedCloudMsg.h"
#include "map_ros/cloud_publisher_trigger.h"
#include <ros/ros.h>
#include <txt_io/message_writer.h>
#include <tf/transform_listener.h>
#include <qapplication.h>
#include <gl_helpers/opengl_primitives.h>
#include "map_viewers_ros/global_map_viewer.h"

using namespace std;
using namespace boss;
using namespace map_viewers_ros;
using namespace gl_helpers;

const char* banner[] = {
  "global_mapper_viewer_node: shows the global maps under construction",
  "usage:",
  " global_mapper_dumper_node ",
  "once the gui starts",
  "1: toggles/untoggles the current view (and saves a lot of bandwidth)",
  "shift + left click on a node: highlights the local map of the node",
  0
};
  
int main(int argc, char** argv) {
  if (argc>1){
    system_utils::printBanner(banner);
    return 0;
  }

  ros::init(argc, argv, "global_mapper_viewer");
  ros::NodeHandle n;
  tf::TransformListener* listener = new tf::TransformListener(ros::Duration(60.0));

  QApplication* app=0; 
  GlobalMapViewer* viewer=0;
  
  app=new QApplication(argc, argv);
  viewer = new GlobalMapViewer();
  viewer->show();
  viewer->init(n, listener);
  

  while(ros::ok()){
    ros::spinOnce();
    app->processEvents();
    if (viewer->needRedraw()) {			
      viewer->updateGL();
    }
  }
}
