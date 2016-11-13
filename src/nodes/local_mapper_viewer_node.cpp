#include <iostream>
#include <cstring>
#include <globals/system_utils.h>
#include "map_ros/map_msgs_ros.h"
#include "mapping/StampedCloudMsg.h"
#include "map_ros/cloud_publisher_trigger.h"
#include <ros/ros.h>
#include <txt_io/message_writer.h>
#include <tf/transform_listener.h>
#include <qapplication.h>
#include <gl_helpers/opengl_primitives.h>
#include "map_ros/local_map_listener.h"
#include "map_viewers_ros/local_map_viewer.h"

using namespace std;
using namespace boss;
using namespace gl_helpers;
using namespace map_viewers_ros;

const char* banner[] = {
  "local_mapper_viewer_node: shows the local maps under construction",
  "usage:",
  " local_mapper_dumper_node ",
  "once the gus starts",
  "1: toggles/untoggles the current view (and saves a lot of bandwidth)",
  "shift + left click on a node: highlights the local map of the node",
  0
};
  

int main(int argc, char** argv) {
  if (argc>1 && ! strcmp(argv[1],"-h")){
    system_utils::printBanner(banner);
    return 0;
  }

  ros::init(argc, argv, "local_map_viewer");
  ros::NodeHandle n;

  tf::TransformListener* listener = new tf::TransformListener(ros::Duration(60.0));

  QApplication* app=0; 
  LocalMapViewer* viewer=0;
  
  app=new QApplication(argc, argv);
  viewer = new LocalMapViewer();
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
