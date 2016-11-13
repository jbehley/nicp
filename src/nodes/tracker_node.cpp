#include <globals/system_utils.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>
#include <iostream>
#include <txt_io/static_transform_tree.h>
#include <txt_io/message_seq_synchronizer.h>
#include "nicp/depth_utils.h"
#include "nicp/nn_aligner.h"
#include "nicp/projective_aligner.h"
#include "map_tracker/tracker.h"
#include "map_tracker/multi_tracker.h"
#include "map_tracker/base_triggers.h"
#include "map_ros/cloud_publisher_trigger.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <ros_wrappers/image_message_listener.h>
#include "map_tracker/call_tracker_trigger.h"

#include "map_viewers/tracker_viewer.h"
#include <qapplication.h>
#include <qevent.h>

using namespace std;
using namespace Eigen;
using namespace txt_io;
using namespace map_tracker;
using namespace map_ros;
using namespace map_viewers;
using namespace ros_wrappers;

tf::TransformListener * listener = 0;
std::string base_link_frame_id = "";
std::string odom_frame_id = "/odom"; 
Tracker* tracker = 0;
std::vector<MessageSeqSynchronizer> synchronizers;

const char* banner[] = {
  "tracker_node: offline tracker working as ros node",
  "usage:",
  " tracker_node [options]",
  " where: ",
  "  -aligner:      [string] aligner type [projective or nn], default: projective",
  "  -config:       [string] config type [Xtion640x480, Kinect640x480, Xtion320x240], default: Xtion320x240",
  "  -max_distance: [float] max range of the beams to consider for alignment, default 3",
  "  -min_distance: [float] min range of the beams to consider for alignment, default 0",
  "  -cam_only:     flag, if set ignores the odometry and operates in the camera frame",
  "  -t:            [string] specifies which image topic to use, if unset will use all",
  "                          to issye multiple topics use \"-t <topic1>  -t <topic2> .. -t <topicN> \"",  "  -rgbt:         [string] specifies which rgb image topics to use. same as above. The number of -rgbt should match the order and the number of -t.",
  "  -single        [flag] if set uses the single projector, otherwise it uses the multi projector on all topics",
  "  -tf            [string] if set overrides the transforms in the file with theones of the file provided",
  "  -base_link_frame_id [string]: if specified listens for odom, and tracks the pose of the base_link specified",
  "  -odom_frame_id [string]: odometry frame, default /odom",
  "  -bpr:          [float] bad points ratio [float], default: 0.1",
  "  -damping:      [float] solver damping, default: 0",
  "  -shrink :      [int]   image downscaling (2 means half size), default: 1",
  "  -gui:          [flag]  enables the gui for fancy visualization"
  "  -o:            [string] output filename where to write the model, default \"\"",
  "once the gui has started you can ump the current cloud by pressing W ",
  0
};

void saveCloud(const std::string& prefix, int& num ){
  if (!tracker->referenceModel())
    return;
  if (!prefix.length())
    return;

  char buf[1024];
  sprintf(buf, "%s-%05d.cloud", prefix.c_str(), num);
  ofstream os(buf);
  tracker->referenceModel()->write(os);
  cerr << "Saving cloud in file " << buf << endl;
  num++;
}

int main(int argc, char **argv) {
  std::vector<std::string> depth_topics;
  std::vector<std::string> rgb_topics;
  std::list<ImageMessageListener*> camera_listeners;

  std::string alignerType="projective";
  std::string config="Xtion320x240";
  std::string transforms_filename = "";
  std::string output_filename="";

  float bad_points_ratio = 0.1;
  float damping = 0;
  float tbb = 5;
  float obb = 1;
  int shrink = 1;
  float min_distance = 0;
  bool cam_only = false;
  int c = 1;
  float max_distance = 3;
  bool gui = false;
  bool single = false;
  while (c<argc){
    if (! strcmp(argv[c], "-h")){
      system_utils::printBanner(banner);
      return 0;
    } else if (! strcmp(argv[c], "-cam_only")){
      cam_only=true;
      cerr << "cam_only" << endl;
    } else if (! strcmp(argv[c], "-single")){
      single=true;
      cerr << "single tracker" << endl;
    } else if (! strcmp(argv[c], "-gui")){
      gui=true;
      cerr << "enabled gui" << endl;
    } else if (! strcmp(argv[c], "-aligner")){
      c++;
      alignerType = argv[c];
    }
    else if (! strcmp(argv[c], "-max_distance")){
      c++;
      max_distance = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-min_distance")){
      c++;
      min_distance = atof(argv[c]);
    } else if (! strcmp(argv[c], "-base_link_frame_id")){
      c++;
      base_link_frame_id = argv[c];
    } else if (! strcmp(argv[c], "-odom_frame_id")){
      c++;
      odom_frame_id = argv[c];
    }
    else if (! strcmp(argv[c], "-t")){
      c++;
      depth_topics.push_back(argv[c]);
    }
    else if (! strcmp(argv[c], "-rgbt")){
      c++;
      rgb_topics.push_back(argv[c]);
    }
    else if (! strcmp(argv[c], "-config")){
      c++;
      config = argv[c];
    }
    else if (! strcmp(argv[c], "-shrink")){
      c++;
      shrink = atoi(argv[c]);
    }
    else if (! strcmp(argv[c], "-bpr")){
      c++;
      bad_points_ratio = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-damping")){
      c++;
      damping = atof(argv[c]);
    } else if (! strcmp(argv[c], "-tf")){
      c++;
      transforms_filename = argv[c];
    }

    else if (! strcmp(argv[c], "-o")){
      c++;
      output_filename = argv[c];
    } 
    c++;
  }

  StaticTransformTree * _transforms = 0;
  if (transforms_filename.length()){
    _transforms = new StaticTransformTree;
    _transforms->load(transforms_filename);
  }
  
  cerr << "resizing synchronizers" << endl;
  synchronizers.resize(depth_topics.size());
  if (rgb_topics.size()>0){
    if (rgb_topics.size()!=depth_topics.size()){
      cerr << "fatal error the number of RGB topics should be the same as the -t topics" << endl;
      return 0;
    }
    for (size_t i=0; i<depth_topics.size(); i++){
      std::vector<string> depth_plus_rgb_topic;
      depth_plus_rgb_topic.push_back(depth_topics[i]);
      depth_plus_rgb_topic.push_back(rgb_topics[i]);
      synchronizers[i].setTopics(depth_plus_rgb_topic);
    }
  } else {
    for (size_t i=0; i<depth_topics.size(); i++){
      std::vector<string> depth_topic;
      depth_topic.push_back(depth_topics[i]);
      synchronizers[i].setTopics(depth_topic);
    }
  }

  cerr << "constructing tracker ... ";
  if (depth_topics.size() < 2 || single) {
    tracker = Tracker::makeTracker(alignerType, config);
  } else {
    MultiTracker* multi_tracker = MultiTracker::makeTracker(alignerType, config);
    multi_tracker->init(depth_topics);
    tracker = multi_tracker;
  }
  if (! tracker) {
    cerr << "unknown tracker type [" << alignerType << "] aborting" << endl;
    return 0;
  }

  tracker->setBadPointsRatio(bad_points_ratio);
  tracker->aligner().solver().setDamping(damping);
  tracker->setImageShrink(shrink);
  tracker->setMaxDistance(max_distance);
  tracker->setImageShrink(shrink);

  cerr << " Done" << endl;



  // new VerboseTrigger(tracker, Tracker::TRACK_BROKEN, 0, "TRACK BROKEN!!!");
  new VerboseTrigger(tracker, Tracker::PROCESSING_DONE, 0, 
   		     "frame_count <frame_count> <seq> Time: <total_time>, FPS: <fps>, [cloud: <make_cloud_time>, alignment: <alignment_time>, validate: <validate_time>, merge: <merge_time>, tail: <tail_time>]");


  cerr << "ALL IN PLACE" << endl;
 
  ros::init(argc, argv, "tracker_node");
  if (base_link_frame_id.length()>0){
    cerr << "making listener" << endl;
    listener = new tf::TransformListener(ros::Duration(60.0));
  }
  ros::NodeHandle nh;
  image_transport::ImageTransport itr(nh);

  SensorMessageSorter sorter;
  sorter.setTimeWindow(0.);
  CallTrackerTrigger* caller = new CallTrackerTrigger(&sorter, 0, tracker, &synchronizers);
  for (std::vector<std::string>::iterator it = depth_topics.begin(); it!=depth_topics.end(); it++) {
    std::string topic = *it;
    ImageMessageListener* camera_listener = 
      new ImageMessageListener (&nh, &itr, &sorter, listener, odom_frame_id, base_link_frame_id);
    camera_listener->setDequeLength(1);
    camera_listener->subscribe(topic,depth_topics.size());
    cerr << "subscribing for topic: " << topic << endl;
    camera_listeners.push_back(camera_listener);
  }

  for (std::vector<std::string>::iterator it = rgb_topics.begin(); it!=rgb_topics.end(); it++) {
    std::string topic = *it;
    ImageMessageListener* camera_listener = 
      new ImageMessageListener (&nh, &itr, &sorter, listener, odom_frame_id, base_link_frame_id);
    camera_listener->setDequeLength(1);
    camera_listener->subscribe(topic,depth_topics.size());
    cerr << "subscribing to topic: " << topic << endl;
    camera_listeners.push_back(camera_listener);
  }

  tf::TransformBroadcaster* broadcaster = new tf::TransformBroadcaster;

  CloudPublisherTrigger* cloud_publisher = 
    new CloudPublisherTrigger(tracker, 
			      Tracker::PROCESSING_DONE,
			      100, nh, broadcaster);

  QApplication* app = 0;
  TrackerViewer* viewer = 0;
  if (gui) {
    app=new QApplication(argc, argv);
    viewer = new TrackerViewer(tracker);
    viewer->show();
  }

  bool show_changes=false;
  if (gui) {
    while (ros::ok()){
      ros::spinOnce();
      app->processEvents();
      QKeyEvent* event=viewer->lastKeyEvent();
      if (event){
	switch(event->key()) {
	case Qt::Key_R: 
	  tracker->clearStatus();
	  break;
	case Qt::Key_M: 
	  tracker->enableMerging(!tracker->mergingEnabled());
	  break;
	case Qt::Key_J: 
	  show_changes=!show_changes;
	  if (show_changes) {
	    tracker->setChangesThreshold(0.05);
	  } else {
	    tracker->setChangesThreshold(0);
	  }
	  break;
	default:;
	}
	viewer->keyEventProcessed();
      }
      viewer->updateGL();
      usleep(10000);
    }
  } else {
    ros::spin();
  }
}

