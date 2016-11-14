#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <tf/transform_listener.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>
#include <iostream>
#include "nicp/depth_utils.h"
#include "nicp/nn_aligner.h"
#include "nicp/projective_aligner.h"
#include <qglviewer.h>
#include <qapplication.h>
#include "map_tracker/tracker.h"
#include "map_tracker/base_triggers.h"
#include "local_mapper/local_mapper.h"
#include "map_tracker/multi_tracker.cpp"
#include <txt_io/tf_overrider_trigger.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <globals/system_utils.h>
#include <ros_wrappers/image_message_listener.h>
#include <txt_io/message_enlister_trigger.h>
#include <txt_io/message_dumper_trigger.h>
#include <txt_io/pinhole_image_message.h>
#include <txt_io/message_seq_synchronizer.h>

#include "map_ros/map_msgs_ros.h"
#include "map_ros/local_mapper_ros.h"
#include <pthread.h>
#include "map_ros/cloud_publisher_trigger.h"
#include <stdexcept>

using namespace std;
using namespace Eigen;
using namespace txt_io;
using namespace ros_wrappers;
using namespace map_tracker;
using namespace map_ros;

Tracker* tracker = 0;
VerboseTrigger* vt = 0;

int current_frame_skip = 1;
float current_frame_skipf = 1.0;
bool auto_frame_skip_enabled = false;
float sorter_delay_time = 1;

SensorMessageSorter* sorter = new SensorMessageSorter;
SensorMessageList* _messages = new SensorMessageList;
bool thread_run = false;
MessageWriter* writer = 0;
ProfilerTrigger* profiler = 0;
std::vector<MessageSeqSynchronizer> synchronizers;

ros::Publisher* frameSkipPublisher = NULL;
ros::Publisher* queueSizePublisher = NULL;

const char* banner[] = {
  "local_mapper_gui_app: offline local mapper working on dump files written with thin_message_dumper_node",
  "usage:",
  " local_mapper_gui_app [options]",
  " where: ",
  "  -aligner:      [string] aligner type [projective or nn], default: projective",
  "  -config:       [string] config type [Xtion640x480, Kinect640x480, Xtion320x240], default: Xtion320x240",
  "  -max_distance: [float] max range of the beams to consider for alignment, default 3",
  "  -min_distance: [float] min range of the beams to consider for alignment, default 0",
  "  -clipping_distance: [float] distance at which to clip a local map, default 0",
  "  -cam_only:     flag, if set ignores the odometry and operates in the camera frame",
  "  -t:            [string] specifies which image topic to use, if unset will use all",
  "                          to issye multiple topics use \"-t <topic1>  -t <topic2> .. -t <topicN> \"",  "  -rgbt:         [string] specifies which rgb image topics to use. same as above. The number of -rgbt should match the order and the number of -t.",
  "  -single        [flag] if set uses the single projector, otherwise it uses the multi projector on all topics",
  "  -tf            [string] if set overrides the transforms in the file with theones of the file provided",
  "  -base_link_frame_id [string]: if specified listens for odom, and tracks the pose of the base_link specified",
  "  -odom_frame_id [string]: specifies the frame id of the odometry, default /odom",
  "  -bpr:          [float] bad points ratio [float], default: 0.1",
  "  -damping:      [float] solver damping, default: 100",
  "  -shrink :      [int]   image downscaling (2 means half size), default: 1",
  "  -skip :        [int]   skip each x frames, default: 1, put 'auto' for automatic frame skip",
  "  -pubInfo :     [int]   1 to publish the current queue size and frame skip, default: 0",
  "  -tbb:          [float] when to break the local map (translation), default: 2",
  "  -obb:          [float] when to break the local map (orientation), default: 6.28",
  "  -sorter_delay_time  [float] latency for synchronizing messages, default 1",
  "  -o:            [string] output filename where to write the local maps, default \"\"",
  "once the gui has started, with shift + left click on a node you toggle the display of the local map",
  0
};


void* run_local_mapper(void*) {
  while(thread_run) {
    if (! _messages || _messages->empty()){
      usleep(100000);
      continue;
    }

    std::tr1::shared_ptr<BaseSensorMessage> msg = _messages->front();
    _messages->pop_front();
    PinholeImageMessage* img = dynamic_cast<PinholeImageMessage*>(msg.get());
    if (! img)
      return 0 ;

    Matrix6f odom_info;
    odom_info.setIdentity();
    if (! img->hasOdom()){
      odom_info.setZero();
    } 
    
    if  (writer) {
      writer->writeMessage(*img);
      img->writeBack();
    } else
      img->untaint();


    PinholeImageMessage* depth_img=0, *rgb_img=0;
    size_t i;
    for (i=0; i<synchronizers.size(); i++){
      synchronizers[i].putMessage(msg);
      if (synchronizers[i].messagesReady()) {
	depth_img=dynamic_cast<PinholeImageMessage*>(synchronizers[i].messages()[0].get());
	if (synchronizers[i].messages().size()>1)
	  rgb_img=dynamic_cast<PinholeImageMessage*>(synchronizers[i].messages()[1].get());
	break;
      }
    }
    if (! depth_img)
      continue;

    RGBImage rgb_image;
    if (rgb_img)
      rgb_image =rgb_img->image();

    //cerr << depth_img << " " << rgb_img  << endl;

    tracker->processFrame(depth_img->image(),
			  rgb_image, 
			  depth_img->cameraMatrix(),
			  depth_img->depthScale(),
			  depth_img->seq(),
			  depth_img->timestamp(),
			  depth_img->topic(),
			  depth_img->frameId(),
			  depth_img->offset(),
			  depth_img->odometry(),
			  odom_info);
    synchronizers[i].reset();
    float cpu = profiler->usageCounter()->totalCPUUsage();
    size_t mem = profiler->usageCounter()->totalMemory();

    if (auto_frame_skip_enabled) {
        if (_messages->size() > 10) current_frame_skipf += 0.1;   // XXX: these are random working values
        if (_messages->size() < 9) current_frame_skipf -= 0.1;
        if (current_frame_skipf > 10.0) current_frame_skipf = 10.0;
        if (current_frame_skipf < 1.0) current_frame_skipf = 1.0;
        int new_frame_skip = floor(current_frame_skipf);
        if (new_frame_skip != current_frame_skip) {
            tracker->setFrameSkip(new_frame_skip);
            current_frame_skip = new_frame_skip;
        }
    }
    if (frameSkipPublisher) frameSkipPublisher->publish(current_frame_skip);
    if (queueSizePublisher) queueSizePublisher->publish(_messages->size());
    if (vt) {
      printf("\r skip: %d, queue: %d, cpu: %f, mem: %ld, %s       ", current_frame_skip, (int) _messages->size(), cpu, mem, vt->lastMessage().c_str());
      fflush(stdout);
    }
  }
}

int main(int argc, char **argv) {
  std::string alignerType="projective";
  std::string config="Xtion320x240";
  std::string output_filename="";
  std::string odom_frame_id = "/odom";
  bool cam_only=false;
  float bad_points_ratio = 0.1;
  float damping = 100;
  float tbb = 2;
  float obb = 6.28;
  float clipping_distance = 0;
  int shrink = 1;
  int skip = 0;
  std::string transforms_filename = "";
  std::string dump_filename = "";
  std::string base_link_frame_id = "";
  float max_distance = 3;
  float min_distance = 0;
  int c = 1;
  bool single = false;
  bool pubInfo = false;
  std::vector<std::string> depth_topics;
  std::vector<std::string> rgb_topics;
  
  while (c<argc){
    if (! strcmp(argv[c], "-h")){
      system_utils::printBanner(banner);
      return 0;
    } else if (! strcmp(argv[c], "-cam_only")){
      cam_only=true;
      cerr << "CAM_ONLY" << endl;
    } else if (! strcmp(argv[c], "-single")){
      single=true;
      cerr << "single tracker" << endl;
    }
    else if (! strcmp(argv[c], "-aligner")){
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
    }
    else if (! strcmp(argv[c], "-config")){
      c++;
      config = argv[c];
    }
    else if (! strcmp(argv[c], "-base_link_frame_id")){
      c++;
      base_link_frame_id = argv[c];
    }
    else if (! strcmp(argv[c], "-odom_frame_id")){
      c++;
      odom_frame_id = argv[c];
    }
    else if (! strcmp(argv[c], "-shrink")){
      c++;
      shrink = atoi(argv[c]);
    } 
    else if (! strcmp(argv[c], "-t")){
      c++;
      depth_topics.push_back(argv[c]);
    }
    else if (! strcmp(argv[c], "-rgbt")){
      c++;
      rgb_topics.push_back(argv[c]);
    }
    else if (! strcmp(argv[c], "-bpr")){
      c++;
      bad_points_ratio = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-damping")){
      c++;
      damping = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-tf")){
      c++;
      transforms_filename = argv[c];
    }
    else if (! strcmp(argv[c], "-tbb")){
      c++;
      tbb = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-obb")){
      c++;
      obb = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-sorter_delay_time")){
      c++;
      sorter_delay_time = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-clipping_distance")){
      c++;
      clipping_distance = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-skip")){
      c++;
      if (strcmp(argv[c], "auto") == 0) {
        auto_frame_skip_enabled = true;
        printf("*** Automatic frame skip enabled ***\n");
        skip = 1;
      }
      else skip = atoi(argv[c]);
      current_frame_skip = skip;
    }
    else if (! strcmp(argv[c], "-pubInfo")) {
      c++;
      pubInfo = atoi(argv[c]);
    }
    else if (! strcmp(argv[c], "-o")){
      c++;
      output_filename = argv[c];
    }
    c++;
  }

  tf::TransformListener * listener = 0;
  std::vector<ImageMessageListener*> camera_listeners;
  
  ros::init(argc, argv, "local_mapper_node");
  if (base_link_frame_id.length()>0){
    cerr << "making listener" << endl;
    listener = new tf::TransformListener(ros::Duration(60.0));
  }
  ros::NodeHandle nh;
  image_transport::ImageTransport itr(nh);

  if (pubInfo) {
    frameSkipPublisher = new ros::Publisher(nh.advertise<std_msgs::Int32>("/tracker/frame_skip", 10));
    queueSizePublisher = new ros::Publisher(nh.advertise<std_msgs::Int32>("/tracker/queue_size", 10));
  }  

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

  sorter->setTimeWindow(sorter_delay_time);
  cerr << "sorter delay time: " << sorter->timeWindow() << endl;
  tracker->setBadPointsRatio(bad_points_ratio);
  tracker->aligner().solver().setDamping(damping);
  tracker->setImageShrink(shrink);
  tracker->setMaxDistance(max_distance);
  tracker->setFrameSkip(skip);  
  tracker->setImageShrink(shrink);
  ProjectiveAligner* aligner = dynamic_cast<ProjectiveAligner*>(&tracker->aligner());

  cerr << " Done" << endl;

  boss::Serializer* ser = 0;
  boss::IdContext * context = 0;
  if (output_filename != "") {
    ser  = new boss::Serializer();
    ser->setFilePath(output_filename);
    ser->setBinaryPath(output_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>");
    context=ser;
  } else {
    context = new boss::IdContext;
  }

  if (dump_filename!= ""){
    writer=new MessageWriter;
    writer->open(dump_filename);
  }
  
  LocalMapperRos* local_map_maker = new LocalMapperRos(tracker, 1, ser, context);

  local_map_maker->setTrajectoryMaxTranslation(tbb);
  local_map_maker->setTrajectoryMaxOrientation(obb);
  local_map_maker->setClippingDistance(clipping_distance);

  vt = new VerboseTrigger(tracker, Tracker::PROCESSING_DONE, 0, 
		     "<seq> Time: <total_time>, FPS: <fps>, [cloud: <make_cloud_time>, alignment: <alignment_time>, validate: <validate_time>, merge: <merge_time>, tail: <tail_time>]");


  vt->setOutputStream(0);

  cerr << "ALL IN PLACE" << endl;
  
  sorter->setWriteBackEnabled(false);
  if (transforms_filename.length()){
    StaticTransformTree * transforms = 0;
    transforms = new StaticTransformTree;
    transforms->load(transforms_filename);
    TfOverriderTrigger* tf_overrider = new TfOverriderTrigger(sorter, 0, transforms);
  }
 
  MessageEnlisterTrigger* enlister = new MessageEnlisterTrigger(sorter, 10, _messages);
  
  SystemUsageCounter* usage_counter = new SystemUsageCounter;
  profiler = new ProfilerTrigger(tracker, Tracker::PROCESSING_DONE, 100, usage_counter);
  //CallTrackerTrigger* caller = new CallTrackerTrigger(&sorter, 1, tracker);
  for (std::vector<std::string>::iterator it = depth_topics.begin(); it!=depth_topics.end(); it++) {
    std::string topic = *it;
    ImageMessageListener* camera_listener = 
      new ImageMessageListener (&nh, &itr, sorter, listener, odom_frame_id, base_link_frame_id);
    camera_listener->subscribe(topic);
    cerr << "subscribing to topic: " << topic << endl;
    camera_listeners.push_back(camera_listener);
  }
  
  for (std::vector<std::string>::iterator it = rgb_topics.begin(); it!=rgb_topics.end(); it++) {
    std::string topic = *it;
    ImageMessageListener* camera_listener = 
      new ImageMessageListener (&nh, &itr, sorter, listener, odom_frame_id, base_link_frame_id);
    camera_listener->subscribe(topic);
    cerr << "subscribing to topic: " << topic << endl;
    camera_listeners.push_back(camera_listener);
  }

  tf::TransformBroadcaster* broadcaster = new tf::TransformBroadcaster;

  CloudPublisherTrigger* cloud_publisher = 
    new CloudPublisherTrigger(tracker, 
  					  Tracker::PROCESSING_DONE,
  					  2, nh, broadcaster);
  local_map_maker->init(nh);
  pthread_t runner;
  thread_run  = true;
  pthread_create(&runner, 0, run_local_mapper, 0);
  ros::spin();

  if (queueSizePublisher) { queueSizePublisher->shutdown(); delete queueSizePublisher; }
  if (frameSkipPublisher) { frameSkipPublisher->shutdown(); delete frameSkipPublisher; }
}


