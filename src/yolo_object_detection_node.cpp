#include "yolo_object_detection_node.hpp"

namespace yolo_object_detection_ros2
{

YoloObjectDetectionNode::YoloObjectDetectionNode()
    : Node("yolo_object_detection_node", rclcpp::NodeOptions()
		    			 .allow_undeclared_parameters(true)
					 .automatically_declare_parameters_from_overrides(true))
{
  if (!readParameters()){
    rclcpp::shutdown();
  }

  init();
}

YoloObjectDetectionNode::~YoloObjectDetectionNode()
{
  delete yoloDetector_;
}

bool YoloObjectDetectionNode::readParameters(){
  std::string names_file, cfg_file, weights_file;
  std::string names_path, cfg_path, weights_path;

  // Path to names file
  this->get_parameter_or("yolo_model/names_file/name", names_file, std::string("obj.names"));
  this->get_parameter_or("names_path", names_path, std::string("/default"));
  names_ = names_path + "/" + names_file;

  // Path to cfg file
  this->get_parameter_or("yolo_model/cfg_file/name", cfg_file, std::string("yolov3-tiny-scale_truck.cfg"));
  this->get_parameter_or("cfg_path", cfg_path, std::string("/default"));
  cfg_ = cfg_path + "/" + cfg_file;

  // Path to weights file
  this->get_parameter_or("yolo_model/weights_file/name", weights_file, std::string("yolov3-tiny-scale_truck.weights"));
  this->get_parameter_or("weights_path", weights_path, std::string("/default"));
  weights_ = weights_path + "/" + weights_file;

  // Image parameters
  this->get_parameter_or("image/width", width_, 640);
  this->get_parameter_or("image/height", height_, 480);

  // Load common parameters
  this->get_parameter_or("image_view/enable_opencv", viewImage_, false);
  this->get_parameter_or("image_view/wait_key_delay", waitKeyDelay_, 1);
  this->get_parameter_or("image_view/enable_console_output", enableConsoleOutput_, false);

  return true;
}

void YoloObjectDetectionNode::init(){
  RCLCPP_INFO(this->get_logger(),"Yolo_Init Start");
  gettimeofday(&startTime_, NULL);

  // Initialize publisher and subscriber
  std::string rearCamTopicName;
  int rearCamQueueSize;
  std::string frontCamTopicName;
  int frontCamQueueSize;
  std::string runYoloTopicName;
  int runYoloQueueSize;

  std::string boundingBoxTopicName;
  int boundingBoxQueueSize;

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  this->get_parameter_or("subscribers/rear_camera_reading/topic", rearCamTopicName, std::string("rear_cam/image_raw"));
  this->get_parameter_or("subscribers/rear_camera_reading/queue_size", rearCamQueueSize, 1);
  this->get_parameter_or("subscribers/front_camera_reading/topic", frontCamTopicName, std::string("usb_cam/image_raw"));
  this->get_parameter_or("subscribers/front_camera_reading/queue_size", frontCamQueueSize, 1);
  this->get_parameter_or("subscribers/run_yolo_/topic", runYoloTopicName, std::string("run_yolo_flag"));
  this->get_parameter_or("subscribers/run_yolo_/queue_size", runYoloQueueSize, 1);
  
  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  this->get_parameter_or("publishers/Boundingbox/topic", boundingBoxTopicName, std::string("/yolo_object_detection/Boundingbox"));
  this->get_parameter_or("publishers/Boundingbox/queue_size", boundingBoxQueueSize, 1);


  /************************/
  /* Ros Topic Subscriber */
  /************************/
  frontCamImgSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(frontCamTopicName, frontCamQueueSize, std::bind(&YoloObjectDetectionNode::frontCamImgCallback,this, std::placeholders::_1));

  rearCamImgSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(rearCamTopicName, rearCamQueueSize, std::bind(&YoloObjectDetectionNode::rearCamImgCallback, this, std::placeholders::_1));

  runYoloSubscriber_ = this->create_subscription<ros2_msg::msg::Yoloflag>(runYoloTopicName, runYoloQueueSize, std::bind(&YoloObjectDetectionNode::runYoloCallback, this, std::placeholders::_1)); 

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  boundingBoxPublisher_ = this->create_publisher<ros2_msg::msg::Boundingbox>(boundingBoxTopicName, boundingBoxQueueSize);

  /***************/
  /* Start Setup */
  /***************/
  cv::Mat img_for_init(width_, height_, CV_8UC3, cv::Scalar(0,0,0)); 

  objectNames_ = objectNames(names_);

  yoloDetector_ = new Detector(cfg_, weights_, 0.2f/* thresh*/);
  yoloDetector_->detect(img_for_init);

  detectThread_ = std::thread(&YoloObjectDetectionNode::detectInThread, this);
}

std::vector<std::string> YoloObjectDetectionNode::objectNames(std::string const filename)
{
  std::ifstream file(filename);
  std::vector<std::string> file_lines;
  if (!file.is_open()) return file_lines;
  for(std::string line; getline(file, line);) file_lines.push_back(line);
  std::cout << "object names loaded\n";

  return file_lines;
}

void YoloObjectDetectionNode::runYoloCallback(const ros2_msg::msg::Yoloflag::SharedPtr msg){
  f_run_yolo_ = msg->f_run_yolo;
  r_run_yolo_ = msg->r_run_yolo;
}

void YoloObjectDetectionNode::rearCamImgCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (r_run_yolo_){
    cv_bridge::CvImagePtr cv_ptr;
  
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
      return;
    }
  
    if (cv_ptr) {
      std::scoped_lock lock(rear_cam_mutex_);
      rearCamImageCopy_ = cv_ptr->image.clone();
      resize(rearCamImageCopy_, rearCamImageCopy_, cv::Size(width_, height_));
      sec_ = msg->header.stamp.sec;
      nsec_ = msg->header.stamp.nanosec;
    }
  }
}

void YoloObjectDetectionNode::frontCamImgCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (f_run_yolo_){
    cv_bridge::CvImagePtr cv_ptr;
  
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
      return;
    }
  
    if (cv_ptr) {
      std::scoped_lock lock(front_cam_mutex_);
      frontCamImageCopy_ = cv_ptr->image.clone();
      resize(frontCamImageCopy_, frontCamImageCopy_, cv::Size(width_, height_));
    }
  }
}

void YoloObjectDetectionNode::detectInThread()
{
  RCLCPP_INFO(this->get_logger(),"detectInThread Start");
  struct timeval endTime;
  static double time = 0.0;
//  static int cnt = 0;
  uint8_t mark = 0;
  while(rclcpp::ok()){
    objects_.clear();
    if (f_run_yolo_ || r_run_yolo_){
      {
        std::scoped_lock lock(rear_cam_mutex_, front_cam_mutex_);
        std::string obj_name;
        if (!rearCamImageCopy_.empty()) {
          objects_ = yoloDetector_->detect(rearCamImageCopy_);
//          gettimeofday(&endTime, NULL);
//          cnt++;
//          time += ((endTime.tv_sec - sec) * 1000.0) + ((endTime.tv_usec - nsec) / 1000.0); //ms
//          delay_ = time / (double)cnt;
//          if (cnt > 3000){
//            time = 0.0;
//            cnt = 0;
//          }
//          delay_ = ((endTime.tv_sec - sec_) * 1000.0) + ((endTime.tv_usec - nsec_) / 1000.0); //ms   
          mark = 1;
        }
        else if (!frontCamImageCopy_.empty()) {
          objects_ = yoloDetector_->detect(frontCamImageCopy_);
	  mark = 2;
        }

        for(auto &i : objects_){
          if(objectNames_.size() > i.obj_id){
            obj_name = objectNames_[i.obj_id];
	  }
	}

        publishInThread(objects_, obj_name);
    
        if (viewImage_ && mark != 0) {
          cv::Mat draw_img;

          if (mark == 1) draw_img = rearCamImageCopy_.clone();
	  else if (mark == 2) draw_img = frontCamImageCopy_.clone();

          drawBoxes(draw_img, objects_);

          if (!draw_img.empty()) {
            cv::namedWindow("YOLO");
            cv::moveWindow("YOLO", 1280,520);
            cv::imshow("YOLO", draw_img);
            cv::waitKey(waitKeyDelay_);
          }
        }
      }
//      recordData(startTime_);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void YoloObjectDetectionNode::publishInThread(std::vector<bbox_t> objects, std::string obj_name)
{
  ros2_msg::msg::Boundingbox msg;
  unsigned int max_bbox_size = 0;
  bbox_t bbox;

  // Find max size bbox
  for (auto &i : objects) {
    if (max_bbox_size < i.w * i.h) {
      max_bbox_size = i.w * i.h;
      bbox = i;
    }
  }

  msg.name = obj_name;
  msg.x = bbox.x;
  msg.y = bbox.y;
  msg.w = bbox.w;
  msg.h = bbox.h;

  boundingBoxPublisher_->publish(msg);
}

void YoloObjectDetectionNode::drawBoxes(cv::Mat mat_img, std::vector<bbox_t> objects)
{
  int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

  for(auto &i : objects)
  {
    cv::Scalar color = obj_id_to_color(i.obj_id);
    cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);

    if(objectNames_.size() > i.obj_id)
    {
      std::string obj_name = objectNames_[i.obj_id];
      if (i.track_id > 0) obj_name += " - " + std::to_string(i.track_id);
      cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
      int max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);
      max_width = std::max(max_width, (int)i.w + 2);

      cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 35, 0)),
                    cv::Point2f(std::min((int)i.x + max_width, mat_img.cols - 1), std::min((int)i.y, mat_img.rows - 1)), color, CV_FILLED, 8, 0);
      putText(mat_img, obj_name, cv::Point2f(i.x, i.y - 16), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
    }
  }
}

void YoloObjectDetectionNode::recordData(struct timeval startTime){
  struct timeval currentTime;
  char file_name[] = "YOLO_log00.csv";
  static char file[128] = {0x00, };
  char buf[256] = {0x00,};
  static bool flag = false;
  double diff_time;
  std::ifstream read_file;
  std::ofstream write_file;
  std::string log_path = "/home/logfiles/";
  if(!flag){
    for(int i = 0; i < 100; i++){
      file_name[8] = i/10 + '0';  //ASCII
      file_name[9] = i%10 + '0';
      sprintf(file, "%s%s", log_path.c_str(), file_name);
      read_file.open(file);
      if(read_file.fail()){  //Check if the file exists
        read_file.close();
        write_file.open(file);
        break;
      }
      read_file.close();
    }
    write_file << "time,lvReqtoYoloDelay" << std::endl; //seconds, miliseconds
    flag = true;
  }
  else{
    gettimeofday(&currentTime, NULL);
    diff_time = ((currentTime.tv_sec - startTime.tv_sec)) + ((currentTime.tv_usec - startTime.tv_usec)/1000000.0);
    {
      std::scoped_lock lock(rear_cam_mutex_);
      sprintf(buf, "%.10e, %.10e", diff_time, delay_);
    }
    write_file.open(file, std::ios::out | std::ios::app);
    write_file << buf << std::endl;
  }
  write_file.close();
}

} // namespace yolo_object_detection
