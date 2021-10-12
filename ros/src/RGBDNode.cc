#include "RGBDNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    image_transport::ImageTransport image_transport (node_handle);

    RGBDNode node (ORB_SLAM2::System::RGBD, node_handle, image_transport);

    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}


RGBDNode::RGBDNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth_registered/image_raw", 1);
  object_subscriber_ = new message_filters::Subscriber<sem_img_msg::centerBdboxes> (node_handle, "/camera/boundingbox", 1);
  camera_info_topic_ = "/camera/rgb/camera_info";

  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_, *object_subscriber_);
  sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2, _3));
}


RGBDNode::~RGBDNode () {
  delete rgb_subscriber_;
  delete depth_subscriber_;
  delete object_subscriber_;
  delete sync_;
}


void RGBDNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD, const sem_img_msg::centerBdboxes::ConstPtr &bdbox) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  ORB_SLAM2::objectdetection objects;
  for (unsigned int i = 0; i < bdbox->centerBdboxes.size(); i++)
  {
    objects.add_object(bdbox->centerBdboxes[i].probability, bdbox->centerBdboxes[i].x_cen, bdbox->centerBdboxes[i].y_cen, bdbox->centerBdboxes[i].width, bdbox->centerBdboxes[i].height, bdbox->centerBdboxes[i].id, bdbox->centerBdboxes[i].Class, bdbox->centerBdboxes[i].depth);
    // if (bdbox->centerBdboxes[i].depth != -1)
    //     ROS_INFO("bdbox->centerBdboxes[i].Class %s bdbox->centerBdboxes[i].depth %f", bdbox->centerBdboxes[i].Class.c_str(), bdbox->centerBdboxes[i].depth);
  }

  current_frame_time_ = msgRGB->header.stamp;

  orb_slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(), objects);

  Update ();
}
