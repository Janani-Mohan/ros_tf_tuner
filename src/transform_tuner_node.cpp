/**************************************************************************
**
**  Class:  file: transform_tuner_node.cpp
**
**  Author: Luca Marchionni (luca)
**  Email : luca.marchionni@pal-robotics.com
**  Created on: 13-4-2012
**
**  Copyright (c) 2010 PAL Robotics sl. All Rights Reserved
**************************************************************************/

#include <boost/thread/recursive_mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <ros_tf_tuner/TransformTunerConfig.h>
#include <boost/thread/locks.hpp>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/noncopyable.hpp>

namespace pal
{
  namespace util
  {
    class TransformTuner : boost::noncopyable
    {
      public:
        TransformTuner() :
          x_(0),
          y_(0),
          z_(0),
          roll_(0),
          pitch_(0),
          yaw_(0),
          parent_frame_id_("map"),
          child_frame_id_("map1")
        {
          dsrv_ = new dynamic_reconfigure::Server<ros_tf_tuner::TransformTunerConfig>(ros::NodeHandle("~"));
          dynamic_reconfigure::Server<ros_tf_tuner::TransformTunerConfig>::CallbackType cb = boost::bind(&TransformTuner::reconfigureCB, this, _1, _2);
          dsrv_->setCallback(cb);
        }

        void reconfigureCB(ros_tf_tuner::TransformTunerConfig &config, uint32_t level){
          boost::recursive_mutex::scoped_lock l(configuration_mutex_);

          parent_frame_id_ = config.parent;
          child_frame_id_ = config.child;
          x_ = config.x;
          y_ = config.y;
          z_ = config.z;
          pitch_ = config.pitch;
          yaw_ = config.yaw;
          roll_ = config.roll;

          ROS_INFO_STREAM("parent " <<  parent_frame_id_ << " child " << child_frame_id_);
          ROS_INFO_STREAM("x y z " <<  x_ << " " << y_ << " " << z_);
          ROS_INFO_STREAM("yaw pitch roll " <<  yaw_ << " " << pitch_ << " " << roll_);
        }

      private:
        boost::recursive_mutex configuration_mutex_;
        dynamic_reconfigure::Server<ros_tf_tuner::TransformTunerConfig> *dsrv_;
      public:
        double x_,y_,z_,roll_,pitch_,yaw_;
        std::string parent_frame_id_, child_frame_id_;
    };

  }
}

int main(int argc, char** argv){

  ros::init(argc, argv, "transform_tuner_node");
  pal::util::TransformTuner tf_tuner;

  tf::TransformBroadcaster broadcaster;

  ros::Rate rate(10.0);
  while(ros::ok())
  {
    ros::spinOnce();
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = tf_tuner.parent_frame_id_;
    transform.child_frame_id = tf_tuner.child_frame_id_;
    transform.header.stamp = ros::Time::now();
    transform.transform.translation.x = tf_tuner.x_;
    transform.transform.translation.y = tf_tuner.y_;
    transform.transform.translation.z = tf_tuner.z_;
    transform.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(tf_tuner.roll_, tf_tuner.pitch_, tf_tuner.yaw_);
    broadcaster.sendTransform(transform);
    ROS_INFO_STREAM("loop");
    rate.sleep();
  }

  return(0);

}
