// Copyright 2018 David V. Lu!!
#include <social_navigation_layers/social_layer.h>
#include <math.h>
#include <angles/angles.h>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <string>

#include <people_msgs_utils/utils.h>

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace social_navigation_layers
{
void SocialLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;
  first_time_ = true;
  people_sub_ = nh.subscribe("/people", 1, &SocialLayer::peopleCallback, this);

  // load parameters
  nh.param<bool>("enabled", enabled_, true);
  // store param in temporary variable
  double keep_time = 0.0;
  nh.param<double>("keep_time", keep_time, 0.0);
  people_keep_time_ = ros::Duration(keep_time);
}

void SocialLayer::peopleCallback(const people_msgs::People& people)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  people_frame_ = people.header.frame_id;
  // convert to custom type
  std::tie(people_, std::ignore) = people_msgs_utils::createFromPeople(people.people);
}

void SocialLayer::preprocessForBounds()
{
  // prepare list of people to compute costs for
  transformed_people_.clear();
  // non time-optimal, but safe in terms of `people_frame_` being empty etc.
  for (const auto& person: people_)
  {
    // transform people poses and velocities to costmap frame
    try
    {
      auto transform = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), people_frame_, ros::Time(0));
      // copy for transform
      auto person_copy = person;
      person_copy.transform(transform);
      transformed_people_.push_back(person_copy);
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      continue;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      continue;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      continue;
    }
  }
}

void SocialLayer::updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
  boost::recursive_mutex::scoped_lock lock(lock_);

  preprocessForBounds();
  updateBoundsFromPeople(min_x, min_y, max_x, max_y);
  if (first_time_)
  {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    first_time_ = false;
  }
  else
  {
    double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
    *min_x = std::min(last_min_x_, *min_x);
    *min_y = std::min(last_min_y_, *min_y);
    *max_x = std::max(last_max_x_, *max_x);
    *max_y = std::max(last_max_y_, *max_y);
    last_min_x_ = a;
    last_min_y_ = b;
    last_max_x_ = c;
    last_max_y_ = d;
  }
}
};  // namespace social_navigation_layers
