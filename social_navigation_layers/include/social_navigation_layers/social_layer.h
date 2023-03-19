// Copyright 2018 David V. Lu!!
#ifndef SOCIAL_NAVIGATION_LAYERS_SOCIAL_LAYER_H
#define SOCIAL_NAVIGATION_LAYERS_SOCIAL_LAYER_H
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <people_msgs/People.h>
#include <people_msgs_utils/person.h>
#include <list>

namespace social_navigation_layers
{
class SocialLayer : public costmap_2d::Layer
{
public:
  SocialLayer()
  {
    layered_costmap_ = NULL;
  }

  /**
   * Initializes parameters for the layer
   *
   * The costmap calls this method when it loads this class as a plugin.
   *
   * @url https://github.com/DLu/navigation_layers/compare/melodic...BadgerTechnologies:navigation_layers:develop
   */
  virtual void onInitialize();

  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);

  /**
   * Updates the master costmap cells within the bounded window
   *
   * This method is called by the costmap on every update cycle. It calculates the cost value for each costmap cell
   * surrounding each human reported and applies them to the costmap.
   *
   * @param master_grid reference to 2D costmap
   * @param min_i bounding box of changes to costmap
   * @param min_j bounding box of changes to costmap
   * @param max_i bounding box of changes to costmap
   * @param max_j bounding box of changes to costmap
   *
   * @url https://github.com/DLu/navigation_layers/compare/melodic...BadgerTechnologies:navigation_layers:develop
   */
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) = 0;

  /**
   * Calculates the min/max bounding box of costmap changes for all people
   *
   * This is called by the costmap via the updateBounds method in the parent SocialLayer class. Instead of updating
   * all of the cells for the entire costmap, only update a region of interest. This iterates through all of the people
   * detected and finds the minimum bounding box to contain all of the changes for this update cycle.
   *
   * @param min_x pointer to bounding box of changes
   * @param min_y pointer to bounding box of changes
   * @param max_x pointer to bounding box of changes
   * @param max_y pointer to bounding box of changes
   *
   * @url https://github.com/DLu/navigation_layers/compare/melodic...BadgerTechnologies:navigation_layers:develop
   */
  virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y) = 0;

  bool isDiscretized()
  {
    return false;
  }

protected:
  void peopleCallback(const people_msgs::People& people);
  std::string people_frame_;
  ros::Subscriber people_sub_;
  people_msgs_utils::People people_;
  std::list<people_msgs_utils::Person> transformed_people_;
  ros::Duration people_keep_time_;
  boost::recursive_mutex lock_;
  bool first_time_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
};
}  // namespace social_navigation_layers

#endif  // SOCIAL_NAVIGATION_LAYERS_SOCIAL_LAYER_H
