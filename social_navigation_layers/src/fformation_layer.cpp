// Copyright 2023
// Jarosław Karwowski
#include <social_navigation_layers/proxemic_layer.h>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <algorithm>
#include <list>
#include <string>

#include <social_navigation_layers/detections_storage.h>
#include <people_msgs_utils/utils.h>

namespace social_navigation_layers
{
class FformationLayer : public ProxemicLayer
{
public:
  FformationLayer() : ProxemicLayer()
  {
  }

  virtual void preprocessForBounds() override {
    /*
     * No need to process single humans @ref ProxemicLayer::preprocessForBounds
     * as this layer fully relies on groups (and group class transforms members too)
     */

    // prepare a set of groups to compute costs for; spatial attributes will be transformed to the costmap's frame
    std::vector<people_msgs_utils::Group> transformed_groups;
    // lookupTransform in each step is not time-optimal, but safe in terms of `people_frame_` being empty etc.
    for (const auto& group: groups_)
    {
      // transform poses to costmap frame
      try
      {
        auto transform = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), people_frame_, ros::Time(0));
        // copy for transform
        auto group_copy = group;
        group_copy.transform(transform);
        transformed_groups.push_back(group_copy);
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
    // update the storage
    detections_.update(ros::Time::now().toSec(), transformed_groups);
  }

  virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y) override
  {
    // No need to process single humans @ref ProxemicLayer::updateBoundsFromPeople as this layer fully relies on groups

    // this method finds which part of the costmap will be modified
    for (const auto& group: detections_.getBuffer())
    {
      // distance from the mean of the `Gaussian` (group center) that has a `cutoff` cost assigned
      double point = computeAdjustmentsRadius(
        computeVarianceFromSpan(group.getSpanX()),
        computeVarianceFromSpan(group.getSpanY())
      ).second;

      // find cost bounds, modelled by a circle
      *min_x = std::min(*min_x, group.getPositionX() - point);
      *min_y = std::min(*min_y, group.getPositionY() - point);
      *max_x = std::max(*max_x, group.getPositionX() + point);
      *max_y = std::max(*max_y, group.getPositionY() + point);
    }
  }

  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);
    if (!enabled_) return;

    if (detections_.getBuffer().empty())
      return;

    if (cutoff_ >= amplitude_)
      return;

    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    double res = costmap->getResolution();

    for (const auto& group: detections_.getBuffer())
    {
      double angle = group.getOrientationYaw();

      // variances defining F-formation's O-space spread
      double var_x = computeVarianceFromSpan(group.getSpanX());
      double var_y = computeVarianceFromSpan(group.getSpanY());
      // spread of the O-space
      // radius of the O-space along 'longer' direction
      double point = computeAdjustmentsRadius(var_x, var_y).second;

      /*
       * TODO: width and height can be optimized (previously base + point, but this doesn't cover required area
       * when asymmetry is significant)
       */
      unsigned int width = std::max(1, static_cast<int>((2.0 * point) / res));
      unsigned int height = std::max(1, static_cast<int>((2.0 * point) / res));

      double cx = group.getPositionX();
      double cy = group.getPositionY();

      // TODO: ox and oy can be optimized, but with attention to the covered area when asymmetry is significant
      double oy = cy - point;
      double ox = cx - point;

      int dx, dy;
      costmap->worldToMapNoBounds(ox, oy, dx, dy);

      int start_x = 0, start_y = 0, end_x = width, end_y = height;
      if (dx < 0)
        start_x = -dx;
      else if (dx + width > costmap->getSizeInCellsX())
        end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - dx);

      if (static_cast<int>(start_x + dx) < min_i)
        start_x = min_i - dx;
      if (static_cast<int>(end_x + dx) > max_i)
        end_x = max_i - dx;

      if (dy < 0)
        start_y = -dy;
      else if (dy + height > costmap->getSizeInCellsY())
        end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - dy);

      if (static_cast<int>(start_y + dy) < min_j)
        start_y = min_j - dy;
      if (static_cast<int>(end_y + dy) > max_j)
        end_y = max_j - dy;

      double bx = ox + res / 2,
             by = oy + res / 2;
      for (int i = start_x; i < end_x; i++)
      {
        for (int j = start_y; j < end_y; j++)
        {
          unsigned char old_cost = costmap->getCost(i + dx, j + dy);
          if (old_cost == costmap_2d::NO_INFORMATION)
            continue;

          double x = bx + i * res, y = by + j * res;
          // stores cost 'amplitude'
          double a = 0.0;

          // whether to clear old detections or to operate normally (i.e. updating with a new data)
          if (!detections_.bufferWillBeErased())
          {
            a = gaussian(x, y, cx, cy, amplitude_, var_x, var_y, angle);
            // count in both the tracking accuracy and the relations strength
            a *= group.getReliability() * group.getSocialRelationsStrength();

            // if amplitude is too small, do not update costs
            if (a < cutoff_)
            {
              continue;
            }
          }

          unsigned char cvalue = (unsigned char) a;
          costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
        }
      }
    }
  }

  virtual std::pair<double, double> computeAdjustmentsRadius(double cov_x, double cov_y) const
  {
    double base = get_radius(cutoff_, amplitude_, std::min(cov_x, cov_y));
    double point = get_radius(cutoff_, amplitude_, std::max(cov_x, cov_y));
    return std::make_pair(base, point);
  }

  /**
   * @param span length of the group model along 1 direction (e.g. 2 lengths of the semimajor axis)
   * @param sigma_num how many sigmas to use to compute std deviation using the 68–95–99.7 rule
   * @return double variance
   */
  static double computeVarianceFromSpan(double span, int sigma_num = 2) {
    double stddev = (span / 2.0) / static_cast<double>(sigma_num);
    return std::pow(stddev, 2.0);
  }

  virtual void peopleCallback(const people_msgs::People& people) override {
    boost::recursive_mutex::scoped_lock lock(lock_);
    people_frame_ = people.header.frame_id;
    // convert to custom types
    std::tie(people_, groups_) = people_msgs_utils::createFromPeople(people.people);
  }

protected:
  virtual void configure(ProxemicLayerConfig& config, uint32_t level) override
  {
    ProxemicLayer::configure(config, level);
    detections_.setKeepTime(people_keep_time_.toSec());
  }

  people_msgs_utils::Groups groups_;
  DetectionsStorage<people_msgs_utils::Group> detections_;
};
}; // namespace social_navigation_layers

PLUGINLIB_EXPORT_CLASS(social_navigation_layers::FformationLayer, costmap_2d::Layer)
