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

#include <people_msgs_utils/utils.h>

namespace social_navigation_layers
{
class FformationLayer : public ProxemicLayer
{
public:
  FformationLayer() : ProxemicLayer()
  {
  }

  virtual void updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y,
                            double* max_x, double* max_y)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);

    // prepare set of groups to compute costs for
    transformed_groups_.clear();
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
        transformed_groups_.push_back(group_copy);

        double point = computeAdjustmentsRadius(
          computeVarianceFromSpan(group_copy.getSpanX()),
          computeVarianceFromSpan(group_copy.getSpanY())
        ).second;

        *min_x = std::min(*min_x, group_copy.getPositionX() - point);
        *min_y = std::min(*min_y, group_copy.getPositionY() - point);
        *max_x = std::max(*max_x, group_copy.getPositionX() + point);
        *max_y = std::max(*max_y, group_copy.getPositionY() + point);
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

  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);
    if (!enabled_) return;

    if (groups_.empty())
      return;
    if (cutoff_ >= amplitude_)
      return;

    std::list<people_msgs::Person>::iterator p_it;
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    double res = costmap->getResolution();

    for (const auto& group: transformed_groups_)
    {
      double angle = group.getOrientationYaw();

      // variances defining F-formation's O-space spread
      double var_x = computeVarianceFromSpan(group.getSpanX());
      double var_y = computeVarianceFromSpan(group.getSpanY());
      // spread of the O-space
      // prev 'base': radius of the O-space along X direction
      double base;
      // prev 'point': radius of the O-space along Y direction
      double point;
      std::tie(base, point) = computeAdjustmentsRadius(var_x, var_y);

      unsigned int width = std::max(1, static_cast<int>((base + point) / res));
      unsigned int height = std::max(1, static_cast<int>((base + point) / res));

      double cx = group.getPositionX();
      double cy = group.getPositionY();

      double ox, oy;
      if (sin(angle) > 0)
        oy = cy - base;
      else
        oy = cy + (point - base) * sin(angle) - base;

      if (cos(angle) >= 0)
        ox = cx - base;
      else
        ox = cx + (point - base) * cos(angle) - base;


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
          double a = gaussian(x, y, cx, cy, amplitude_, var_x, var_y, angle);

          // count in both the tracking accuracy and the relations strength
          a *= group.getReliability() * group.getSocialRelationsStrength();

          if (a < cutoff_)
            continue;

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
  people_msgs_utils::Groups groups_;
  std::vector<people_msgs_utils::Group> transformed_groups_;
};
}; // namespace social_navigation_layers

PLUGINLIB_EXPORT_CLASS(social_navigation_layers::FformationLayer, costmap_2d::Layer)
