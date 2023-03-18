// Copyright 2018 David V. Lu!!
#include <social_navigation_layers/proxemic_layer.h>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <algorithm>
#include <list>
#include <string>

namespace social_navigation_layers
{
class PassingLayer : public ProxemicLayer
{
public:
  PassingLayer() : ProxemicLayer()
  {
  }

  virtual void updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y,
                            double* max_x, double* max_y)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);

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

        double point = computeAdjustmentsRadius().second;

        *min_x = std::min(*min_x, person_copy.getPositionX() - point);
        *min_y = std::min(*min_y, person_copy.getPositionY() - point);
        *max_x = std::max(*max_x, person_copy.getPositionX() + point);
        *max_y = std::max(*max_y, person_copy.getPositionY() + point);
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

    if (people_.empty())
      return;
    if (cutoff_ >= amplitude_)
      return;

    std::list<people_msgs::Person>::iterator p_it;
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    double res = costmap->getResolution();

    for (const auto& person: transformed_people_)
    {
      double angle = person.getOrientationYaw() + 1.51;

      // spread of the passing space
      // prev 'base': radius of the passing space along heading direction
      double base;
      // prev 'point': radius of the passing space along 'side' direction
      double point;
      std::tie(base, point) = computeAdjustmentsRadius();

      unsigned int width = std::max(1, static_cast<int>((base + point) / res));
      unsigned int height = std::max(1, static_cast<int>((base + point) / res));

      double cx = person.getPositionX();
      double cy = person.getPositionY();

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
          double ma = atan2(y - cy, x - cx);
          double diff = angles::shortest_angular_distance(angle, ma);
          double a;
          if (fabs(diff) < M_PI / 2)
            a = gaussian(x, y, cx, cy, amplitude_, computeVarianceHeading(), computeVarianceSide(), angle);
          else
            continue;

          if (a < cutoff_)
            continue;
          unsigned char cvalue = (unsigned char) a;
          costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
        }
      }
    }
  }

  virtual double computeVarianceHeading(double speed = 0.0) const override
  {
    // according to Kirby's thesis, eqn. 4.9
    return 2.0;
  }

  virtual double computeVarianceSide(double speed = 0.0) const override
  {
    // according to Kirby's thesis, eqn. 4.10
    return 0.25;
  }

  virtual double computeVarianceRear(double speed = 0.0) const override
  {
    // according to Kirby's thesis, eqn. 4.11
    return 0.01;
  }

  // radius does not change with speed
  virtual std::pair<double, double> computeAdjustmentsRadius(double speed = 0.0) const override
  {
    double base = get_radius(cutoff_, amplitude_, computeVarianceSide());
    double point = get_radius(cutoff_, amplitude_, computeVarianceHeading());
    return std::make_pair(base, point);
  }

};
};  // namespace social_navigation_layers

PLUGINLIB_EXPORT_CLASS(social_navigation_layers::PassingLayer, costmap_2d::Layer)
