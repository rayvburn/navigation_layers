// Copyright 2018 David V. Lu!!
#include <social_navigation_layers/proxemic_layer.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <list>
PLUGINLIB_EXPORT_CLASS(social_navigation_layers::ProxemicLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew)
{
  double dx = x - x0, dy = y - y0;
  double h = sqrt(dx * dx + dy * dy);
  double angle = atan2(dy, dx);
  double mx = cos(angle - skew) * h;
  double my = sin(angle - skew) * h;
  double f1 = pow(mx, 2.0) / (2.0 * varx),
         f2 = pow(my, 2.0) / (2.0 * vary);
  return A * exp(-(f1 + f2));
}

double get_radius(double cutoff, double A, double var)
{
  return sqrt(-2 * var * log(cutoff / A));
}


namespace social_navigation_layers
{
void ProxemicLayer::onInitialize()
{
  SocialLayer::onInitialize();
  ros::NodeHandle nh("~/" + name_), g_nh;
  server_ = new dynamic_reconfigure::Server<ProxemicLayerConfig>(nh);
  f_ = boost::bind(&ProxemicLayer::configure, this, _1, _2);
  server_->setCallback(f_);
}

void ProxemicLayer::updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y)
{
  // this method finds which part of the costmap will be modified
  std::list<people_msgs::Person>::iterator p_it;

  for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
  {
    people_msgs::Person person = *p_it;

    // velocity vector magnitude
    double mag = sqrt(pow(person.velocity.x, 2) + pow(person.velocity.y, 2));
    // possibly strengthen impact of velocity with `factor_` (factor is at least 1.0)
    double factor = 1.0 + mag * factor_;
    // distance from the mean of the `Gaussian` (person center) that has `cutoff` cost assigned
    double point = get_radius(cutoff_, amplitude_, covar_ * factor);

    // find cost bounds, modelled by a circle
    *min_x = std::min(*min_x, person.position.x - point);
    *min_y = std::min(*min_y, person.position.y - point);
    *max_x = std::max(*max_x, person.position.x + point);
    *max_y = std::max(*max_y, person.position.y + point);
  }
}

void ProxemicLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  if (!enabled_) return;

  if (people_list_.people.size() == 0)
    return;
  if (cutoff_ >= amplitude_)
    return;

  std::list<people_msgs::Person>::iterator p_it;
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  double res = costmap->getResolution();

  // iterate over all people data
  for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
  {
    // current person
    people_msgs::Person person = *p_it;
    // yaw angle estimate of the person (will be quite `off` once person is standing)
    double angle = atan2(person.velocity.y, person.velocity.x);
    // overall speed of the person
    double mag = sqrt(pow(person.velocity.x, 2) + pow(person.velocity.y, 2));
    // possibly strengthen impact of velocity with `factor_` (factor is at least 1.0)
    double factor = 1.0 + mag * factor_;
    // radius of the proxemics zone if person is stationary
    double base = get_radius(cutoff_, amplitude_, covar_);
    // radius of the proxemics zone if person is moving (including the velocity factor)
    double point = get_radius(cutoff_, amplitude_, covar_ * factor);

    /*
     * Width and height (in costmap pixels) of the proxemics zone
     * Width and height values are prepared here for the worst case scenario, when either
     * width or height can have length of at most (base+point) and at least of 1 px
     */
    unsigned int width = std::max(1, static_cast<int>((base + point) / res)),
                 height = std::max(1, static_cast<int>((base + point) / res));

    // person center coordinates
    double cx = person.position.x, cy = person.position.y;

    /*
     * Compute the world coordinates of the right-down vertex (y_min, x_min) of the Gaussian.
     * The Gaussian already includes the asymmetry caused by a velocity of the person (prolongation along
     * the heading direction)
     */
    double ox, oy;
    if (sin(angle) > 0)
      oy = cy - base;
    else
      oy = cy + (point - base) * sin(angle) - base;

    if (cos(angle) >= 0)
      ox = cx - base;
    else
      ox = cx + (point - base) * cos(angle) - base;

    // find correspondent map coordinates to the world ones;
    // NOTE: map orientation matches global coordinate system's
    int dx, dy;
    costmap->worldToMapNoBounds(ox, oy, dx, dy);

    /*
     * Recompute range that we're going to modify costs for;
     * start_x + dx indicate first x-coordinate that will be modified by this layer
     */
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

    /*
     * Boundary world coordinates
     *
     * Shift bounding box start by half of a costmap cell to represent the center of the cell, instead of the
     * lower-left corner, to calculate the cost. (bx, by) are in units of meters.
     */
    double bx = ox + res / 2,
           by = oy + res / 2;
    for (int i = start_x; i < end_x; i++)
    {
      for (int j = start_y; j < end_y; j++)
      {
        // save `old cost` i.e. from other layers to find maximum one at last
        unsigned char old_cost = costmap->getCost(i + dx, j + dy);
        if (old_cost == costmap_2d::NO_INFORMATION)
          continue;

        // map coordinates we're calculating for
        double x = bx + i * res, y = by + j * res;
        // direction of the vector pointing to the center of the person
        double ma = atan2(y - cy, x - cx);
        // difference between the `map angle` and the person's yaw
        double diff = angles::shortest_angular_distance(angle, ma);
        // amplitude of the Gaussian function
        double a;
        // separate angle range that the Gaussian is calculated for as we're considering asymmetric Gaussian (prolonged
        // in the person's heading direction); the other part is not affected by the orientation
        if (fabs(diff) < M_PI / 2)
          a = gaussian(x, y, cx, cy, amplitude_, covar_ * factor, covar_, angle);
        else
          a = gaussian(x, y, cx, cy, amplitude_, covar_,       covar_, 0);

        if (a < cutoff_)
          continue;
        unsigned char cvalue = (unsigned char) a;
        // NOTE how cell of the map is identified
        costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
      }
    }
  }
}

void ProxemicLayer::configure(ProxemicLayerConfig &config, uint32_t level)
{
  cutoff_ = config.cutoff;
  amplitude_ = config.amplitude;
  covar_ = config.covariance;
  factor_ = config.factor;
  people_keep_time_ = ros::Duration(config.keep_time);
  enabled_ = config.enabled;
}
};  // namespace social_navigation_layers
