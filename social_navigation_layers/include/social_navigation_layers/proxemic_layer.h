// Copyright 2018 David V. Lu!!
#ifndef SOCIAL_NAVIGATION_LAYERS_PROXEMIC_LAYER_H
#define SOCIAL_NAVIGATION_LAYERS_PROXEMIC_LAYER_H
#include <ros/ros.h>
#include <social_navigation_layers/social_layer.h>
#include <dynamic_reconfigure/server.h>
#include <social_navigation_layers/ProxemicLayerConfig.h>
#include <social_navigation_layers/detections_storage.h>

#include <utility>

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
double get_radius(double cutoff, double A, double var);

namespace social_navigation_layers
{
class ProxemicLayer : public SocialLayer
{
public:
  ProxemicLayer()
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

  /**
   * Virtual method called before @ref updateBoundsFromPeople in @ref updateBounds
   */
  virtual void preprocessForBounds();

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
  virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y);

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
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual double computeVarianceHeading(double speed) const;
  virtual double computeVarianceSide(double speed) const;
  virtual double computeVarianceRear(double speed) const;
  virtual std::pair<double, double> computeAdjustmentsRadius(double speed) const;

protected:
  virtual void configure(ProxemicLayerConfig &config, uint32_t level);
  double cutoff_, amplitude_;
  dynamic_reconfigure::Server<ProxemicLayerConfig>* server_;
  dynamic_reconfigure::Server<ProxemicLayerConfig>::CallbackType f_;

  DetectionsStorage<people_msgs_utils::Person> detections_people_;
};
}  // namespace social_navigation_layers

#endif  // SOCIAL_NAVIGATION_LAYERS_PROXEMIC_LAYER_H
