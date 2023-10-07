#pragma once

#include <string>

namespace social_navigation_layers
{

/**
 * Extension of a class that stores detections embedded onto the costmap
 *
 * Add methods indicating that an object will soon be erased from detections buffer
 */
template <typename T>
class StoredObject: public T {
public:
  StoredObject(const T& base, double timestamp, const std::string& id):
    T(base),
    so_timestamp_(timestamp),
    so_id_(id),
    so_object_age_(0.0),
    so_is_persisting_detection_(false),
    so_got_outdated_(false),
    so_outdated_(false)
  {
    // keep time (2nd arg) does not matter at that point - we are considering a new observation
    updateStoredObjectAge(timestamp, std::numeric_limits<double>::max());
  }

  /// Updates the internal @ref so_object_age_ according to the current timestamp given by @ref timestamp
  void updateStoredObjectAge(double timestamp_curr, double keep_time) {
    so_object_age_ = timestamp_curr - so_timestamp_;
    so_is_persisting_detection_ = timestamp_curr != so_timestamp_;

    // reliability decaying over time
    so_age_reliability_ = so_object_age_ / keep_time;
    so_age_reliability_ = std::max(std::min(1.0, so_age_reliability_), 0.0);
    so_age_reliability_ = 1.0 - so_age_reliability_;

    bool outdated_prev = so_outdated_;
    so_outdated_ = so_object_age_ > keep_time;

    so_got_outdated_ = so_outdated_ && outdated_prev != so_outdated_;
  }

  inline double getStoredObjectTimestamp() const {
    return so_timestamp_;
  }

  inline std::string getStoredObjectIdentifier() const {
    return so_id_;
  }

  inline double getStoredObjectAge() const {
    return so_object_age_;
  }

  /**
   * Indicates that a stored object has the age updated but the contents of itself weren't
   * i.e. wasn't detected since (at least) previous iteration
   */
  inline bool isStoredObjectPersistingDetection() const {
    return so_is_persisting_detection_;
  }

  /**
   * Returns true if an object has just got the outdated flag assigned
   * Can be used by the costmap layer to clear the area of an outdated detection
   * This will return true only once throughout the object lifecycle
   */
  inline bool didStoredObjectGotOutdated() const {
    return so_got_outdated_;
  }

  inline bool isStoredObjectOutdated() const {
    return so_outdated_;
  }

  /// Returns a reliability arising from the age of the detection
  inline double getStoredObjectAgeReliability() const {
    return so_age_reliability_;
  }

protected:
  double so_timestamp_;
  std::string so_id_;
  double so_object_age_;

  /// Flag indicating that the stored object was not updated since the last @ref updateStoredObjectAge call
  bool so_is_persisting_detection_;

  /// Flag indicating that the object will exceed the last given keep_time in the next iteration
  bool so_got_outdated_;

  /// Flag indicating that the object detection is outdated
  bool so_outdated_;

  // A reliability arising from the age of the detection
  double so_age_reliability_;
};

} // namespace social_navigation_layers
