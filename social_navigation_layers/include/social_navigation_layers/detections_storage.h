#pragma once

#include <social_navigation_layers/stored_object.h>

#include <vector>

namespace social_navigation_layers
{

/**
 * @brief Manages detections (disappearing over time) that are put onto costmaps
 *
 * Implements a logic that allows to clear the costmap area (related to a detection) once relevant data regarding
 * a detection is for a long time unavailable from perception. Aim is to prevent leaving a high cost area on the
 * costmap.
 *
 * It is intended to keep a detection for 1 more iteration after the timeout of the detection's validity has expired
 *
 * @note This might be reimplemented using the std::map
 */
template <typename T>
class DetectionsStorage
{
public:
  DetectionsStorage():
    duration_keep_(0.0)
  {}

  void setParameters(double keep_time_sec, double min_reliability = 0.5) {
    duration_keep_ = keep_time_sec;
    min_reliability_ = min_reliability;
  }

  /*
   * NOTE: this might be changed to a method that takes a pointer to a T's function that allows to retrieve
   * an ID (more generic approach compared to getName; same with the getReliability method)
   */
  void update(double timestamp, const std::vector<T>& transformed_buffer) {
    // recompute how old is each detection
    for (auto& object: buffer_) {
      object.updateStoredObjectAge(timestamp, duration_keep_);
    }

    // start with erasing the detections that are too old;
    // this will be possibly done on a smaller buffer than after updating detections
    // Ref: https://stackoverflow.com/a/4713588
    buffer_.erase(
      std::remove_if(
        buffer_.begin(),
        buffer_.end(),
        [](StoredObject<T> obj) {
          // conditions of a deletion
          return obj.isStoredObjectOutdated() && !obj.didStoredObjectGotOutdated();
        }
      ),
      buffer_.end()
    );


    // iterate over all newly received detections
    for (const auto& object: transformed_buffer) {
      // NOTE: entities that can be used with this class template should have the getName() method implemented
      // which should return a string
      StoredObject<T> detection(object, timestamp, object.getName());

      bool detection_already_in_buffer = false;

      // iterate over buffered detections
      for (auto& object_buf: buffer_) {
        // compare IDs of the stored objects with an ID of a newly detected object
        if (object_buf.getStoredObjectIdentifier() != detection.getStoredObjectIdentifier()) {
          continue;
        }
        detection_already_in_buffer = true;

        // NOTE: update if ID matches and only if reliability is reasonable
        if (detection.getReliability() >= min_reliability_) {
          object_buf = detection;
        } /* else {
          // refresh the timestamp only
          object_buf = StoredObject<T>(object_buf, timestamp, object.getName());
        } */
        break;
      }

      // skip if a detection was found in the buffer
      if (detection_already_in_buffer) {
        continue;
      }

      // add a detection if it is not found in the buffer
      buffer_.push_back(detection);
    }
  }

  /**
   * @brief Returns a buffer with the most recent updates of stored detections
   * Persisting detections (that have not been observed for less than the @ref duration_keep_) will be included
   */
  std::vector<StoredObject<T>> getBuffer() const {
    return buffer_;
  }

protected:
  /// A vector of detections with an associated IDs and timestamps
  std::vector<StoredObject<T>> buffer_;

  /// For how much the detections will be stored since the newest data with the matching ID
  double duration_keep_;

  /// Reliability threshold below which the upcoming detections will not filled in the storage
  double min_reliability_;
};

}
