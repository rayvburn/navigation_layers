#pragma once

#include <vector>

namespace social_navigation_layers
{

/**
 * @brief Manages detections (disappearing over time) that are put onto costmaps
 *
 * Implements a logic that allows to clear the costmap area once detections are unavailable from perception,
 * prevents leaving a high cost area.
 *
 * It is intended to keep input buffer for 1 iteration longer after the timeout on the buffer's validity expires
 */
template <typename T>
class DetectionsStorage
{
public:
    /// Helper struct
    struct AreaBounds {
        AreaBounds(): min_x(0.0), min_y(0.0), max_x(0.0), max_y(0.0) {}

        double min_x;
        double min_y;
        double max_x;
        double max_y;
    };

    DetectionsStorage():
        buffer_input_empty_(true),
        input_buffer_become_empty_(false),
        time_start_keep_(0.0),
        duration_keep_(0.0),
        keep_time_elapsed_(false),
        counting_keep_time_(false)
    {}

    void setKeepTime(double time_sec)
    {
        duration_keep_ = time_sec;
    }

    void update(
        double timestamp,
        const std::vector<T>& transformed_buffer,
        double min_x,
        double min_y,
        double max_x,
        double max_y
    ) {
        bool new_buffer_empty = transformed_buffer.empty();

        // clear buffer when elapsed (pending @ref keep_time_elapsed_ flag from the previous @ref update)
        if (counting_keep_time_ && keep_time_elapsed_) {
            buffer_.clear();
            counting_keep_time_ = false;
        }

        // compare with the previous update to detect the event
        input_buffer_become_empty_ = new_buffer_empty && !buffer_input_empty_;

        // start the count when raw buffer has become empty
        if (input_buffer_become_empty_) {
            time_start_keep_ = timestamp;
            counting_keep_time_ = true;
        } else if (!new_buffer_empty) {
            // stop the count
            counting_keep_time_ = false;
            keep_time_elapsed_ = false;
        }

        // evaluate whether keep time has elapsed (since the input buffer has been empty), while storing sth
        // in the @ref buffer_; counting elapsed time to safely clear the @ref buffer_
        if (counting_keep_time_) {
            keep_time_elapsed_ = (timestamp - time_start_keep_) >= duration_keep_;
        }

        // save flag for later use
        buffer_input_empty_ = new_buffer_empty;

        // skip overwriting stored data when counting time
        if (counting_keep_time_) {
            return;
        }

        // copy the buffer (either non-empty or empty)
        buffer_ = transformed_buffer;
        // save valid area bounds related to the stored buffer
        bounds_.min_x = min_x;
        bounds_.min_y = min_y;
        bounds_.max_x = max_x;
        bounds_.max_y = max_y;
    }

    AreaBounds getArea() const {
        return bounds_;
    }

    std::vector<T> getBuffer() const {
        return buffer_;
    }

    /// Returns true when the **input** buffer was detected being cleared in the recent @ref update
    bool bufferHasBeenCleared() const {
        return input_buffer_become_empty_;
    }

    /// Returns true when the stored buffer is outdated - @ref duration_keep_ has elapsed since the input buffer
    // has been cleared out
    bool bufferKeepTimeElapsed() const {
        return keep_time_elapsed_;
    }

    /// Returns true when the timeout on the stored buffer expired and the buffer will be erased during the next
    /// @ref update call
    bool bufferWillBeErased() const {
        return counting_keep_time_ && keep_time_elapsed_;
    }

    /// Returns true when the stored buffer will be returned in @ref getBuffer
    /// in favour of the newest input buffer, which was empty
    bool usingPersistingBuffer() const {
        return input_buffer_become_empty_ || counting_keep_time_;
    }

protected:
    bool buffer_input_empty_;
    bool input_buffer_become_empty_;

    std::vector<T> buffer_;
    AreaBounds bounds_;

    /**
     * @defgroup timing Members used for counting time to keep the last buffer before erasing
     * @{
     */
    double time_start_keep_;
    double duration_keep_;

    bool keep_time_elapsed_;
    bool counting_keep_time_;
    /// @}
};

}
