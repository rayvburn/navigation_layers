#include <gtest/gtest.h>

#include <social_navigation_layers/detections_storage.h>

// example class that matches the DetectionsStorage template impl
#include <people_msgs_utils/person.h>

using namespace social_navigation_layers;

people_msgs_utils::Person createEntity(const std::string& id) {
  geometry_msgs::PoseWithCovariance pose;
  geometry_msgs::PoseWithCovariance velocity;
  double reliability = 1.0;
  bool occluded = false;
  bool matched = true;
  unsigned int detection_id = rand();
  unsigned long int track_age = rand();
  std::string group_name = std::to_string(rand());

  return people_msgs_utils::Person(
    id,
    pose,
    velocity,
    reliability,
    occluded,
    matched,
    detection_id,
    track_age,
    group_name
  );
}

// Test cases
TEST(DetectionsStorageTest, lifecycle) {
  auto john = createEntity("john");
  auto josh = createEntity("josh");
  auto frank = createEntity("frank");
  auto lisa = createEntity("lisa");
  auto linda = createEntity("linda");
  auto steve = createEntity("steve");

  DetectionsStorage<people_msgs_utils::Person> ds;
  ds.setParameters(5.0, 0.25);

  ds.update(0.5, {john, josh, frank});
  ASSERT_EQ(ds.getBuffer().size(), size_t(3));

  ds.update(2.5, {john});
  ASSERT_EQ(ds.getBuffer().size(), size_t(3));

  ds.update(4.5, {john});
  ASSERT_EQ(ds.getBuffer().size(), size_t(3));

  // objects stored additionally until the next "update"
  ds.update(6.5, {john});
  ASSERT_EQ(ds.getBuffer().size(), size_t(3));

  ds.update(8.5, {john});
  ASSERT_EQ(ds.getBuffer().size(), size_t(1));

  ds.update(10.5, {steve, lisa, linda, frank});
  ASSERT_EQ(ds.getBuffer().size(), size_t(5)); // plus john

  ds.update(12.5, {lisa, linda, frank});
  ASSERT_EQ(ds.getBuffer().size(), size_t(5));

  ds.update(14.5, {linda, frank});
  ASSERT_EQ(ds.getBuffer().size(), size_t(5));

  ds.update(16.5, {frank});
  ASSERT_EQ(ds.getBuffer().size(), size_t(4)); // john outdated

  ds.update(18.5, {});
  ASSERT_EQ(ds.getBuffer().size(), size_t(3)); // steve outdated

  ds.update(20.5, {});
  ASSERT_EQ(ds.getBuffer().size(), size_t(2)); // lisa outdated

  ds.update(22.5, {});
  ASSERT_EQ(ds.getBuffer().size(), size_t(1)); // linda outdated

  ds.update(24.5, {});
  ASSERT_EQ(ds.getBuffer().size(), size_t(0)); // frank outdated

  ds.update(26.5, {});
  ASSERT_EQ(ds.getBuffer().size(), size_t(0));
}


int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
