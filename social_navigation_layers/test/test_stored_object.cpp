#include <gtest/gtest.h>

#include <social_navigation_layers/stored_object.h>
#include <people_msgs_utils/group.h>

using namespace social_navigation_layers;

people_msgs_utils::Group createEntity(const std::string& id) {
  unsigned long int age = 60;
  geometry_msgs::Point center_of_gravity;

  return people_msgs_utils::Group(
    id,
    age,
    std::vector<people_msgs_utils::Person>(), // members
    std::vector<std::string>(), // member_ids
    std::vector<std::tuple<std::string, std::string, double>>(), // relations
    center_of_gravity
  );
}

// Test cases
TEST(StoredObjectTest, init) {
  auto buds = createEntity("buds");

  StoredObject<people_msgs_utils::Group> obj(buds, 0.5, "buds");

  ASSERT_DOUBLE_EQ(obj.getStoredObjectTimestamp(), 0.5);
  ASSERT_EQ(obj.getStoredObjectIdentifier(), "buds");
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 0.0);
  ASSERT_FALSE(obj.didStoredObjectGotOutdated());
  ASSERT_FALSE(obj.isStoredObjectOutdated());
}

TEST(StoredObjectTest, detectionAge) {
  auto colleagues = createEntity("colleagues");

  StoredObject<people_msgs_utils::Group> obj(colleagues, 1.0, "colleagues");
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 0.0);
  ASSERT_EQ(obj.getStoredObjectTimestamp(), 1.0);

  obj.updateStoredObjectAge(2.0, 5.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 1.0);
  ASSERT_EQ(obj.getStoredObjectTimestamp(), 1.0);

  obj.updateStoredObjectAge(4.0, 5.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 3.0);
  ASSERT_EQ(obj.getStoredObjectTimestamp(), 1.0);

  obj.updateStoredObjectAge(8.0, 5.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 7.0);
  ASSERT_EQ(obj.getStoredObjectTimestamp(), 1.0);
}

TEST(StoredObjectTest, erasePrediction) {
  auto workers = createEntity("workers");
  StoredObject<people_msgs_utils::Group> obj(workers, 10.0, "workers");
  ASSERT_FALSE(obj.isStoredObjectPersistingDetection());

  obj.updateStoredObjectAge(11.0, 5.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 1.0);
  ASSERT_TRUE(obj.isStoredObjectPersistingDetection());
  ASSERT_FALSE(obj.didStoredObjectGotOutdated());
  ASSERT_FALSE(obj.isStoredObjectOutdated());

  obj.updateStoredObjectAge(12.0, 5.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 2.0);
  ASSERT_TRUE(obj.isStoredObjectPersistingDetection());
  ASSERT_FALSE(obj.didStoredObjectGotOutdated());
  ASSERT_FALSE(obj.isStoredObjectOutdated());

  obj.updateStoredObjectAge(13.1, 5.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 3.1);
  ASSERT_TRUE(obj.isStoredObjectPersistingDetection());
  ASSERT_FALSE(obj.didStoredObjectGotOutdated());
  ASSERT_FALSE(obj.isStoredObjectOutdated());

  obj.updateStoredObjectAge(14.1, 5.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 4.1);
  ASSERT_TRUE(obj.isStoredObjectPersistingDetection());
  ASSERT_FALSE(obj.didStoredObjectGotOutdated());
  ASSERT_FALSE(obj.isStoredObjectOutdated());

  obj.updateStoredObjectAge(15.1, 5.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 5.1);
  ASSERT_TRUE(obj.isStoredObjectPersistingDetection());
  ASSERT_TRUE(obj.didStoredObjectGotOutdated());
  ASSERT_TRUE(obj.isStoredObjectOutdated());

  obj.updateStoredObjectAge(16.0, 5.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 6.0);
  ASSERT_TRUE(obj.isStoredObjectPersistingDetection());
  ASSERT_FALSE(obj.didStoredObjectGotOutdated());
  ASSERT_TRUE(obj.isStoredObjectOutdated());

  obj.updateStoredObjectAge(17.0, 5.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 7.0);
  ASSERT_TRUE(obj.isStoredObjectPersistingDetection());
  ASSERT_FALSE(obj.didStoredObjectGotOutdated());
  ASSERT_TRUE(obj.isStoredObjectOutdated());
}

TEST(StoredObjectTest, lifecycle) {
  auto workers = createEntity("workers");
  StoredObject<people_msgs_utils::Group> obj(workers, 10.0, "workers");

  obj.updateStoredObjectAge(25.0, 20.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 15.0);
  ASSERT_FALSE(obj.didStoredObjectGotOutdated());
  ASSERT_FALSE(obj.isStoredObjectOutdated());

  obj.updateStoredObjectAge(35.0, 20.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 25.0);
  ASSERT_TRUE(obj.didStoredObjectGotOutdated());
  ASSERT_TRUE(obj.isStoredObjectOutdated());

  obj = StoredObject<people_msgs_utils::Group>(workers, 45.0, "workers");

  obj.updateStoredObjectAge(50.0, 20.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 5.0);
  ASSERT_FALSE(obj.didStoredObjectGotOutdated());
  ASSERT_FALSE(obj.isStoredObjectOutdated());

  obj.updateStoredObjectAge(60.0, 20.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 15.0);
  ASSERT_FALSE(obj.didStoredObjectGotOutdated());
  ASSERT_FALSE(obj.isStoredObjectOutdated());

  // change keep time
  obj.updateStoredObjectAge(65.0, 30.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 20.0);
  ASSERT_FALSE(obj.didStoredObjectGotOutdated());
  ASSERT_FALSE(obj.isStoredObjectOutdated());

  obj.updateStoredObjectAge(71.0, 30.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 26.0);
  ASSERT_FALSE(obj.didStoredObjectGotOutdated());
  ASSERT_FALSE(obj.isStoredObjectOutdated());

  obj.updateStoredObjectAge(76.0, 30.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 31.0);
  ASSERT_TRUE(obj.didStoredObjectGotOutdated());
  ASSERT_TRUE(obj.isStoredObjectOutdated());

  obj.updateStoredObjectAge(80.0, 30.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 35.0);
  ASSERT_FALSE(obj.didStoredObjectGotOutdated());
  ASSERT_TRUE(obj.isStoredObjectOutdated());
}

TEST(StoredObjectTest, ageReliability) {
  auto workers = createEntity("workers");
  StoredObject<people_msgs_utils::Group> obj(workers, 10.0, "workers");

  obj.updateStoredObjectAge(11.0, 10.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 1.0);
  ASSERT_NEAR(obj.getStoredObjectAgeReliability(), 0.9, 1e-06);

  obj.updateStoredObjectAge(13.0, 10.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 3.0);
  ASSERT_NEAR(obj.getStoredObjectAgeReliability(), 0.7, 1e-06);

  obj.updateStoredObjectAge(15.0, 10.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 5.0);
  ASSERT_NEAR(obj.getStoredObjectAgeReliability(), 0.5, 1e-06);

  obj.updateStoredObjectAge(17.0, 10.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 7.0);
  ASSERT_NEAR(obj.getStoredObjectAgeReliability(), 0.3, 1e-06);

  obj.updateStoredObjectAge(20.0, 10.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 10.0);
  ASSERT_NEAR(obj.getStoredObjectAgeReliability(), 0.0, 1e-06);

  obj.updateStoredObjectAge(23.0, 10.0);
  ASSERT_DOUBLE_EQ(obj.getStoredObjectAge(), 13.0);
  ASSERT_NEAR(obj.getStoredObjectAgeReliability(), 0.0, 1e-06);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
