#include <gtest/gtest.h>

#include <social_navigation_layers/detections_storage.h>

using namespace social_navigation_layers;

const auto BUF_EMPTY = std::vector<int>();
const auto BUF_NON_EMPTY = std::vector<int>{1, 2, 3, 4, 5};
const auto BUF_NON_EMPTY2 = std::vector<int>{5, 4, 3, 2, 1};
const double MIN_X = 0.0;
const double MIN_Y = 0.0;
const double MAX_X = 100.0;
const double MAX_Y = 100.0;

// Test cases
TEST(DetectionsStorageTest, init) {
    DetectionsStorage<int> d;

    ASSERT_EQ(d.getBuffer(), BUF_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());

    // does not affect
    d.setKeepTime(0.0);
    ASSERT_EQ(d.getBuffer(), BUF_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());
}

TEST(DetectionsStorageTest, emptyData) {
    DetectionsStorage<int> d;

    d.update(1.00, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());

    d.update(2.00, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());
}

TEST(DetectionsStorageTest, keepTimeZero) {
    DetectionsStorage<int> d;
    d.setKeepTime(0.0);

    d.update(0.50, BUF_NON_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());

    d.update(1.00, BUF_NON_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());

    d.update(1.50, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_TRUE(d.bufferHasBeenCleared());
    ASSERT_TRUE(d.bufferKeepTimeElapsed());
    ASSERT_TRUE(d.bufferWillBeErased());
    ASSERT_TRUE(d.usingPersistingBuffer());

    d.update(2.00, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_TRUE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());

    d.update(2.50, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_TRUE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());
}

TEST(DetectionsStorageTest, keepTimeNonZero) {
    DetectionsStorage<int> d;
    d.setKeepTime(5.0);

    d.update(0.10, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);

    d.update(0.50, BUF_NON_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());

    d.update(1.00, BUF_NON_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());

    // first empty data set received
    d.update(1.50, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_TRUE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_TRUE(d.usingPersistingBuffer());

    d.update(2.50, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_TRUE(d.usingPersistingBuffer());

    d.update(5.50, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_TRUE(d.usingPersistingBuffer());

    d.update(6.49, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_TRUE(d.usingPersistingBuffer());

    d.update(6.51, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_TRUE(d.bufferKeepTimeElapsed());
    ASSERT_TRUE(d.bufferWillBeErased());
    ASSERT_TRUE(d.usingPersistingBuffer());

    // immediately after first detection of elapsed time, the buffer becomes empty
    d.update(6.52, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_TRUE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());
}

TEST(DetectionsStorageTest, renewedDataReception) {
    DetectionsStorage<int> d;
    d.setKeepTime(5.0);

    d.update(6.00, BUF_NON_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());

    d.update(8.00, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_TRUE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_TRUE(d.usingPersistingBuffer());

    d.update(10.00, BUF_NON_EMPTY2, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY2);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());

    d.update(12.00, BUF_NON_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());

    d.update(14.00, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_TRUE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_TRUE(d.usingPersistingBuffer());

    d.update(16.00, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_TRUE(d.usingPersistingBuffer());

    d.update(20.00, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_TRUE(d.bufferKeepTimeElapsed());
    ASSERT_TRUE(d.bufferWillBeErased());
    ASSERT_TRUE(d.usingPersistingBuffer());

    d.update(22.00, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_TRUE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());

    d.update(24.00, BUF_NON_EMPTY2, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY2);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());

    d.update(25.00, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY2);
    ASSERT_TRUE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_TRUE(d.usingPersistingBuffer());

    d.update(31.00, BUF_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY2);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_TRUE(d.bufferKeepTimeElapsed());
    ASSERT_TRUE(d.bufferWillBeErased());
    ASSERT_TRUE(d.usingPersistingBuffer());

    d.update(33.00, BUF_NON_EMPTY, MIN_X, MIN_Y, MAX_X, MAX_Y);
    ASSERT_EQ(d.getBuffer(), BUF_NON_EMPTY);
    ASSERT_FALSE(d.bufferHasBeenCleared());
    ASSERT_FALSE(d.bufferKeepTimeElapsed());
    ASSERT_FALSE(d.bufferWillBeErased());
    ASSERT_FALSE(d.usingPersistingBuffer());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
