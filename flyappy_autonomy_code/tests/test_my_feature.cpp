#include <gtest/gtest.h>

#include "flyappy_autonomy_code/flyappy.hpp"

TEST(MyFeature, TestGetAcceleration)
{
    Flyappy flyappy;
    // target_velocity is (2, 0)

    auto acc = flyappy.getAcceleration(2, 0);
    EXPECT_EQ(acc.x, 0);
    EXPECT_EQ(acc.y, 0);

    acc = flyappy.getAcceleration(1, -1);
    EXPECT_EQ(acc.x, 3);
    EXPECT_EQ(acc.y, 30);

    acc = flyappy.getAcceleration(0, -2);
    EXPECT_EQ(acc.x, 3);
    EXPECT_EQ(acc.y, 35);
}

TEST(MyFeature, TestCollisionDetection)
{
    Flyappy flyappy;

    std::vector<float> ranges(1, 3.0);
    float angle_min = 0;
    float angle_increment = 0;
    auto res = flyappy.collisionDetection(ranges, angle_min, angle_increment);
    EXPECT_EQ(res, Flyappy::None);

    ranges = {1};
    res = flyappy.collisionDetection(ranges, angle_min, angle_increment);
    EXPECT_EQ(res, Flyappy::Front);

    angle_min = -1.2;
    res = flyappy.collisionDetection(ranges, angle_min, angle_increment);
    EXPECT_EQ(res, Flyappy::Floor);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
