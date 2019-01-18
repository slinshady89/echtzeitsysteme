#include <gtest/gtest.h>
#include <ros/ros.h>

void foo1(int& i) {
    ROS_INFO("%d", i);
}
int& foo2() {
    int i = 5;
    return i;
}
TEST(FirstTestSuite, firstTestCase) {
    ASSERT_TRUE(true);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    foo1(foo2());
    return RUN_ALL_TESTS();
}