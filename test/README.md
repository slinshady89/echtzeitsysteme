# GTests Unit Tests

You can place any unit tests in this folder. See firstTest.cpp for an example usage.

## Test files

Include gtest with `#include <gtest/gtest.h>`.

The main method should contain the lines
```
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
```
so that the testing environment works correctly.

Add your test to the CMakeLists.txt and link it to ROS libraries if needed :
```
catkin_add_gtest(your_test test/your_test.cpp)
target_link_libraries(firstTest ${catkin_LIBRARIES})
```
