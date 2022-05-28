# Time disparity analysis of message synchronizer in ROS 
Modeling ROS Approximate Time policy and CyberRT latest policy.

## 1 Before building

### 1.1 [Build ROS2 Dashing in `ros2_dashing` workspace](https://docs.ros.org/en/dashing/Installation/Ubuntu-Development-Setup.html)
### 1.2 Clone this project into `ros2_dashing/src`
```
git clone github.com/fighting8969/synchronizer
```
### 1.3 Add files

- Put `.h` files in `message_filters/sync_policies` of this project into `ros2_dashing/src/ros2/message_filters/include/message_filters/sync_policies`

- Put test files in `message_filters/test` of this project into `ros2_dashing/src/ros2/message_filters/test`

### 1.4 Modify CMakeLists.txt in `message_filters` package

Add following content into CMakeLists.txt:

```
  ament_add_gtest(${PROJECT_NAME}-test_latest_policy test/test_latest_policy.cpp)
  if(TARGET ${PROJECT_NAME}-test_latest_policy)
    target_link_libraries(${PROJECT_NAME}-test_latest_policy ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_approximate_time_model test/test_approximate_time_model.cpp)
  if(TARGET ${PROJECT_NAME}-test_approximate_time_model)
    target_link_libraries(${PROJECT_NAME}-test_approximate_time_model ${PROJECT_NAME})
  endif()
```
## 2 Build

### 2.1 Build `message_filters` package

 ```
cd ros2_dashing
colcon build --packages-select message_filters --symlink-install
source ./install/setup.bash
 ```
 
## 2.2 Build this project
```
cd ros2_dashing
source ./install/setup.bash
colcon build --packages-select synchronizer --symlink-install
```

## 3 Run
### 3.1 run nodes of this project
### 3.1.1 Synchronize sensor data from SVL simulator (Our model and original approximate time implementation)
Run the synchronizer node:
```
ros2 run synchronizer synchronizer_node
```

Replay the recorded sensor data (see folder `sensor_data`):
```
ros2 bag play data2_lidar1_cam2_imu100
```

### 3.1.2 Synchronize periodly generated data by ROS timer (Our model and original approximate time implementation)
```
ros2 run synchronizer verification_node
```

### 3.1.3 Synchronize periodly generated data by ROS timer (Our model and latest policy)
```
ros2 run synchronizer test_node
```

### 3.2 Run google test
```
colcon test --packages-select message_filters
```

or

```
cd ~/ros2_dashing/build/message_filters/test
ctest -V -R message_filters-test_approximate_time_model
```

## Acknowledgements
Our model implementation is based on the ROS Approximate Time policy implementation from https://github.com/ros2/message_filters.

## Notes
1. `subscriber_test_v1.h`: it is the first version for latest and approximate time model test. For latest policy, the master channel is the channel with the largest period.
2. `subscriber_test.h`: it is the second version for latest and approximate time model test. For latest policy, the master channel is a random channel.