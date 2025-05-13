# ROS2_Kitti_Publisher
A ros2 package that publishes kitti sequence and kitti raw datasets

Clone the repository to your src folder

```bash
git clone https://github.com/almos-a/ROS2_Kitti_Publisher.git
```
## Update dataset directory
Remember to update your Kitti sequence and raw dataset directory in the `kitti_publisher/kitti_publisher.py` and the `kitti_publisher/kitti_raw_publisher.py`

## Install
Install with `colcon build` and source using `source install/setup.bash`.

## Example Usage 
```bash
ros2 launch kitti_publisher kitti_playback.launch.py 
```

### Dataset
Download the dataset from https://www.cvlibs.net/datasets/kitti/index.php
