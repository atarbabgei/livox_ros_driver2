# Livox ROS Driver 2 (for ROS 2 Humble)

Livox ROS Driver 2 is a driver package used to connect LiDAR products produced by Livox. this repository is specifically for **ROS 2 Humble**.

**Note:**

As a debugging tool, Livox ROS Driver is not recommended for mass production but limited to test scenarios. You should optimize the code based on the original source to meet your various needs.

## 1\. Preparation

### 1.1 OS and ROS 2 Requirements

  * **OS:** Ubuntu 22.04
  * **ROS 2:** ROS 2 Humble Hawksbill

### 1.2 Install ROS 2 Humble

If you do not have ROS 2 Humble installed, please follow the official installation instructions. A **Desktop-Full** installation is recommended to have all necessary tools like RViz2.

  * **ROS 2 Humble Installation Instructions:** [docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

This package uses `colcon` as its build tool, which is included in the Desktop-Full installation of ROS 2.

## 2\. Build & Run Livox ROS Driver 2

### 2.1 Create a Workspace and Clone the Source Code

```shell
# Create a ROS 2 workspace, for example, 'ws_livox'
mkdir -p ws_livox/src

# Clone the driver source code into the 'src' directory
git clone https://github.com/rocketSzw/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
```

**Note:**

Be sure to clone the source code in a `[workspace_name]/src/` folder (as shown above), otherwise, `colcon` will not be able to find and build the package.

### 2.2 Build & Install the Livox-SDK2

The ROS 2 driver depends on the underlying Livox-SDK2. You must install it first.

**Please follow the installation guide in the [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md)**. Ensure you build and install it correctly before proceeding.

### 2.3 Build the Livox ROS Driver 2

Use the standard ROS 2 `colcon` build tool to compile the driver.

```shell
# Navigate to the root of your workspace
cd ws_livox

# Source the ROS 2 Humble environment
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build
```

### 2.4 Run Livox ROS Driver 2

After a successful build, you can run the driver using a launch file.

```shell
# First, source the setup file of your workspace in a new terminal
# Make sure you are in the root of your workspace (e.g., ws_livox)
source install/setup.bash

# Now, use ros2 launch to run the driver
ros2 launch livox_ros_driver2 [launch_file_name]
```

  * **[launch\_file\_name]**: is the Python launch file you want to use. The `launch` folder contains several samples for your reference.

An example to launch a HAP LiDAR and view it in RViz2 would be:

```shell
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

## 3\. Launch File and Parameter Configuration

### 3.1 Launch File Descriptions

The ROS 2 launch files are located in the `ws_livox/src/livox_ros_driver2/launch` directory. Different launch files are used for different scenarios:

| Launch File Name            | Description                                                  |
| --------------------------- | ------------------------------------------------------------ |
| `rviz_HAP_launch.py`        | Connects to a HAP LiDAR, publishes `PointCloud2` data, and auto-loads RViz2. |
| `msg_HAP_launch.py`         | Connects to a HAP LiDAR and publishes Livox's custom message data. |
| `rviz_MID360_launch.py`     | Connects to a MID360 LiDAR, publishes `PointCloud2` data, and auto-loads RViz2. |
| `msg_MID360_launch.py`      | Connects to a MID360 LiDAR and publishes Livox's custom message data. |
| `rviz_mixed_launch.py`      | Connects to HAP and MID360 LiDARs, publishes `PointCloud2` data, and auto-loads RViz2. |
| `msg_mixed_launch.py`       | Connects to HAP and MID360 LiDARs and publishes Livox's custom message data. |

### 3.2 Main Internal Parameters

All internal parameters are configured within the launch files. Below are descriptions of the three most common parameters:

| Parameter    | Detailed Description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| `publish_freq` | Sets the frequency of point cloud publishing (in Hz). \<br\>Floating-point data type. Recommended values: 5.0, 10.0, 20.0, 50.0. The maximum is 100.0 Hz. | 10.0    |
| `multi_topic`  | Determines if each LiDAR publishes to a separate topic.\<br\>0 -- All LiDARs use the same topic.\<br\>1 -- Each LiDAR has its own topic. | 0       |
| `xfer_format`  | Sets the point cloud format.\<br\>0 -- Livox `PointCloud2` (PointXYZRTLT) format.\<br\>1 -- Livox custom message format.\<br\>2 -- PCL standard `PointCloud2` (pcl::PointXYZI) format. | 0       |

**Note:** Other parameters not mentioned here should not be changed unless fully understood.

### 3.3 Point Cloud Data Formats

1.  **Livox PointCloud2 (PointXYZRTLT)**:

    ```c
    float32 x               # X axis, unit:m
    float32 y               # Y axis, unit:m
    float32 z               # Z axis, unit:m
    float32 intensity       # Reflectivity value, 0.0~255.0
    uint8   tag             # Livox tag
    uint8   line            # Laser number in lidar
    float64 timestamp       # Timestamp of point
    ```

2.  **Livox Custom Message**:

    ```c
    std_msgs/Header header     # ROS standard message header
    uint64          timebase   # Timestamp of the first point in the packet
    uint32          point_num  # Total number of points
    uint8           lidar_id   # Lidar device ID
    uint8[3]        rsvd       # Reserved
    CustomPoint[]   points     # Point cloud data
    ```

      * **CustomPoint Format**:
        ```c
        uint32  offset_time     # Offset time relative to the base time
        float32 x, y, z         # Coordinates (m)
        uint8   reflectivity    # 0~255
        uint8   tag             # Livox tag
        uint8   line            # Laser number
        ```

3.  **PCL Standard PointCloud2 (pcl::PointXYZI)**:
    Please refer to the `pcl::PointXYZI` data structure in the PCL library (`point_types.hpp`).

## 4\. LiDAR Configuration

LiDAR settings (IP address, port, data type, etc.) are set via a JSON-style config file. Sample files are in the `config` folder. The `user_config_path` parameter in the launch files points to the desired JSON file.

A configuration example for a HAP LiDAR (`config/HAP_config.json`) is shown below:

```json
{
  "lidar_summary_info" : { "lidar_type": 8 },
  "HAP": {
    "device_type" : "HAP",
    "lidar_ipaddr": "",
    "lidar_net_info" : { "cmd_data_port": 56000, "point_data_port": 57000, "imu_data_port": 58000, ... },
    "host_net_info" : { "cmd_data_ip" : "192.168.1.5", "point_data_ip": "192.168.1.5", ... }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : { "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "x": 0, "y": 0, "z": 0 }
    }
  ]
}
```

*(For brevity, some examples have been omitted. Refer to the original [repository](https://github.com/Livox-SDK/livox_ros_driver2?tab=readme-ov-file) for complete examples.)*


## 5\. Supported LiDAR List

  * HAP
  * Mid360
  * *(more types are coming soon...)*

## 6\. FAQ

### 6.1 RViz2 launches but no point cloud is displayed?

In the RViz2 "Displays" panel on the left, check the "Global Options". Set the **Fixed Frame** to `livox_frame`. Also, ensure the checkbox next to the "PointCloud2" topic display is checked.

### 6.2 The launch fails with "cannot open shared object file liblivox\_sdk\_shared.so"?

This means the system cannot find the installed Livox-SDK2 library. The recommended solution is to update the dynamic linker cache.

```shell
# This command updates the cache to include standard directories like /usr/local/lib
sudo ldconfig
```

After running this command, open a new terminal and try launching again. If the problem persists, you can manually add the library path to your environment:

  * For the current terminal only:
    ```shell
    export LD_LIBRARY_PATH=/path/to/your/livox_sdk_lib:$LD_LIBRARY_PATH
    ```
  * To make it permanent for your user:
    ```shell
    # Add the path to your .bashrc file
    echo 'export LD_LIBRARY_PATH=/path/to/your/livox_sdk_lib:$LD_LIBRARY_PATH' >> ~/.bashrc

    # Source the file to apply changes to the current terminal
    source ~/.bashrc
    ```