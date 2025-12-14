# Livox ROS Driver 2

Driver package for Livox LiDAR products with **ROS 2 Humble**.

> **Note:** For development and testing only. Optimize for production use.

## 1. Requirements

* Ubuntu 22.04
* ROS 2 Humble ([Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))
* Livox-SDK2

## 2. Installation

### 2.1 Build and Install Livox-SDK2

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2/
mkdir build && cd build
cmake .. && make -j
sudo make install
```

### 2.2 Create ROS2 Workspace and build Livox ROS2 Driver

```bash
mkdir -p ws_livox/src && cd ws_livox/src
git clone https://github.com/atarbabgei/livox_ros_driver2.git
cd ../..
source /opt/ros/humble/setup.bash
colcon build
```

## 3. Usage

### 3.1 Network Configuration

Configure your PC's network to communicate with the LiDAR **before** launching the driver.

**Default Settings (MID360_config.json):**

**PC Configuration:**
- IP Address: `192.168.1.5`
- Subnet Mask: `255.255.255.0`

**MID360 LiDAR:**
- IP Address: `192.168.1.3` (verify in Livox Viewer)

> **Important:** 
> 1. Use [Livox Viewer](https://www.livoxtech.com/downloads) to verify your LiDAR's actual IP address
> 2. Update `config/MID360_config.json` to match:
>    - `host_net_info.cmd_data_ip` → Your PC's IP (e.g., `192.168.1.5`)
>    - `lidar_configs[0].ip` → Your LiDAR's IP (e.g., `192.168.1.3`)

### 3.2 Launch the Driver

```bash
source ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 <launch_file>
```

**Example:**
```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

### 3.3 Available Launch Files

| Launch File | Description |
|-------------|-------------|
| `rviz_MID360_launch.py` | MID360 with PointCloud2 + RViz2 |
| `msg_MID360_launch.py` | MID360 with custom message |
| `rviz_HAP_launch.py` | HAP with PointCloud2 + RViz2 |
| `msg_HAP_launch.py` | HAP with custom message |
| `rviz_mixed_launch.py` | Mixed setup + RViz2 |

## 4. Configuration

### 4.1 Launch Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `publish_freq` | Publishing frequency (5.0-100.0 Hz) | 10.0 |
| `multi_topic` | `0`=Single topic, `1`=Per LiDAR | 0 |
| `xfer_format` | `0`=Livox PointCloud2, `1`=Custom, `2`=PCL | 0 |

### 4.2 Point Cloud Formats

**Livox PointCloud2 (PointXYZRTLT):** x, y, z, intensity, tag, line, timestamp  
**Livox Custom Message:** Header + point array with offset times  
**PCL PointCloud2 (pcl::PointXYZI):** Standard PCL format

## 5. Troubleshooting

### Network issues?

- Verify IP configuration: `ip addr show`
- Check connectivity: `ping 192.168.1.3`
- Disable firewall temporarily: `sudo ufw disable`

> **Note:** Ensure the LiDAR is configured with the correct IP (192.168.1.x subnet). The default is `192.168.1.3`. Verify your LiDAR's IP using Livox Viewer.

### No point cloud in RViz2?

- Set **Fixed Frame** to `livox_frame` in RViz2
- Check PointCloud2 topic is enabled

### SDK library not found?

Remove SDK first
```bash
sudo rm -rf /usr/local/lib/liblivox_lidar_sdk_*
sudo rm -rf /usr/local/include/livox_lidar_*
```

and reinstall the driver

## Supported LiDAR

* MID360
* HAP