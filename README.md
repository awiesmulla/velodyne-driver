# velodyne-driver
Velodyne VLP32 and VLP16 LiDAR driver compatible with ROS foxy  
The workspace contains two ROS foxy packages:
- Velodyne LiDAR driver (velodyne)
- Script for collecting Velodyne LiDAR data (lidar_collect)

## Velodyne driver

Be sure to check the velodyne LiDAR model before using the driver. Current driver is used for Velodyne VLP32 and VLP16 LiDAR.  
To run all the nodes for corresponding use the following

    ros2 launch velodyne velodyne-all-nodes-VLP32C-launch.py

Refer to respective folders for runnning specific nodes and for other LiDAR models from the package

## lidar_collect

Specify the destination path to store data in lidar_collect.py  
Run the following command to store the data
    
    ros2 run lidar_collect lidar_collect

The values obtained are multiplied by 100 and are saved as numpy int16 arrays due to space constraints  
So, divide the stored values by 100 before using
