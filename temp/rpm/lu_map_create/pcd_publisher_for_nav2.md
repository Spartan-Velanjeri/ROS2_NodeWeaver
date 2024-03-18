# Howto lu_map_create/pcd_publisher_for_nav2.py

## Usage

1. Build ```lu_map_create```

2. Download the PTS file (Le131 Scan with Leica TS60) <br>
   https://artifactory.boschdevcloud.com/artifactory/BautiroArtifacts-local/maps_Le131/28092022_154530-crop.pts
   

2. Run
    ```bash
    ros2 run lu_map_create pcd_publisher_for_nav2 /PATH/TO/28092022_154530-crop.pts
    ```

Parameters:

```python
87        self.frame_id_scan = 'map'              # frame ID
88        self.resolution = 0.10                  # resolution of the occupancy grid in meters
89        self.height_range = np.array((0.4,6.0)) # height range to be considered in meters
```

Pulished topics:
- /scan           (sensor_msgs.PointCloud2)
- /occupancygrid  (nav_msgs.OccupancyGrid)
