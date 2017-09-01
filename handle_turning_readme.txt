
To run the scanning demo:

- Open laser_stitcher/data/postprocessing/handle_turning_demo.yaml
   - Modify the values in the vector 'box' vector [x_min, x_max, y_min, y_max, z_min, z_max]
   - These control the clipping box applied to the final output cloud
- Open laser_stitcher/data/lidar_ur5_manager/handle_turning_demo.yaml
   - Modify 'min_angle' and 'max_angle' variables to ensure that the range spanned by the scan is correct
   - Modify 'wrist_speed' based on resolution needs
- Run hokuyo_lidar
- roslaunch laser_stitcher handle_turning_demo.launch 
