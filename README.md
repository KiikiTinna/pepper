# Pepper ROS Workspace

This workspace contains ROS packages for operating and interacting with the Pepper robot.

## Setup
1. Clone this repository:
   ```bash
   git clone <repository-url> ~/catkin_ws/src
   ```
2. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Usage
### Teleoperation
Run the following launch file to control Pepper using a keyboard:
```bash
roslaunch teleop_pepper teleop_pepper.launch
```

### Mapping
To create a map with Pepper:
1. Launch the SLAM system:
   ```bash
   roslaunch pepper_mapping create_map.launch
   ```
2. Save the map:
   ```bash
   rosrun map_server map_saver -f ~/path/to/maps_file/map_name
   ```

### Localization
To localize Pepper in a pre-existing map:
```bash
roslaunch pepper_localization localization_with_map.launch
```

## Dependencies
- ROS Kinetic
- `pepper_bringup`
- `gmapping`
- `map_server`
- `amcl`
- `teleop_twist_keyboard`

## File Structure
- `src/`: Contains all ROS packages.
- `maps/`: Directory for saving maps.




