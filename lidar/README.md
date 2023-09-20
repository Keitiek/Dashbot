We are using Innomaker LiDAR_LD19 for our project. 

## Specs
Dimensions: 38*38*34.3mm
Detection range: 0.02-12m
Angular resolution: 0.2°-1.0°
The laser wavelength: 905nm
Measurement frequency: 4500Hz
Sweep frequency: 5-13Hz
Protection grade: IPX-4
Measuring Angle: 0°-360°

## Links
- Manufacturer: https://www.ldrobot.com
- User guide: https://www.inno-maker.com
- Product page: https://www.inno-maker.com/product/lidar-ld06/
- Datasheet: https://www.inno-maker.com/wp-content/uploads/2020/11/LDROBOT_LD06_Datasheet.pdf
- LDRobot official github page: https://github.com/ldrobotSensorTeam/ 

## Terms and abbreviations

- LiDAR: Light Detection and Ranging. A remote sensing method that uses light in the form of a pulsed laser to measure ranges (variable distances) to the Earth.
- PWM: Pulse Width Modulation. PWM refers to controlling the optical output of a laser via manipulation of the source that drives the gain medium.

## Communication and interface
- ZH1.5T-4P 1.5mm standard socket has to be connected with external system for powersupply, rotation control and data output.

## Installation and running
- First, follow https://www.youtube.com/watch?v=OJWAsV6-0GE to the point. When LD06 is picked in the video, choose LD19 instead.
- Source setup.bash
- Then, run this command to see sensor data: ros2 launch ldlidar_stl_ros2 viewer_ld19.launch.py
- To see messages published in the topic, run this command: ros2 topic echo /scan
