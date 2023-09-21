# Innomaker LiDAR_LD19
We are using Innomaker LiDAR_LD19 for our project. 

## Specifications
- **Dimensions:** 38 x 38 x 34.3mm
- **Detection Range:** 0.02 - 12m
- **Angular Resolution:** 0.2° - 1.0°
- **Laser Wavelength:** 905nm
- **Measurement Frequency:** 4500Hz
- **Sweep Frequency:** 5 - 13Hz
- **Protection Grade:** IPX-4
- **Measuring Angle:** 0° - 360°

## Useful Links
- **Manufacturer:** [LDRobot](https://www.ldrobot.com)
- **User Guide:** [Inno-Maker](https://www.inno-maker.com)
- **Product Page:** [LiDAR_LD06 Official Page](https://www.inno-maker.com/product/lidar-ld06/), [LiDAR LD19 product page on elecrow.com](https://www.elecrow.com/download/product/SLD06360F/LD19_Development%20Manual_V2.3.pdf)
- **Datasheet:** [Download Datasheet](https://www.inno-maker.com/wp-content/uploads/2020/11/LDROBOT_LD06_Datasheet.pdf)
- **LDRobot Official GitHub Page:** [GitHub Repository](https://github.com/ldrobotSensorTeam/)

## Terms and Abbreviations

- **LiDAR:** Short for "Light Detection and Ranging," LiDAR is a remote sensing method that employs pulsed laser light to measure ranges or distances to objects on the Earth's surface.
- **PWM:** Abbreviation for "Pulse Width Modulation," PWM is a technique used for controlling the optical output of a laser by manipulating the source that drives the gain medium.

## Communication and Interface
- To interface with the LiDAR_LD19, connect the ZH1.5T-4P 1.5mm standard socket to an external system. This connection is required for power supply, rotation control, and data output.

## Installation and Running
To set up and run the LiDAR_LD19, follow these steps:
- Follow the instructional video [Innomaker LiDAR_LD19 Setup Video](https://www.youtube.com/watch?v=OJWAsV6-0GE). When selecting the text file for LiDAR model in the video, choose LD19 instead of LD06
2. Execute the following commands in your terminal:
    - Set up the environment:
        ```
        source setup.bash
        ```

    - Launch the sensor data viewer:
        ```
        ros2 launch ldlidar_stl_ros2 viewer_ld19.launch.py
        ```

    - To monitor the messages published on the topic, use the following command:
        ```
        ros2 topic echo /scan
        ```
    - To launch RViz and vizualize ROS topic messages, execute the following command:
        ```
        rviz2
        ```     
        You might need to setup RViz settings, in that case just go back to the instructional video above.

## tty Permission Error

In case you see a tty0 permission error, it means that your OS is asking you to permit the port to be opened. You can do it manually by running chmod command from the instructional video above, or, if you want to give your user the permission permanently, add the user to tty and dialout groups.

You can check what groups the user is in, using this command:
```
groups
```
If `tty` and `dialout` are not in the list, then the user is not a part of these groups, and to add them, use this command:

```
sudo usermod -a -G tty [yourusername]
sudo usermod -a -G dialout [yourusername]
```

For the changes to take effect, log out and in.
