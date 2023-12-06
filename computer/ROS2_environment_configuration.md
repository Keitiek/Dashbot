# ROS2 Environment Configuration

ROS2 environment configuration needs to be done to isolate several ROS environments in the same physical network and to launch ROS2 programs in different environments.

In each new environment, domain ID and ROS master uri need to be set. 
ROS domain id is a mechanism in ROS2 that allows you to isolate two or more ROS environments in the same physical network (arbitrarily selected integer with value ranging from 0 to 232 - 1 (0 to 4294967295).
ROS master uri is a required setting that tells nodes where they can locate the master (local ip of the master). Port 11311 is the default port for ROS which is used to communicate with the ROS master.

### Setting ROS master uri
After deciding which device is going to be the master, find out what it's local IP address is. Then, export the values on master device and all other devices of the ROS environment.

```sh
export ROS_MASTER_URI=http://10.0.1.123::11311
```
```sh
export ROS_DOMAIN_ID=42
```

To see what are the set values, use these commands:
```sh
echo $ROS_MASTER_URI
```
```sh
echo $ROS_DOMAIN_ID
```
