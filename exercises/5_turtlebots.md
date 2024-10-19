# 5 Turtlebots

Each group will have a Turtlebot. Each robot is numbered. The number specifies the IP address and ID of the discover server running on the robot.

The IP address of the robot's server will be: `192.168.0.IP` where `IP = 20 + robot_number`.

The ID matches one to one with the ID of the server.

> [!NOTE]
> Make sure to open new terminal windows with clean environments to avoid potential conflicting environment issues.

<table>
<tr><td><b>Terminal 1: Teleop Turtlebot</b></td></tr>
<tr><td>

Set the robot server in the discovery server variable:
```
export ROS_DISCOVERY_SERVER=”;192.168.0.<IP>:11811”
```

> [!NOTE]
> You need to add N number of `;` to set the correct the ID.

Examples:
```
export ROS_DISCOVERY_SERVER="192.168.0.20"            # Robot 00
export ROS_DISCOVERY_SERVER=";192.168.0.21"           # Robot 01
export ROS_DISCOVERY_SERVER=";;192.168.0.22"          # Robot 02
export ROS_DISCOVERY_SERVER=";;;192.168.0.23"         # Robot 03
export ROS_DISCOVERY_SERVER=";;;;192.168.0.24"        # Robot 04
export ROS_DISCOVERY_SERVER=";;;;;192.168.0.25"       # Robot 05
export ROS_DISCOVERY_SERVER=";;;;;;192.168.0.26"      # Robot 06
export ROS_DISCOVERY_SERVER=";;;;;;;192.168.0.27"     # Robot 07
export ROS_DISCOVERY_SERVER=";;;;;;;;192.168.0.28"    # Robot 08
export ROS_DISCOVERY_SERVER=";;;;;;;;;192.168.0.29"   # Robot 09
export ROS_DISCOVERY_SERVER=";;;;;;;;;;192.168.0.30"  # Robot 10
export ROS_DISCOVERY_SERVER=";;;;;;;;;;;192.168.0.31" # Robot 11
```


Then, set the terminal as super client:
```
export ROS_SUPER_CLIENT=1
```

Restart the daemon and ensure you are able to see all of the robot's nodes and topics:
```
ros2 daemon stop; ros2 daemon start;
ros2 node list
ros2 topic list
```

Now, you can drive the Turtlebot:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/<tb_ns>
```

</td></tr>
</table>

## Bandwidth (Optional)
<table>
<tr><td><b>Terminal 1: Camera Test</b></td></tr>
<tr><td>

Install the FFMPEG image transport plugin:
```
sudo apt install ros-humble-ffmpeg-image-transport
```

Try to retrieve the camera topic:
```
ros2 topic hz /<tb_ns>/oakd/rgb/preview/image_raw
```

If you're lucky you might get some packets through. It is likely that no data will reach your device.

Instead, try the FFMPEG topic which has encoded image data.
```
ros2 topic hz /<tb_ns>/oakd/rgb/preview/image_raw/ffmpeg
```

You won't be able to view the image data in RViz while it is encoded. Instead, decode it using the republisher node:
```
ros2 run image_transport republish ffmpeg in/ffmpeg:=/<tb_ns>/oakd/rgb/preview/image_raw/ffmpeg raw out:=/<unique_namespace>/oakd/decoded/image
```

Now, you should be able to get the image data as a standard image topic:
```
ros2 topic hz /<unique_namespace>/oakd/decoded/image
ros2 topic bw /<unique_namespace>/oakd/decoded/image
```

Then, open RViz and view the image:
```
rviz2
```

</td></tr>
</table>
