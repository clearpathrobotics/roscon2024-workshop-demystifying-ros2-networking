# 5 Turtlebots

Each group will have a Turtlebot. Each robot is numbered. The number specifies the IP address and ID of the discover server running on the robot.

The IP address of the robot's server will be: `192.168.0.IP` where `IP = 20 + robot_number`.

The ID matches one to one with the ID of the server.

<table>
<tr><td><b>Terminal 1: Teleop Turtlebot</b></td></tr>
<tr><td>

Set the robot server in the discovery server variable:
```
export ROS_DISCOVERY_SERVER=”;192.168.0.<IP>:11811”
```
Note, you need to add N number of `;` to set the correct the ID.

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

## Bandwidth
<table>
<tr><td><b>Terminal 1: Camera Test</b></td></tr>
<tr><td>

Try to retrieve the camera topic:
```
ros2 topic hz /<tb_ns>/oakd/rgb/preview/image_raw
```

If you're lucky you might get some packets through. It is likely that no data will reach your device.

</td></tr>
</table>
