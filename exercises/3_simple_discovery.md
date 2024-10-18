# 3 Simple Discovery
In this section, we will cover the simple discovery protocol and the use of domain IDs.

> [!NOTE]
> Make sure to open new terminal windows with clean environments to avoid potential conflicting environment issues.

## Turtlesim on Domains

<table>
<tr>
<td> <b>Terminal 1: Turtlesim Node</b> </td>
<td> <b>Terminal 2: Command Line Tools</b> </td>
</tr>

<tr>
<td>

Start the demo _Turtlesim_ with a unique namespace:
```
ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/<unique_namespace>
```
Make sure to keep the forward slash at the beginning of the namespace argument.

</td>
<td>
Restart the daemon, to reload the clean environment, and make sure you can see the <i>Turtlesim</i> nodes:

```
ros2 daemon stop
ros2 daemon start
ros2 node list
```

</td>
</tr>

<tr>
<td>
Get into pairs or a small group. Each group will have its own domain. Set your domain ID and restart the <i>Turtlesim</i>:

```
export ROS_DOMAIN_ID=<DOMAINID>

ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/<unique_namespace>
```

</td>
<td>
Set the domain ID, restart the daemon, and make sure that only your team's nodes are on that domain.

```
export ROS_DOMAIN_ID=<DOMAINID>
ros2 daemon stop
ros2 daemon start
ros2 node list
```

</td>
</tr>

<tr>
<td> Keep the node running </td>
<td>
Now you can drive your robot and/or your partner's robot:

```
ros2 run turtlesim turtle_teleop_key --ros-args -r __ns:=/<unique_namespace>
```

</td>
</tr>
</table>
