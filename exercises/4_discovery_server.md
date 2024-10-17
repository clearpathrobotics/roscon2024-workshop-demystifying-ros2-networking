# 4 Discovery Servers
In this section, we will cover how to setup discovery servers and set environment variables to use them.

> [!NOTE]
> Make sure to open new terminal windows with clean environments to avoid potential conflicting environment issues.

## Setup a Server
Create a discovery server on the local host so only nodes on the same device as the local host can discover each other.

<table>
<tr><td><b>Terminal 1: Localhost Server</b></td></tr>
<tr><td>

Start up a discovery server on the local host.
```
fastdds discovery -i 0 -l 127.0.0.1 -p 11811
```

The options above correspond to:
- `-i 0`: assign ID 0 to the server
- `-l 127.0.0.1`: the serer will listen on this address, localhost
- `-p 11811`: the server will listen on this port

</td></tr>
</table>

## Turtlesim on the Localhost Server
With this first server, let's test that we can discover just nodes on the same device and drive the turtle in the _Turtlesim_.

<table>
<tr>
<td><b>Terminal 2: Turtlesim Node</b></td>
<td><b>Terminal 3: Keyboard Teleop</b></td>
</tr>

<tr>
<td>
Set the discovery server environment variable to have the node use specifically the localhost server:

```
export ROS_DISCOVERY_SERVER=“127.0.0.1:11811”
```

Start the _Turtlesim_ node:
```
ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/<unique_namespace>
```

</td>
<td>
Set the discovery server environment variable:

```
export ROS_DISCOVERY_SERVER=“127.0.0.1:11811”
```

Unlike nodes, command line tools need to be set as super clients to be able to discover all nodes and topics, not just the topics it needs to subscribe to or advertises.
```
export ROS_SUPER_CLIENT=1
```

Restart the daemon for changes to take effect.
```
ros2 daemon stop; ros2 daemon start
```

Check that the <i>Turtlesim</i> node is discoverable.
```
ros2 node list
```

Run teleoperation
```
ros2 run turtlesim turtle_teleop_key --ros-args -r __ns:=/<unique_namespace>
```

</td>
</tr>
</table>

## Turtlesim on Multiple Servers
Now, let's setup a second server. This second server will be setup to allow a remote device to be able to connect to it.

<table>
<tr><td><b>Terminal 4: Wireless Interface Server</b</td></td>
<tr><td>
Find your wireless interface IP using:

```
ip a
```

Start the server:

```
fastdds discovery -i 1 -l <YOUR_WIFI_IP> -p 11812
```

Exchange IP addresses with a partner.
</td></tr>

</table>


<table>
<tr>
<td><b>Terminal 2: Turtlesim Node</b></td>
<td><b>Terminal 3: Keyboard Teleop</b></td>
</tr>

<tr>
<td>
Update the discover server environment variable:

```
export ROS_DISCOVERY_SERVER=“127.0.0.1:11811;<YOUR_WIFI_IP>:11812;”
```
Note, we want to setup the servers such that the <i>Turtlesim</i> node can communicate over localhost and the wireless interface with other nodes.

Then, restart the <i>Turtlesim</i>:
```
ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/<unique_namespace>
```

</td>
<td>
We want to be able to drive both the turtle in the local and in our partner's <i>Turtlesim</i> nodes. Therefore, we will set the discover server environment variable to point to the local server and the parter's server.

```
export ROS_DISCOVERY_SERVER=“127.0.0.1:11811;<PARTNER_IP>:11812;”
```

And, because we want to be able to discover topics, make sure to have set the super client variable:
```
export ROS_SUPER_CLIENT=1
```

Restart the daemon and check that both <i>Turtlesim</i> nodes are discoverable.
```
ros2 daemon stop; ros2 daemon start; ros2 node list
```

Start the teleoperatoin of your turtle:
```
ros2 run turtlesim turtle_teleop_key --ros-args -r __ns:=/<unique_namespace>
```

Start the teleoperation of your partner's turtle:
```
ros2 run turtlesim turtle_teleop_key --ros-args -r __ns:=/<partners_namespace>
```

</td>
</tr>
</table>

## Removing the Local Server
Now, we will stop the local server.

<table>
<tr><td><b>Terminal 1: Localhost Server</b></td></tr>
<tr><td>
Stop the server.
</td></tr>
</table>

<table>
<tr><td><b>Terminal 3: Keyboard Teleop</b></td></tr>
<tr><td>
Check the node list once again.

```
ros2 node list
```
Is your node discoverable? Is your partner's?

Can you drive your robot or your partner's?
```
ros2 run turtlesim turtle_teleop_key --ros-args -r __ns:=/<namespace>
```

</td></tr>
</table>

## One Server is Enough
Between you and your partner, pick one person that will continue to run the wireless interface server. The other person will stop theirs.

### Client Partner
If you're the client partner, then you need to stop your server, and restart your _Turtlesim_.
<table>
<tr><td><b>Terminal 4: Wireless Interface Server</b></td></tr>

<tr><td>
Stop the server.
</td></tr>
</table>

<table>
<tr><td><b>Terminal 2: Turtlesim Node</b></td></tr>
<tr><td>
Setup only your partner's server. Note, the leading semicolon is important; it delineates the ID of the servers.

```
export ROS_DISCOVERY_SERVER=“;<PARTNER_IP>:11812;”
```

Restart the _Turtlesim_:
```
ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/<unique_namespace>
```

</td></tr>
</table>

### Server Partner
If you're the server partner, then you need to update the discovery server environment variable to point to the wireless interface server instead of your partner's.

<table>
<tr><td><b>Terminal 3: Keyboard Teleop</b></td></tr>
<tr><td>
Reset the discovery server environment variables:

```
export ROS_DISCOVERY_SERVER=“;<YOUR_WIFI_IP>:11812;”
```

Restart the daemon and check that both your node and your partner's node are discoverable:
```
ros2 daemon stop; ros2 daemon start; ros2 node list
```

Start the teleoperation:
```
ros2 run turtlesim turtle_teleop_key --ros-args -r __ns:=/<namespace>
```

</td></tr>
</table>

Now, both you and your partner should be able to drive both turtles, while using just one discovery server.
