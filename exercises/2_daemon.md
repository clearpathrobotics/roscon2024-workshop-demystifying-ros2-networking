# 2. ROS 2 Daemon
In this section, we will cover the ROS 2 command line utilities and the underlying daemon. Whenever a CLI command, such as `ros2 topic list`, is called, all ROS 2 nodes need to be discovered. Depending on the number of nodes and the discovery process, this process can take a few seconds and the command may be have to ran more than once to display the complete list of topics.

The daemon runs in the background after the first command line tool is ran. It is continously discovering and stores a list of all nodes in the network. Whenever subsequent CLI calls are made, the daemon is used to promptly retrive node and topic information.


## Environment Variables and the Daemon
Environment variables are used to specify the ROS 2 discovery process. Whenever, the ROS 2 daemon is started, it is started with the same environment as the process that created it. In most cases, that process is the terminal.

List all environment variables of the current terminal:
```
printenv
```

To narrow down the output, we can use `grep` to display only ROS related variables:
```
printenv | grep ROS
```

If the output of the previous command is empty, it indicates the ROS environment has not been sourced. Do so now:
```
source /opt/ros/$ROS_DISTRO/setup.bash
```

You will need to source the ROS environment every time a new terminal window is opened. If you have not already, add the call to source the environment into the `~/.bashrc` file so that it is done automatically upon opening a terminal.

### Demo Talker
Using the demo talker node, we will demonstrate how to use the command line tools and what to keep in mind when using them.

<table>
<tr>
<td> <b>Terminal 1: Talker Node</b> </td>
<td> <b>Terminal 2: Command Line Tools</b> </td>
</tr>
<tr>
<td>
Change the &ltunique_namespace&gt to a unique namespace, such as your name or initials. Make sure to keep the leading forward slash.

```
ros2 run demo_nodes_cpp talker --ros-args -r __ns:=/<unique_namespace>
```

Now, you have a namespaced node publishing a simple `Hello World!` message.

</td>

<td>
Once the talker is running, we can run the command line tools to see the topics and messages.

```
ros2 topic list
```

This command will result in all topics from all nodes. You should see everybody's chatter topics with their own unique namespace.

You can then ensure that the messages are being delivered using the following command, where you replace &lttopic&gt with the full name of the topic you want to echo:
```
ros2 topic echo <topic>
```

You can also check the rate at which the messages are being delivered using the `hz` command:
```
ros2 topic hz <topic>
```

</td>
</tr>

<td>
Keep the demo talker node running and follow the instructions on the second terminal.
</td>

<td>
If we wanted to avoid other devices' nodes, we can use the `ROS_LOCALHOST_ONLY` environment variable to limit communication to only nodes on the same device.

Set the environment variable:

```
export ROS_LOCALHOST_ONLY=1
```

Now, try listing the topics once again.

```
ros2 topic list
```

You will notice that the environment variable did not change the discovery of the nodes and topics.

You must restart the daemon for the environment variable to be set in the daemon's environment:

```
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

Now, you will notice that there are no topics showing up now, including the topic from the node running on your device.
</td>

<tr>
<td>
For the daemon to reliably communicate with other nodes, they need to have compatible environments. Therefore, stop the talker node, set the localhost variable, and restart the talker:

```
export ROS_LOCALHOST_ONLY=1
ros2 run demo_nodes_cpp talker --ros-args -r __ns:=/unique_namespace
```

</td>
<td>
Now that we've restarted the talker with the same environment, the talker's topic should be listed by the command line tool:

```
ros2 topic list
```

</td>
</tr>
</table>

