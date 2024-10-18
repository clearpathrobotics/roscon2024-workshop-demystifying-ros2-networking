# 1. Linux Networking
In this section, we will cover the basic linux commands used to debug networking issues.

## Ping
When you have access to the device you want to interface with and are able to determine it's IP, using the ping command is the simplest way to determine that the device is reachable using the current network.

In this case, you will ensure that the current network allows you to reach your neighbours computer. To do so, you must determine your IP address and provide it to your neighbour and vice-versa.

Check the status of the networking interfaces on your device and find your IP address with the following command.
```bash
ip a
```

Exchange IP addresses with a neighbour and make sure you are able to establish communication using the `ping` command.
```
ping <neighbours_ip>
```

## Nmap
In other cases, you may not have access to the computer you want to interface with, or are unaware of how many there even are connected to the same network. In such cases, we can use the `nmap` command to scan for all hosts on a specific subnet.

Find all IP addresses associated with a host or service:
```bash
nmap -sP 192.168.0.1/24
```

## Performance
Just because you are able to `ping` a device does not guarantee that you will be able to communicate reliably with it. Every network is limited by the hardware of the devices that make it up. With wireless interfaces, the environment and the location of the devices in the network must also be kept in consideration.

Regardless of the network, we can utitilize the `iperf3` server and client utility to determine the performance of the communication between two devices in that network.

Once again, pair up with a neighbour. One of you will run the server and the other will be the client.

The server must be started first. Start it using the following command:
```bash
iperf3 -s
```

Then, the client can be started:
```
iperf3 -c <servers_ip>
```


## Wireshark
