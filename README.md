# Docker: ROS2 for Raspberry Pi

## install docker on Raspberry Pi

```
# download and install
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# add user to docker group
sudo vim /etc/group

# check
docker run hello-world
docker images
docker ps
```


## build

```
cd docker_rpi4
bash build.sh
```


## run

```
bash run.sh
docker exec -it ros2_jazzy_container bash

source ./install/setup.bash
ros2 run py_pubsub talker
```

## Tools

- To find your raspberry pi IP address

Use `./tools/find_raspberrypi_en.sh` to scan your network.

Select interface that is connected to the network where your Raspberry Pi is working.

Example to scan on WiFi:

```
$ bash ./tools/find_raspberrypi_en.sh
Available interfaces:
     1  br-cafe0123babe
     2  docker0
     3  eno1
     4  lo
     5  wlx0011deadbeef
Enter the number of the interface you want to use: 5
[sudo] password for yourname:
The IP address of Raspberry Pi is 192.168.1.161.
```

## License

MIT
