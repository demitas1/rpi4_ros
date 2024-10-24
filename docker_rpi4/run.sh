#ros_image="ros2_humble:latest"
#ros_container="ros2_humble_container"
ros_image="ros2_jazzy:latest"
ros_container="ros2_jazzy_container"

docker run -d -it --rm \
    --net=host \
    --shm-size=1gb \
    -v /dev/shm:/dev/shm \
    -e ROS_DOMAIN_ID=42 \
    --name $ros_container $ros_image
