#ros_image="ros2_humble:latest"
#ros_container="ros2_humble_container"
ros_image="ros2_jazzy:latest"
ros_container="ros2_jazzy_container"
#ros_image="ros2_kilted:latest"
#ros_container="ros2_kilted_container"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker run -d -it --rm \
    --net=host \
    --shm-size=1gb \
    -v /dev/shm:/dev/shm \
    -v "$SCRIPT_DIR/ros2_ws:/home/ros2_user/ros2_ws" \
    -e ROS_DOMAIN_ID=42 \
    -w /home/ros2_user/ros2_ws \
    --name $ros_container $ros_image
