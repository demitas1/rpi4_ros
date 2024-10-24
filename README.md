# Docker: ROS2 for Raspberry Pi

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

## License

MIT
