### How to launch control for the bot

## Build the project
Go to your ROS2 workspace and run :

```
colcon build --packages-select bot_control && . install/setup.bash
```

The package should build correctly.

## Launch the brain node 
Run : 
```
ros2 run bot_control brain.py 
```
It should display that the brain node is launched.
You can launch RQT in another terminal window to check the different topics.

## Launch everything
Run :
```
ros2 launch bot_control control.launch.py
```

Should launch all the necessary nodes for the bot control.
(Deprecated for now, only brain node)