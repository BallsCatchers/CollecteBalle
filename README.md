# BallsCatchers AGATE
## (A)utonomous (G)atherer for (A)dvanced (T)ennis (E)xercise

![logo](/images/324945165_2118673848323594_5974956229308836638_n.gif)
## Lancer la simulation


### Git ref for demo controller

[gazebo_ros2_control_demos](https://github.com/ros-controls/gazebo_ros2_control/tree/master/gazebo_ros2_control_demos)
### Dépendences
Install ros2 foxy from website: [ROS](https://docs.ros.org/en/foxy/Installation.html)
Ros2 control library :
```
sudo apt install ros-foxy-ros2-control
sudo apt install ros-foxy-ros2-controllers
```

### Launch AGATE only

```
colcon build
. install/setup.bash
ros2 launch tennis_bot_description display.launch.py
```

### Launch AGATE on the field

```
colcon build
. install/setup.bash
ros2 launch tennis_bot_description spawner.launch.py
```

Control with the joystick :
```
ros2 launch tennis_bot_description spawner_joy.launch.py
```

### Launch image processing

```
colcon build
. install/setup.bash
ros2 run bot_control camera.py
```

## Groupe

### Membres

Thomas TACHERON

Jonas SOUEIDAN

Rémi POREE

Simon GERVAISE

### Gestion de projet

[Taiga](https://tree.taiga.io/project/thomastacheron-collecteballe/timeline)



## Structure du dépôt

Ce dépôt doit être cloné dans le dossier `src` d'un workspace ROS 2.

### Package `tennis_court`

Le dossier `tennis_court` est un package ROS contenant le monde dans lequel le robot ramasseur de balle devra évoluer ainsi qu'un script permettant de faire apparaître des balles dans la simulation.
Ce package ne doit pas être modifié.
Consulter le [README](tennis_court/README.md) du package pour plus d'informations.

### Package `tennis_bot_description`

Le dossier `tennis_bot_description` est un package ROS contenant le model du robot.

### Documents

Le dossier `docs` contient tous les documents utiles au projet:
- Des [instructions pour utiliser Git](docs/GitWorkflow_fork.md)
- Un [Mémo pour ROS 2 et Gazebo](docs/Memo_ROS2.pdf)
- Les [slides de la présentation Git](docs/GitPresentation.pdf)


### Rapports

Le dossier `reports` doit être rempli avec les rapports d'[objectifs](../reports/GoalsTemplate.md) et de [rétrospectives](../reports/DebriefTemplate.md) en suivant les deux templates mis à disposition. Ces deux rapports doivent être rédigés respectivement au début et à la fin de chaque sprint.
