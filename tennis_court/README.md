# Tennis Court

Package ROS 2 pour la simulation d'un court de tennis dans Gazebo.

## Fonctionnalités

Ce package contient:
- Un monde Gazebo [`court.world`](worlds/court.world) composé d'un court de tennis et d'une caméra zénithale
- Un script [`ball_manager.py`](scripts/ball_manager.py) dédié à la création de balles de tennis


## Utilisation

Démarrer la simulation en lançant ou incluant le fichier [`tennis_court.launch.py`](launch/tennis_court.launch.py).

Lancer le fichier:
```shell
ros2 launch tennis_court tennis_court.launch.py
```

Inclure le fichier:
```python
def generate_launch_description():
    tennis_court_share = get_package_share_directory("tennis_court")
    tennis_court_launch_file = os.path.join(tennis_court_share, "launch", "tennis_court.launch.py")
    tennis_court_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(tennis_court_launch_file)
    )
    return LaunchDescription([tennis_court_launch])
```


### Nœuds

- `gzserver`  
  Le serveur Gazebo, démarré avec le monde `court.world`.
  
- `gzclient` (Optionnel)  
  Le client Gazebo.
  
- `static_transform_publisher`  
  Publie une transformée statique entre la *frame* de référence `map` et celle de la caméra zénithale `zenith_camera_link`.
  
- `ball_manager` (Optionnel)  
  Le script `ball_manager.py`.
  
- `rviz` (Optionnel)  
  L'outil Rviz.
  

### Topics

- `/zenith_camera/image_raw` (*sensor_msgs/msg/Image*)  
  Le flux de la caméra zénithale.
  
- `/zenith_camera/camera_info` (*sensor_msgs/msg/CameraInfo*)  
  Les informations de calibration de la caméra zénithale.


### Paramètres

- `manager` (*bool*, default: true)  
  Si vrai, démarre le script `ball_manager.py`.
  
- `rviz` (*bool*, default: false)  
  Si vrai, démarre Rviz.

- `gui` (*bool*, default: true)  
  Si vrai, démarre le client Gazebo.
  
- `paused` (*bool*, default: false)  
  Si vrai, démarre la simulation dans l'état `pause`.
