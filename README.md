### Tennis Ball Collector
<img src="https://forthebadge.com/images/badges/made-with-python.svg" />
Ceci est un template de dépôt Git pour le cours d'ingénierie système et modélisation robotique à l'ENSTA Bretagne en 2021.


## Lancer la simulation

### Dépendences


```bash
sudo apt install ros-<ROS-DISTRO>-gazebo-* ros-<ROS-DISTRO>-joint-state-publisher ros-<ROS-DISTRO>-joint-state-publisher-gui \
pip install imutils
```

### Démarrer la simulation


```bash
cd ros2_workspace
colcon build --symlink-install
source install/setup.bash
```
Dans un terminal :
```bash
ros2 launch tennis_court tennis_court.launch.py
```

Dans un autre terminal
```bash
ros2 launch tennis_ball_collector_launch mission_1.launch.py
```

## Groupe

### Membres

| Nom                                            |
|------------------------------------------------|
| [Samuel PROUTEN](https://github.com/samprt)    |
| [Hugo SABATIER](https://github.com/Hugosabb)        |
| [Maxime LEGEAY](https://github.com/MaxLgy)     |
| [Estelle ARRICAU](https://github.com/estellearrc)|
| [Hugo PIQUARD](https://github.com/hugoPiq)     |
| [Antonin BETAILLE](https://github.com/Anton1B) |




### Gestion de projet

###### Lien vers le [Taiga](https://tree.taiga.io/project/hugopiq-vadrouilletbc/backlog).



## Structure du dépôt

```
your_folder/
│
└───TennisBallCollector/
│   └───compute_trajectory/
|   |
|   └───docs/
|   |
|   └───interfaces/
|   |   └───mgs/  *Messages personalisés*
|   |
|   └───reports/  *Rapports PO*
│   |
└───ressources/
│   |
└───roblochon_gazebo/
│   |
└───robochon_description/  *Description du robot*
|   |   └───urdf/  *Scripts urdf du robot*
│   |
└───sam_bot_description/
│   |
└───tennis_ball_collector_launch/  *Package launch*
|   |   └───launch/  *Script launch*
|   |
└───tennis_ball_detector/ *Packages et fichier liès à la détection des balles*
|   |   └───ressource/  *Scritps*
|   |   └───tennis_ball_dectector/ *Package*
│   |
└───tennis_court/  *Monde de la simulation*
│   |

```
### Package `tennis_court`

Le dossier `tennis_court` est un package ROS contenant le monde dans lequel le robot ramasseur de balle devra évoluer ainsi qu'un script permettant de faire apparaître des balles dans la simulation.
Ce package ne doit pas être modifié.
Consulter le [README](tennis_court/README.md) du package pour plus d'informations.


### Documents

Le dossier `docs` contient tous les documents utiles au projet:
- Des [instructions pour utiliser Git](docs/GitWorkflow.md)
- Un [Mémo pour ROS 2 et Gazebo](docs/Memo_ROS2.pdf)
- Les [slides de la présentation Git](docs/GitPresentation.pdf)


### Rapports

Le dossier `reports` doit être rempli avec les rapports d'[objectifs](../reports/GoalsTemplate.md) et de [rétrospectives](../reports/DebriefTemplate.md) en suivant les deux templates mis à disposition. Ces deux rapports doivent être rédigés respectivement au début et à la fin de chaque sprint.
