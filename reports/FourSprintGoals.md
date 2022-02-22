# Objectifs du 22/02/2022

PO: Hugo PIQUARD


## Fonctionnalités attendues

###### A compléter avec la liste des attentes fonctionnelles du produit en fin de sprint.

* Cartographier l'environnement avec Nav2.
* Controle simple du déplacement du robot.
* Intégration des capteurs du robot.
* Repèrage du robot en vision.

marqueur robot
Hugo fin de vision
launc + readme merge
## Tâches à réaliser

###### A compléter avec la liste macroscopique des tâches à réaliser afin d'implémenter les fonctionnalités détaillées précédemment.

* Finir la cartographie avec Nav2
* Commencer à metre en place le SLAM avec Nav2
* Intégration du capteur LIDAR avec Nav2
* Mise en place d'un algorithm d'évitement d'obstacle simple.
* Repérage du robot par vision avec la caméra globale du projet.

## Challenges techniques

###### A compléter avec la liste argumentée des challenges et verrous techniques liées aux tâches à réaliser.

* Concordance de la position des balles et de la carte à l'issue du SLAM :
  * La position des balles est donnée dans le repère image.
  * La carte obtenue par le SLAM sera le repère de référence.
  
* Évitement du filet :
  * Repérage du filet avec le LIDAR.
  * Proposer une trajectoire d'évitement du filet fonctionnelle.

* Repérage du robot dans son environnement:
  * Position précise du robot (vitesse et cap) avec la caméra globale.
  * Utilisation d'élément de couleurs sur le robot pour obtenir le cap.
  
* Intégration de l'environnement, de la carte et des capteurs fonctionnelle :
  * Bonne comunication entre les nodes, définition précise des topics.
  
