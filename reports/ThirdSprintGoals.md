# Objectifs du 21/02/2022

PO: Antonin BETAILLE


## Fonctionnalités attendues

###### A compléter avec la liste des attentes fonctionnelles du produit en fin de sprint.

* Améliorer le robot de façon à optimiser le ramassage de balles
* Cartographier l'environnement avec Nav2

## Tâches à réaliser

###### A compléter avec la liste macroscopique des tâches à réaliser afin d'implémenter les fonctionnalités détaillées précédemment.

* Comparer les caractéristiques de notre robot avec le modèle proposé par Nav2.
* Approfondir le travail de recherche sur nav2 effectuer une commande de trajectoire.
* Implémentation de l'algorithme d'optimisation du trajet

## Challenges techniques

###### A compléter avec la liste argumentée des challenges et verrous techniques liées aux tâches à réaliser.

* Concordance de la position des balles et de la carte à l'issue du SLAM :
  * La position des balles est donnée dans le repère image
  * La carte obtenue par le SLAM sera le repère de référence
  * --> Il faut trouver la transformation entre les deux repères
  
* Implémenter l'algorithme d'optimisation à l'aide du planner de Nav2 :
  * Comment le planner définit-il la trajectoire à prendre en compte ?
  * Le SLAM prend-il en compte le filet ou faut il créer une zone opaque dans l'environnement pour prendre en compte le filet ? 
  
* S'assurer que le robot est en mesure de ramasser une balle à l'issue d'une trajectoire plannifiée par Nav2 :
  * Tester plusieurs cas de figure pour vérifier si le robot ramasse la balle sans difficulté
  * Si besoin, mettre en place une deuxième phase d'approche pour ramasser la balle uniquement (vision)
