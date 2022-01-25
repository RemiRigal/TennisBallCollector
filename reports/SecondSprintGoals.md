# Objectifs du 25/01/2022

PO: Estelle ARRICAU


## Fonctionnalités attendues

###### A compléter avec la liste des attentes fonctionnelles du produit en fin de sprint.

* Obtenir l'ordre dans lequel les balles doivent être ramassées par le robot.
* Déplacer le robot par téléopération sur le terrain de tennis.

## Tâches à réaliser

###### A compléter avec la liste macroscopique des tâches à réaliser afin d'implémenter les fonctionnalités détaillées précédemment.

* Numéroter les balles dans leur ordre d'apparition sur le terrain avec leur position une fois arrêtée au sol.
* Tester la brique nav2 de ROS 2 pour déplacer le robot sur le terrain de tennis.
* Calcul du chemin le plus rapide passant par une liste de points.

## Challenges techniques

###### A compléter avec la liste argumentée des challenges et verrous techniques liées aux tâches à réaliser.

* Détection des balles immobiles uniquement :
  * les balles en mouvement (spawn) ne sont pas détectées (position balle entre 2 frames successives assez proche)
  * les balles ramassées par le robot ne doivent pas être détectées (cachées sous le robot quand elles sont ramassées)
* Actualisation de la liste des balles à ramasser :
  * Ajout de nouvelles balles dans la liste
  * Suppression des balles déjà ramassées par le robot de la liste et de celles dans la zone orange
  * Modification de la position d'une balle si elle est poussée par le robot lors de son trajet de ramassage ? (identifier qu'il s'agit de la même balle que précédemment et non une nouvelle)

