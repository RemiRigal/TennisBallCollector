# Debrief du 25/01/2022

PO: Estelle ARRICAU


## Bilan

Pourcentage de tâches réalisées: 70 %

### Ce qui a fonctionné

* Calcul du trajet le plus rapide passant par la liste de position des balles sur le terrain.
* Passer par la zone orange de dépôt des balles quand le robot est proche.
* Détecter les balles et créer la liste des possitions des balles avec leurs dimensions et leur instant d'apparition sur le terrain.
* Modèle simple du robot avec 2 bras rabatteurs sous le châssis.

### Ce qui n'a pas fonctionné

* Détecter qu'une balle sur 2 frames successives est identique même si elle s'est déplacée un peu et actualiser sa position : parfois, il différencie une même balle qui a trop bougé d'une frame à l'autre.
* Ramasser les balles avec le robot : elles rebondissent sur les bras rabatteurs...


### Retour d'expérience du PO

Groupe très agréable à encadrer et très volontaire, expérience intéressante en tant que PO qui permet d'avoir une vision globale du projet. Rapports à remplir un peu pénible. Difficile de prendre en main les outils comme Taïga dès le début mais une fois pris en main ils permettent une organisation bien plus efficace.
Prendre une bonne habitude de pull requests / forks / merges sur github semble être un atout majeur pour la lisibilité du projet. 

### Conseils pour le prochain PO

L'équipe est très volontaire et il faut s'en servir comme une force, laisser les gens s'exprimer et prendre en compte les remarques de chacun qui sont souvent pertinentes. Il vaut mieux discuter en début de séance en lançant le sujet, puis écouter ce que chacun pense devoir faire pour créer une tache adaptée plutot que d'imposer des taches a chacun selon la vision du PO.
Cependant, une fois que les taches sont attribuées, il faut veiller à ce que chacun accomplisse sa mission et ne se détourne pas vers la mission de quelqu'un d'autre, afin que les taches avancent vite et à peu près au même rythme.


## Nouvelles mesures

* Idée pour le contrôleur : champ de potentiel pour suivre les lignes générées par l'algo du voyageur de commerce.
* Gestion des personnes à distance avec un appel collectif en début de séance comme mis en place aujourd'hui.
