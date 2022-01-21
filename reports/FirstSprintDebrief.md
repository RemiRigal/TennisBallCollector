# Debrief du 21/01/2022
PO: Maxime Legeay


## Bilan

Pourcentage de tâches réalisées: 80 %

### Ce qui a fonctionné

La détection de balles, la réutilisation de codes déjà établis et l'application à ce problème particulier. La répartition des tâches s'est faite naturellement et chacun a pris une tâche qu'il maitrise donc le travail a été très efficace.


### Ce qui n'a pas fonctionné

Un membre de l'équipe est malade (Covid) et avait pourtant des compétences intéressantes en simulation dont le groupe aurait eu besoin. Cependant, le groupe a fait preuve d'une belle adaptation.
Au début tout le monde voulait participer en inventant un design original pour le robot et de bonnes idées ont émergé. Cependant, on a perdu du temps pour effectuer des tâches élémentaires nécessaires pour lancer le projet (avoir un robot qui se déplace avant de lui demander de faire des tâches complexes).
Un problème de configuration sur l'ordinateur d'un membre a fortement ralenti sa progression mais sa maitrise du sujet a su compenser ce problème de dernière minute.


### Retour d'expérience du PO

Groupe très agréable à encadrer et très volontaire, expérience intéressante en tant que PO qui permet d'avoir une vision globale du projet. Rapports à remplir un peu pénible. Difficile de prendre en main les outils comme Taïga dès le début mais une fois pris en main ils permettent une organisation bien plus efficace.
Prendre une bonne habitude de pull requests / forks / merges sur github semble être un atout majeur pour la lisibilité du projet. 


### Conseils pour le prochain PO

L'équipe est très volontaire et il faut s'en servir comme une force, laisser les gens s'exprimer et prendre en compte les remarques de chacun qui sont souvent pertinentes. Il vaut mieux discuter en début de séance en lançant le sujet, puis écouter ce que chacun pense devoir faire pour créer une tache adaptée plutot que d'imposer des taches a chacun selon la vision du PO.
Cependant, une fois que les taches sont attribuées, il faut veiller à ce que chacun accomplisse sa mission et ne se détourne pas vers la mission de quelqu'un d'autre, afin que les taches avancent vite et à peu près au même rythme.



## Nouvelles mesures

Création de la branche devel sur git et refus de tout pull request sur le main -> les pull request des membres doivent absolument être faites sur la branche devel, acceptées par le PO, testées par le PO et si elles fonctionnent uniquement, merge avec le main.

Peut-être basculer sur Docker pour être certain de la compatibilité du projet entre les machines ? (Pb de compatibilité entre ROS Foxy et Galactic en début de séance)

