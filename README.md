# Challenge Robotique 2023 - IMT Nord Europe
 Développeurs : Julien MÉRAND - Quentin DESMARTIN


Le principe de l'exercice est d'optimiser la récompense finale d'un robot qui passe par des points dont les coordonneés sont définis à l'avance, dans une liste. Le robot démarre d'une position 0. À chaque fois que le robot passe par un point, il empoche la récompense liée à ce point.
Le robot part avec une quantité de 10L de carburant, qu'il consomme au fur et à mesure de la distance parcourue entre les points.
A chaque fois que le robot passe par un point, sa masse totale augmente de la masse liée à ce point et vaut donc Masse_Tot. La vitesse du robot entre deux points est de (1-exp(-600 / Masse_Tot).
Il faut donc veiller à optimiser le chemin pour que le robot amasse le plus de récompenses en moins de 600 secondes.

Peux-tu créer un programme python qui prend en entrée la liste des points visitables, avec leur indice, leurs coordonnées, leur masse et leur récompense et retourne la liste des indices des points à visiter, dans l'ordre ?
