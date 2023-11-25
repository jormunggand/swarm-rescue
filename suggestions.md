# Suggestions et idées

Mettez ici toutes les idées qui vous passent par la tête

## Première idée de drone

Fonctionnement basé sur la création d'une map interne que le robot utilise pour prendre
ses décisions

3 états: 
SEARCHING: à la recherche d'un blessé
FETCHING: récupérer le blessé
RESCUING: emmener le blessé jusqu'à la zone de secours


## Idées 

> Plus de points pour l'exploration (60% des points) (20% pour les deux autres)

> Utiliser une map
- dès qu'un drone trouve une zone de secours il l'indique 
- cette map est partagée entres les drones

> Détection d'un autre drone avec le capteur sémantique
  - si 1 porte un blessé alors 2 le laisse passer/l'évite
  - si 1 et 2 portent des blessés celui qui a détecté l'autre laisse passer/évite
  - pareil si aucun ne portent de blessés
  
  ? que se passe-t-il si les deux drones se détectent mutuellement et s'arrêtent 



> Utiliser l'odométrie en intégrant les données sur le temps pour avoir
une estimation de la position sans le GPS
> Utiliser "proportionate control" pour des actions comme s'aligner sur un angle
> on peut attraper de tous les côtés du drone

