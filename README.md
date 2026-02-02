# Projet Implémentation

Par: Clarence Pfister, Yannis Tobbal
ROB5

## Vue d'ensemble

Ce package ros2 jazzy permet de controller ce robot à l'aide d'un Raspberry Pi3:
![robot](https://github.com/Darkadius/projet-implementation-polytech/blob/main/images/robot.jpg)

Il s'agit d'un robot muni d'un moteur I²C permettant de faire tourner l'aiguille à son extrémité, d'un encodeur quadratique ainsi que d'un IMU.
Il est possible d'utiliser ce robot dans deux modes différents:
- Boussole: Le robot utilise le magnétomètre de l'IMU pour trouver le nord et pointer l'aiguille dans cette direction
- Suivi de ligne: Le robot utilise une caméra pour detecter une ligne noir verticale. Il fait tourner l'aiguille en continue vers la ligne selon le sens du décallage entre le centre

## Architecture

Le package s'organise ainsi:
![architecture](https://github.com/Darkadius/projet-implementation-polytech/blob/main/images/SADT_Implem.png)

Il y a en tout 4 nodes:
- Camera réalise le traitement d'image du flux vidéo pour detecter la ligne ainsi que son décallage par rapport au centre. Elle publie ensuite le sens de rotation du moteur.
- Encoder décode les signaux GPIO de l'encodeur et calcule l'angle entre la position initiale de l'aiguille et sa position actuelle.
- Magneto lit les mesures du magnétomètre et convertit le vecteur du champs magnétique en angle objectif pour l'aiguille.
- Hardware gère la loi de commande et envoie des messages aux moteurs. Selon le mode actuel, il réalise un contrôle en vitesse avec boucle ouverte (suivi de ligne) ou bien un contrôle en position avec boucle fermée, utilisant un correcteur proportionnel intégral (boussole).

Ces nodes communiquent sur 3 topics:
- /sens: Camera -> Hardware, contient le sens de rotation que le controlleur doit envoyer aux moteur pour suivre la ligne.
- /angle: Encoder -> Hardware, contient l'angle actuel de l'aiguille.
- /goal_angle: Magneto -> Hardware, contient l'angle à atteindre pour pointer vers le nord.

Enfin, le package est dôté de 2 launch files:
-boussole_launch.xml: Lance Hardware, magneto et encoder pour le contrôle de la boussole.
-suiviligne_launch.xml: Lance Hardware et camera pour le suivi de ligne.

## Guide d'utilisation

### 0 - Matériel:
**Materiel Physique:**
- Robot
- Raspberry Pi3
- Clavier et écran (ou pc pour se connecter au Pi3 à distance)
- Caméra, peu importe le modèle

**Matériel informatique:**
- Ubuntu 24.04 installé sur le Pi3
- ros2 jazzy (version de base)
- libgpio
- opencv
- openssh (optionel)

### 1 - Branchements:
Voyons comment connecter le robot au Pi3:

1) Connecter le cable M1 de l'encodeur au port B1 du moteur, faire de même pour M2 vers B2.
2) Connecter le cable GPIO de l'encodeur au port D5 du raspberry Pi3
3) Connecter le cable I²C de l'IMU au port I²C millieu du raspberry Pi3
4) Connecter le cable I²C du moteur au port I²C gauche du raspberry Pi3
5) Connecter le cable d'alimentation du moteur au pin 1, et le cable de la masse au pin 9 du Raspberry Pi3
6) Enfin, connecter la camera à un port USB du Raspberry Pi3

### 2 - Installation:
Installons ensuite les composants nécéssaire au package sur le raspberry:

1) Installer ros2: ![Guide d'installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
2) Installer opencv: ```sudo apt install libopencv-dev```
3) Installer libgpio: ```sudo apt install libgpiod-dev```

Maintenant, il faut s'occuper du package en lui même.
1) Télécharger le package depuis son répo: ```wget https://gitlabsu.sorbonne-universite.fr/clarence-yannis/projet-implementation.git```
2) Extraire et placer le package dans le workspace de ros2. Si vous êtes en connexion ssh, alors utilisez la commande ```scp [NOM]@[ADRESSE] :~/[NOM_DE_VOTRE_WORKSPACE]/src rob5_implem
3) Ouvrir un terminal depuis le workspace et build le package: ```colcon build```.
4) Sourcer le package: ```source install/setup.bash```

Le robot est maintenant prêt à être utilisé!
### 3 - Utilisation.
**Boussole**
1) Placer l'aiguille en position neutre, comme suivant la photo:
![neutral](https://github.com/Darkadius/projet-implementation-polytech/blob/main/images/aiguille.jpg)
2) Lancer le programme: ```ros2 launch rob5_implem boussole_launch.xml```
Le robot s'initialise et l'aiguille pointe maintenant vers le nord: faites tourner le robot et admirez l'aiguille qui se repositionne.

**Suivi de ligne**
1) Lancer le programme: ```ros2 launch rob5_implem suiviligne_launch.xml```
