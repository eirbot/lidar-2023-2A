# Objectifs
Utilisation d'un RP Lidar A1 ([datasheet](https://bucket-download.slamtec.com/e680b4e2d99c4349c019553820904f28c7e6ec32/LM108_SLAMTEC_rplidarkit_usermaunal_A1M8_v1.0_en.pdf)) pour la localisation du robot et de l'adversaire sur la table.

# Mise en place de Mbed
Pour pouvoir téléverser le code de ce dépôt il faut :
`mbed deploy`
`mbed export -i cmake_gcc_arm` si vous êtes sur CLion
`mbed export -i vscode_gcc_arm` si vous êtes sur VSCode
`mbed compile --flash` pour compiler et téléverser 
`mbed sterm -b 115200`pour créer un moniteur série à 115200 baud/s avec la carte

# Matériel
Une Nucleo-L412KB, la PinOut est compatible aussi avec une Nucleo-L432KC.

## PinOut
Nous avons le [PinOut](PinOut/PinOut.xlsx) qui permet à la fois la génération d'une PWM, 2 écritures GPIO, lecture UART et lecture USB (D+ et D-).

La photo disponible est celle du PinOut d'une Nucleo L432KC. Pour une Nucleo-L412KB c'est la même chose sans DAC, SPI3, CAN, SAI.

Pour ajouter des éléments à ce PinOut, veuillez-vous référer au fichier *PeripheralPins.c* qui se trouve dans le dossier générer par le `mbed deploy` dans *mbed-nucleo-l412kb/blob/main/TARGET_NUCLEO_L412KB* (Sur [GitHub](https://github.com/Copper-Bot/mbed-nucleo-l412kb/blob/main/TARGET_NUCLEO_L412KB/PeripheralPins.c)).

## Kicad 
Une carte a été réalisée pour pouvoir interagir avec le Lidar en USB et en UART. 
Nous avons mis à disposition le projet Kicad.

# Remerciements
Nous utilisons le dépôt de Mr Copper-Bot [GitHub](https://github.com/Copper-Bot/mbed-nucleo-l412kb) pour pouvoir téléverser et compiler sur une Nucleo-L412KB.