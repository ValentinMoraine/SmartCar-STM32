# Notes étudiants - SmartCar STM32

## Objectif

Le but n'est pas uniquement de faire bouger le véhicule.

Le but est de comprendre une architecture embarquée supervisée :

```
Qt = IHM opérateur
STM32 = contrôle local
HAL = accès au matériel
MotoDriver2 = puissance moteur
HC-SR04 = capteur de distance
```

## Règle de sécurité

La STM32 doit arrêter ou refuser l'avance si l'obstacle est trop proche.

L'IHM Qt peut masquer le bouton "Avancer", mais cette protection n'est qu'une aide opérateur.

La vraie sécurité doit rester dans le firmware.
