# Simulateur de physique 3D

## Fonctionnalités


### Champs électromagnétiques

-   Sources de charge électrique positive et négative.
-   Sources magnétiques.
-   Visualisation des champs.
-   Variation de la charge de l'objet principal.

### Interface

-   Caméra 3D contrôlable.
-   Placement d'objets à la souris.
-   Mode filaire et panneau d'aide.
-   Pause et réinitialisation de la simulation.

## Dépendances

-   Un compilateur C compatible, par exemple **GCC**.
-   **Raylib** et ses fichiers d'en-tête (`raylib.h`, `raymath.h`).
-   La bibliothèque mathématique standard C.

Sur Ubuntu, vous pouvez installer les outils de compilation avec :

``` bash
sudo apt update
sudo apt install build-essential
```

Raylib doit également être installée. Selon la méthode d'installation
choisie, vérifiez que le compilateur trouve ses en-têtes et sa
bibliothèque.

## Compilation sous Linux

Placez-vous dans le dossier qui contient le fichier source. Si celui-ci
s'appelle `main.c`, lancez :

``` bash
gcc main.c -o simulateur -std=c99 -Wall -Wextra -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
```

Puis démarrez le programme :

``` bash
./simulateur
```

Si votre installation de Raylib nécessite des options de liaison
différentes, adaptez la commande à la documentation de votre
installation.

## Commandes

  -----------------------------------------------------------------------
  Touche                              Action
  ----------------------------------- -----------------------------------
  `1` à `5`                           Choisir la forme de l'objet
                                      principal

  `P`                                 Entrer dans le mode de placement
                                      libre de l'objet principal

  Clic gauche                         Choisir une position ou placer
                                      l'élément sélectionné

  Molette                             Régler la hauteur en mode de
                                      placement `P` ; sinon, zoomer la
                                      caméra

  `Entrée`                            Valider le placement libre

  `Échap`                             Annuler le placement en cours

  `E`                                 Sélectionner une source électrique
                                      positive

  `Q` ou `A`                          Sélectionner une source électrique
                                      négative

  `B`                                 Sélectionner une source magnétique

  `M` ou `;`                          Sélectionner un mur

  `N`                                 Sélectionner une pente inclinée

  `J`                                 Sélectionner un pendule

  Clic gauche après sélection         Placer la source, le mur, la pente
                                      ou le pendule sur le plateau

  `Retour arrière`                    Supprimer la dernière source
                                      électromagnétique

  `+`                                 Augmenter la charge de l'objet
                                      principal

  `-`                                 Diminuer la charge de l'objet
                                      principal

  `F`                                 Afficher ou masquer les champs

  `W` ou `Z`                          Activer ou désactiver le mode
                                      filaire

  `Espace`                            Mettre en pause ou reprendre la
                                      simulation

  `R`                                 Réinitialiser l'objet et vider la
                                      scène ajoutée

  `H`                                 Afficher ou masquer l'aide

  `C`                                 Recentrer la caméra

  Flèches directionnelles             Pousser l'objet principal

  `Échap`                             Annuler le placement en cours
  -----------------------------------------------------------------------

Les commandes `Q/A`, `W/Z` et `M/;` visent à faciliter l'utilisation sur
les dispositions de clavier AZERTY et QWERTY. La disponibilité exacte
des touches peut dépendre de la disposition et de la configuration
reconnues par Raylib.

## Utilisation

1.  Lancez le programme.
2.  Utilisez `1` à `5` pour choisir la forme de l'objet principal.
3.  Ajoutez un mur (`M`), une pente (`N`) ou un pendule (`J`), puis
    cliquez sur le plateau pour le placer.
4.  Ajoutez des sources électriques (`E`, `Q/A`) ou magnétique (`B`) et
    cliquez pour les positionner.
5.  Appuyez sur `F` pour afficher ou masquer la visualisation des
    champs.
6.  Utilisez `Espace` pour mettre la simulation en pause, et `R` pour
    repartir d'une scène réinitialisée.

Le panneau d'aide intégré (`H`) rappelle les principales commandes.
