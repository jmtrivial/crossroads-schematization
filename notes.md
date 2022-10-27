# Développement

PYTHONPATH=$PWD examples/get-crossroad-schematization.py   --branches --exact-islands --osm  -l --display-preview  -c 45.77617 3.09027



## Ce qui reste à faire

* les graphes complexes dans les branches:
    * attention aux voies de service...
* les petits ilots à trois passages piétons, ça arrive que le point soit dans le dilaté de l'OSM. on pourrait le décaler un peu
* les grands îlots (ex: Lafayette)
    * si le rayon de l'îlot est grande, et que les directions des passages piétons sont dans 2 directions majoritaires, on suppose un îlot en longueur, et on reconstruit un segment (en utilisant les voies adjacentes ?)
    * un îlot (triangulaire) vraiment grande devient comme un trottoir. Exemple de Galaxie. Utiliser la distance du passage piéton au centre de l'îlot, avec un seuil qui dépend de la largeur des voies locales pour décider
* la stylisation
* les passages piétons sur des nodes à plus de 3 ways



## Propositions d'implémentation

* envisager une largeur différente entre début et fin de branche
* utiliser le bâti pour corriger les courbes, ignorer des îlots


## problèmes rencontrés

### Ballainvilliers : 45.77579 3.08686

RAS

### Lafayette : 45.77332 3.09232

* une configuration incorrecte de way en double: https://forum.openstreetmap.fr/t/voies-superposees/9254
* un îlot bâti est ignoré (considéré comme un îlot piéton). Devrait-on le virer dès la segmentation ?
* on aimerait un îlot tout en longueur. Détecter plusieurs centres ?
* la courbe au sud est trop rognée. Utiliser le bâti ?

### Le carré : 45.76951 3.09093

RAS

### Bas de Lecoq : 45.77061 3.08785

RAS

### Manon : 45.77725 3.07309

* l'îlot n'est-il pas un peu long ?

### Carrefour de la Jetée : 45.77743 3.09013

* les îlots internes un peu longs, de forme chelou ? 


### Carrefour de master : 45.77617 3.09027

* virage nord-ouest trop serré. Considérer l'intersection avec le reste de l'OSM (cf bâti plus haut aussi)

### Carrefour de Gauthier : 45.77727 3.09615

* les deux voies au sud ne sont pas les mêmes, et le départ de la voie la plus à l'est est très en intérieur du carrefour
* problème positionnement de trottoir à l'ouest

### Place des carmes : 45.78223 3.09546

* le milieu des passages piétons c'est pas mal, mais si on est dans le dilaté de l'OSM, on pourrait déplacer un peu
* les voies au nord portent le même nom, parce que ce sont des voies de place, et ça fait un rendu tout pété
* la voie à l'ouest paraît bien fine

### Côte-Blatin / Léon Blum : 45.77189 3.09384

* RAS (sauf peut-être la longueur de l'îlot)
* positionnement du passage piéton voie nord-est pas dingue (data OSM)

### Comptoir Irlandais : 45.77674 3.07708

RAS

### Carrefour de Marlène : 45.78096 3.07622 --c1 4 --c2 5

* L'îlot pourrait être surfacique

### Galaxie : 45.78220 3.07278

* Un îlot trop grand vers l'intérieur (est-ce qu'on réduit bien la longueur suivant la largeur réelle des voies ?)

### Les p'tits débrouillards : 45.78300 3.07844

BUG: Un node passage piéton à 4 ways : https://www.openstreetmap.org/node/1036743876

### Carrefour Liève : 45.78191 3.09890

RAS

### Rond-point vélorution : 45.78147 3.07065 --c2 5

RAS

### Rond-point Aubière : 45.75888 3.10164

BUG: crash

* Auparavant: Un îlot qui manque à l'ouest car il n'y a pas de passage piéton

### Carrefour du tram : 45.78750 3.10580 --c1 4 --c0 3

RAS
