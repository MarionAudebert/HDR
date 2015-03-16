# HDR

Projet permettant à partir de plusieurs images avec differents temps d'exposition de recréer une seule image.

### Langage
C++

### Compilation et execution
Un fichier cmake permet de générer le makefile. 
Vous trouverez des images de tests dans input (test1, test2, test3 ou test4)
```sh
$ cd HDR
$ cmake .
$ make
$ ./bin/hdr input/test1/*
```
L'image résultat sera disponible dans le dossier output.

### Mathematiques
Vous trouverez le rapport de projet expliquant les mathématiques utilisées pour ce projet dans le fichier rapport.pdf

### Images tests
Quatre séries d'images trouvées sur http://pages.cs.wisc.edu/~csverma/CS766_09/HDRI/DataSet/HDRImageSet.html
