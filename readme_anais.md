## Pràctica final RA - Part anais

En aquesta carpeta hi ha dues subcarpetes:

- `robot_final`: És el paquet de catkin que conté tots els arxius necessaris, de tal forma que només s'ha de copiar a la carpeta `catkin_ws/src` i fer un `catkin_make` des de la carpeta `catkin_ws` per compilar-lo. El paquet conté:
  - `src`: Conté els codis en cpp que es convertiran en executables.
  - `scripts`: Conté els codis en python. Això és degut a que hem fet part del codi en python i part en cpp.
  - `launch`: Conté l'arxiu de llançament que permet executar tot el projecte amb una sola comanda.
  - `maps`: Conté el mapa que utilitza el robot en format `.yaml` i `.pgm`.
  - `CMakeLists.txt` i `package.xml`: Són els arxius necessaris per a la compilació del paquet de catkin.

- `videos`: Conté els vídeos de les demos d'aquesta part del projecte.