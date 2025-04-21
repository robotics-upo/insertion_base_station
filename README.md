# insertion_base_station
Paquetes relacionados con la estacion base del proyecto Insertion


Ejecuta los siguientes comandos para generar el docker y ejecutarlo.

> cd docker
> docker build . -t insertion


Para lanzar el docker tienes un script. Primero tienes que generar el directorio $HOME/insertion_shared

> cd ~
> mkdir insertion_shared
> cd insertion_base_station/scripts
> ./run_container.bash

Ya tendrías el contenedor lanzado para ejecutar el rviz, etc.

Tienes que crear un catkin_ws en el contenedor y descargarte los módulos que estimes oportunos.

La idea es que puedas lanzar un RViz en desde el contenedor y añadir el RvizSatellite y el tópico /dji_sdk/gps_position.

A partir de ahí, construiremos más cosas.
