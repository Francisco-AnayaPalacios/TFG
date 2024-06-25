### LiDAR_con_odometria.py
Código que publica el mapa y la nube de puntos que publica el LiDAR superpuesta teniendo en cuenta la pose estimada del sensor.

### Mejor_medida_GPS.py
Guarda la mejor medida de del receptor GPS en base a la covarianza en x y almacena la pose para ese momento en el sistema de referencia relativo al mapa.

### Postprocesamiento_scans.ipynb
Cuaderno que unifica lo que puede definirse como el postprocesamiento de los scans: 
combinación -> voxelización -> filtrado -> georreferenciación

### localization_node.py
Define la pose odométrica según el ruido deseado y ejecuta ICP y define la pose estimada.

Los códigos que terminan en iterations_auto y voxel_size_auto simplemente automatizan la ejecución para una lista de valores para el posterior análisis.
