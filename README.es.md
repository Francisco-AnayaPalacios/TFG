# Trabajo de Fin de Grado  
## Construcción de mapas de exteriores con un LiDAR 3D embarcado en un robot móvil  

[![GitHub license](https://img.shields.io/github/license/PacoAnaya/TFG)](LICENSE) 
![GitHub top language](https://img.shields.io/github/languages/top/PacoAnaya/TFG) 
![GitHub repo size](https://img.shields.io/github/repo-size/PacoAnaya/TFG)

---

### 📖 Descripción
Este proyecto corresponde al **Trabajo de Fin de Grado** realizado en la *Universidad de Málaga (ETSII)*.  

Se centra en la generación de **mapas densos de nubes de puntos 3D** mediante un sensor LiDAR embarcado en un robot móvil.  
Para la alineación y registro de escaneos se han utilizado algoritmos como **Iterative Closest Point (ICP)** y técnicas avanzadas como **Fast-LIO2** (Odometría LiDAR-Inercial), integradas en frameworks de robótica como **ROS 2** y **PCL**, logrando resultados escalables y precisos.  

Los mapas obtenidos están **georreferenciados**, lo que permite su uso en **exteriores, semiexteriores e interiores**, incluso en **entornos sin GPS**.  

---

### 🛠️ Tecnologías y Herramientas
- **Lenguajes de programación:** Python, C++, C  
- **Frameworks & Middleware:** ROS 2 (Robot Operating System), PCL (Point Cloud Library)  
- **Algoritmos y técnicas SLAM:** Iterative Closest Point (ICP), Fast-LIO2 (Odometría LiDAR-Inercial)  
- **Hardware:** Sensor LiDAR 3D embarcado en robot móvil  
- **Visualización & Simulación:** RViz, Gazebo  
- **Procesamiento de datos:** NumPy, Open3D, Matplotlib  

---

### ✅ Logros
- Mapas 3D de alta resolución con gran nivel de detalle geométrico.  
- Registro robusto mediante **ICP** y **Fast-LIO2**.  
- Localización eficiente con **baja carga computacional**.  
- **Misma precisión en todos los ejes (X, Y, Z)**, a diferencia del GPS que pierde exactitud en el eje vertical.  
- Aplicación demostrada en navegación autónoma en entornos complejos.  

---

### 🛑 Problema abordado
Los sistemas de localización **basados en GPS** presentan limitaciones importantes:  
- **Cañones urbanos** con baja visibilidad satelital.  
- **Entornos interiores o seminteriores** (túneles, aparcamientos).  
- **Navegación en altura**, donde el GPS es mucho menos preciso en el eje Z.  

Esta implementación ofrece una **alternativa basada en LiDAR**, proporcionando una **localización absoluta precisa y uniforme en todos los ejes**, de gran interés en aplicaciones de navegación en altura.  

---

### 🔮 Posibles Implementaciones Futuras
La metodología es aplicable más allá de un único robot:  
- **Flotas de robots:** una vez mapeado un entorno, el modelo 3D puede reutilizarse para que otros robots se localicen sin necesidad de volver a mapear.  
- **Vehículos autónomos:** sistema de respaldo robusto en **zonas sin GPS**, especialmente útil en tareas sensibles a la altura.  
- **Navegación en altura:** localización precisa en estructuras verticales (rascacielos, torres, plantas industriales) donde el GPS pierde fiabilidad.  
- **Robótica de rescate e inspección industrial:** navegación fiable en minas, edificios colapsados o entornos multinivel.  
- **Infraestructuras inteligentes:** integración con *digital twins* y sistemas de planificación urbana.  

---

### 🖼️ Capturas
<p align="center">
  <img src="https://github.com/PacoAnaya/TFG/assets/145780472/5b7bc1b5-85b5-442b-8fcf-9b03f0569dfe" width="700">
</p>

*Mapa de puntos sobrepuesto con modelos 3D de edificios y fotografías aéreas.*  
*Edificio objeto del mapeado: ETSII – Universidad de Málaga (Campus de Teatinos).*  

---

### ⚙️ Cómo usar
Clona el repositorio:  
```bash
git clone https://github.com/PacoAnaya/TFG.git
cd TFG
