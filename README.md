# Final Degree Project (Bachelor’s Thesis)  
## 3D Outdoor Mapping with a Robot-Mounted LiDAR  

<p align="center">
  <img width="1842" height="740" alt="image" src="https://github.com/user-attachments/assets/a252fdda-e340-44af-afd0-f5c19f55e877" width="750">
</p>

<p align="center">
  <em>Point cloud generated overlaid with 3D building models and aerial photographs of the mapped area.  
  Target building: ETSII Faculty, University of Málaga (Bulevar Louis Pasteur 35, Campus de Teatinos, 29071 Málaga).</em>
</p>

![GitHub top language](https://img.shields.io/github/languages/top/Francisco-AnayaPalacios/TFG) 
![GitHub repo size](https://img.shields.io/github/repo-size/Francisco-AnayaPalacios/TFG)

---

**Official title in Spanish:**  
*Construcción de mapas de exteriores con un LiDAR 3D embarcado en un robot móvil*  

---

## 📖 Overview
This project consists of the development of a **LiDAR-based mapping and localization pipeline** implemented as my **Final Degree Project (Bachelor’s Thesis)** at the *University of Málaga (ETSII)*.  

The system focuses on generating **dense 3D point-cloud maps** of outdoor environments using a robot-mounted LiDAR sensor. The mapping process integrates algorithms such as **Iterative Closest Point (ICP)** for scan matching and alignment, and leverages advanced SLAM approaches like **Fast-LIO2** (LiDAR-Inertial Odometry) within **ROS 2** and **PCL**.  

The generated maps are **georeferenced**, which allows them to be exploited in **outdoor, semi-outdoor, and indoor environments**, even in **GPS-denied scenarios**.  

---

## 🛠️ Technologies & Tools
- **Programming Languages:** Python, C++, C  
- **Frameworks & Middleware:** ROS 2 (Robot Operating System), PCL (Point Cloud Library)  
- **Algorithms & SLAM Approaches:** Iterative Closest Point (ICP), Fast-LIO2 (LiDAR-Inertial Odometry)  
- **Hardware:** 3D LiDAR sensor mounted on a mobile robot  
- **Visualization & Simulation:** RViz, Gazebo  
- **Data Processing:** NumPy, Open3D, Matplotlib  

---

## 🛑 Problem Addressed
Traditional **GPS-based localization** presents significant limitations:  
- **Urban canyons** (multipath, poor satellite visibility).  
- **Indoor and semi-indoor areas** (garages, tunnels).  
- **Altitude navigation**, where GPS accuracy strongly decreases in the vertical axis.  

This implementation provides a **LiDAR-based alternative**, offering consistent **absolute localization** with the **same precision in all axes**, making it especially useful in high-altitude navigation scenarios.  

---

## ✅ Results
- High-resolution **3D point-cloud maps** with detailed geometric information.  
- Robust scan registration using **ICP** algorithms.  
- Efficient localization with **low computational overhead**.  
- Same precision maintained across all axes (X, Y, Z), unlike GPS, which has **lower accuracy in altitude**.  
- Demonstrated applicability to **autonomous navigation** tasks in complex environments.  

---

## 🔮 Possible Future Implementations
The methodology is not limited to a single robot:  
- **Multi-robot fleets:** once an environment has been mapped, the **3D model can be reused**, allowing other robots to localize without remapping.  
- **Autonomous vehicles:** robust backup system in **GPS-denied zones**, especially relevant for drones or ground robots operating in altitude-sensitive tasks.  
- **High-altitude navigation:** precise localization in vertical structures (e.g., tall buildings, towers, industrial plants) where GPS becomes unreliable.  
- **Rescue robotics & industrial inspection:** reliable navigation in collapsed structures, underground tunnels, or multi-level environments.  
- **Smart infrastructure:** integration with digital twins and urban planning systems.  

---

## 🖼️ Screenshots
*Point cloud overlaid with 3D building models and aerial photographs.*  
*Mapped building: ETSII – University of Málaga (Campus de Teatinos).*  

---

## 📂 Repository Structure
```bash
TFG/
 ├── CODES/   → Source code (ICP, Fast-LIO2, ROS2 nodes, processing tools)  
 ├── MAPS/    → Generated point-cloud maps  
 ├── DOCS/    → Presentation, videos, annexes  
 └── README.md
