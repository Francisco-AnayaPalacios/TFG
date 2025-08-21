# Final Degree Project (Bachelor’s Thesis)  
## 3D Outdoor Mapping with a Robot-Mounted LiDAR  

**Official title in Spanish:**  
*Construcción de mapas de exteriores con un LiDAR 3D embarcado en un robot móvil*  

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
- **Visualization & Simulation:** RViz  
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

## 🔬 Experiments & Evaluation

To ensure robustness, automated testing scripts to benchmark the SLAM and mapping pipeline under different configurations were developped.  
Key experiments included:

- **Voxelization analysis**: maps were reconstructed using multiple voxel resolutions to evaluate trade-offs between accuracy and computational efficiency.  
- **Error metrics**: trajectory drift and point cloud alignment error were computed automatically for each configuration.  
- **Automated workflow**: evaluation scripts streamlined testing across simulation and real-world datasets, ensuring reproducibility of results.

This systematic evaluation allowed the identification of optimal voxel sizes that preserved accuracy (< X cm RMSE) while reducing computational load, contributing to a more robust and deployable solution.

---

## 📊 Published Datasets

Within the scope of this thesis, a **Mobile Robot Dataset** was collected using an **Ouster OS1-32 LiDAR sensor** mounted on a **Hunter 2.0 UGV (AgileX Robotics)** at the **University of Málaga**. The platform was manually driven along ~1 km trajectories inside the Computer Science building, acquiring **RGB images, IMU, GPS (when available), and LiDAR metadata**. Although raw point clouds are not included due to size constraints, they can be reconstructed using the official Ouster utilities, enabling reproducible 3D mapping.  

The dataset has been curated and made publicly available to support **benchmarking, validation, and further research** in SLAM, robot localization, and 3D perception.  

📎 [Access the dataset on Zenodo (DOI: 10.5281/zenodo.15301791)](https://doi.org/10.5281/zenodo.15301791)

---
## 🔮 Possible Future Implementations
The methodology is not limited to a single robot:  
- **Multi-robot fleets:** once an environment has been mapped, the **3D model can be reused**, allowing other robots to localize without remapping.  
- **Autonomous vehicles:** robust backup system in **GPS-denied zones**, especially relevant for drones or ground robots operating in altitude-sensitive tasks.  
- **High-altitude navigation:** precise localization in vertical structures (e.g., tall buildings, towers, industrial plants) where GPS becomes unreliable.  
- **Rescue robotics & industrial inspection:** reliable navigation in collapsed structures, underground tunnels, or multi-level environments.  
- **Smart infrastructure:** integration with digital twins and urban planning systems.  

---

## 📂 Repository Structure
```bash
TFG/
 ├── CODES/     → Source code (ICP, Fast-LIO2, ROS2 nodes, processing tools)  
 ├── MAPS/      → Generated point-cloud post-processed maps  
 ├── DOCS/      → Presentation, videos, annexes
 ├── RESULTS/   → Detailed Results, Graphics & Conclusions
 └── README.md
