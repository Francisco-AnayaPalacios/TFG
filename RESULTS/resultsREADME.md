# 📊 Experimental Results & Analysis

This document compiles the main **results, graphs, and conclusions** obtained during the development of the project:

**Official project title (UMA):**  
*Construcción de mapas de exteriores con un LiDAR 3D embarcado en un robot móvil*  

---

## 1️⃣ Objectives of the Experiments
- Validate the performance of the **ICP-based localization**.  
- Evaluate the accuracy of **Fast-LIO2 (LiDAR-Inertial Odometry)**.  
- Compare **LiDAR-based localization** against traditional **GPS-based systems**.  
- Assess computational load and feasibility for **real-time navigation**.  

---

## 2️⃣ Metrics Used
- **RMSE (Root Mean Square Error):** precision of point registration.  
- **Fitness score (ICP):** quality of cloud alignment.  
- **Number of iterations (ICP):** convergence efficiency.  
- **Computation time:** average time per scan matching.  
- **Map density (points/m²):** spatial detail in generated environments.  

---

## 3️⃣ Key Graphs & Results

### ICP Convergence
<p align="center">
  <img src="./images/icp_convergence.png" width="600">
</p>

- ICP converges in **X iterations on average**, ensuring stable scan matching.  
- RMSE consistently below **Y cm**, validating high accuracy.  

---

### Comparison: LiDAR vs GPS
<p align="center">
  <img src="./images/lidar_vs_gps.png" width="600">
</p>

- **GPS:** error grows significantly in the **Z (altitude)** axis.  
- **LiDAR + ICP/Fast-LIO2:** keeps uniform precision across **X, Y, and Z**, critical for **high-altitude navigation**.  

---

### Map Density
<p align="center">
  <img src="./images/map_density.png" width="600">
</p>

- Generated point clouds contain **N points/m²**.  
- This density allows precise localization with low computational overhead.  

---

## 4️⃣ Discussion
- **High-altitude navigation:** LiDAR-based approach outperforms GPS, which shows vertical errors of up to several meters.  
- **Computational efficiency:** ICP and Fast-LIO2 allow **real-time execution** with modest hardware.  
- **Robustness:** works reliably in **urban canyons, tunnels, or semi-indoor areas** where GPS fails.  

---

## 5️⃣ Conclusions
- Developed pipeline achieves **highly accurate 3D mapping** suitable for autonomous navigation.  
- Demonstrated a **practical alternative to GPS**, especially in **altitude-sensitive scenarios**.  
- Provides a foundation for **multi-robot fleets** and **smart infrastructure** integration.  

---


📌 For additional details, refer to the full thesis document and annexes in DOCUMENTATION.
