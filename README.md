# Final Degree Project (Bachelor’s Thesis)  
## 3D Outdoor Mapping with a Robot-Mounted LiDAR  

**Languages:** [English](#english-version) | [Español](#versión-en-español)  

---

## English Version

**Official title in Spanish:**  
*Construcción de mapas de exteriores con un LiDAR 3D embarcado en un robot móvil*  

[![GitHub license](https://img.shields.io/github/license/PacoAnaya/TFG)](LICENSE) 
![GitHub top language](https://img.shields.io/github/languages/top/PacoAnaya/TFG) 
![GitHub repo size](https://img.shields.io/github/repo-size/PacoAnaya/TFG)

---

### 📖 Overview
This project consists of the development of a **LiDAR-based mapping and localization pipeline** implemented as my **Final Degree Project (Bachelor’s Thesis)** at the *University of Málaga (ETSII)*.  

The system focuses on generating **dense 3D point-cloud maps** of outdoor environments using a robot-mounted LiDAR sensor. The mapping process integrates algorithms such as **Iterative Closest Point (ICP)** for scan matching and alignment, and leverages common robotics frameworks (**ROS**, **PCL**) to achieve scalable and accurate results.  

The generated maps are **georeferenced**, which allows them to be exploited in **outdoor, semi-outdoor, and indoor environments**, even in **GPS-denied scenarios**.  

---

### ✅ Results
- High-resolution **3D point-cloud maps** with detailed geometric information.  
- Robust scan registration using **ICP** algorithms.  
- Efficient localization with **low computational overhead**.  
- Demonstrated applicability to **autonomous navigation** tasks in complex environments.  

---

### 🛑 Problem Addressed
Traditional **GPS-based localization** is unreliable or impossible in many robotics applications:  
- Urban canyons (multipath, low satellite visibility).  
- Indoors or semi-indoor areas (garages, tunnels).  
- Safety- or privacy-constrained zones where GPS use is restricted.  

This implementation provides a **LiDAR-based alternative**, enabling precise **absolute localization** where GPS is unavailable.  

---

### 🔮 Possible Future Implementations
The methodology is not limited to a single robot:  
- **Multi-robot fleets:** once a building or environment has been mapped, the **3D model can be reused**, enabling any robot to localize without remapping.  
- **Autonomous vehicle navigation:** robust backup system in **GPS-denied zones**.  
- **Rescue robotics & industrial inspection:** reliable navigation in collapsed structures or mines.  
- **Smart infrastructure:** integration with digital twins and urban planning.  

---

### 🖼️ Screenshots
<p align="center">
  <img src="https://github.com/PacoAnaya/TFG/assets/145780472/5b7bc1b5-85b5-442b-8fcf-9b03f0569dfe" width="700">
</p>

*Point cloud overlaid with 3D building models and aerial photographs.*  
*Mapped building: ETSII – University of Málaga (Campus de Teatinos).*  

---

### ⚙️ How to Use
Clone the repository:  
```bash
git clone https://github.com/PacoAnaya/TFG.git
cd TFG
