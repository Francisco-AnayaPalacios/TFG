# 📊 Experimental Results & Analysis  

This section compiles the **main experimental results, graphs, and conclusions** obtained during the project *Construcción de mapas de exteriores con un LiDAR 3D embarcado en un robot móvil*.  

---

## 1️⃣ Mapping Experiments  

Two mapping sessions were carried out at the **ETSII Faculty, University of Málaga**:  

- **Mapping 1 – Outdoor courtyard**  
  - Distance traveled: **90.80 m**  
  - Generated a dense point cloud of the exterior patio.  

- **Mapping 2 – Entire building**  
  - Distance traveled: **959.18 m**  
  - Complete 3D reconstruction of the faculty’s building.  

Both datasets were later **post-processed** (combination, filtering, voxelization, and georeferencing).  

<p align="center">
  <img width="1896" height="861" alt="image" src="https://github.com/user-attachments/assets/9c7698a0-4207-4db5-b8f7-1273498d00f2" width="700">
</p>  

---

## 2️⃣ Post-Processing Results  

- **Combination:** merged partial maps into large-scale environments.  
- **Filtering:** removed outliers and enabled sectional visualizations at specific heights.  
- **Voxelization:** reduced dataset size while preserving geometric detail.  
- **Georeferencing:** aligned point clouds with **EPSG:4326 → EPSG:25830**, enabling integration with **ArcGIS**.  

📌 Example: Mapa 2 compared against official 3D building models, showing high spatial consistency.  

<p align="center">
  <img width="1040" height="519" alt="image" src="https://github.com/user-attachments/assets/6beec650-55b1-4b79-939a-40ebf2aeb7ca" width="700">
</p> 

---

## 3️⃣ Localization Experiments  

The generated maps were validated in **localization tasks** using **Iterative Closest Point (ICP)**.  

### ICP with 10 iterations
- Produced a stable but less accurate registration.  
- Comparison between **Ground Truth (Fast-LIO2)**, **Noisy Odometry**, and **ICP Estimated pose** showed partial correction of errors.  

<p align="center">
  <img width="1978" height="927" alt="image" src="https://github.com/user-attachments/assets/cd4f01d4-3e76-4c36-a913-6f773f4bf678" width="700"/>
</p>

<p align="center">
  <img width="2051" height="811" alt="image" src="https://github.com/user-attachments/assets/4216275a-99b0-4363-ae7e-55ac7f563d2a" width="700"/>
</p>

### ICP with 1000 iterations
- Significantly improved trajectory alignment.  
- RMSE error decreased as iteration count increased.  

<p align="center">
  <img width="2112" height="1004" alt="image" src="https://github.com/user-attachments/assets/455ea990-2a19-47fe-a8b9-b54babb20560" width="700"/>
</p>

<p align="center">
  <img width="2053" height="839" alt="image" src="https://github.com/user-attachments/assets/1746b0c2-93fe-4553-bb77-282688da936c" width="700"/>
</p>

### Effect of voxelization size
- Smaller voxel sizes → higher accuracy but longer computation time.  
- Larger voxel sizes → reduced computation time but loss of detail.  

<p align="center">
  <img width="2361" height="1021" alt="image" src="https://github.com/user-attachments/assets/4cbb6b76-b6ba-4cb0-bb97-418222faaad1" width="700">
</p>  

---

## 5️⃣ Conclusions  

- ✅ Achieved construction of **dense, georeferenced 3D point-cloud maps**.  
- ✅ Demonstrated applicability for **localization** using ICP.  
- ✅ Validated accuracy and robustness compared to GPS, especially in altitude-sensitive navigation.  

### 🔮 Future Work  
- Real-time (online) map generation.  
- Integration of additional data (color, texture).  
- Deployment in **autonomous navigation pipelines**.  

---

📌 For complete details, see the **main thesis document** in `DOCUMENTATION/` and video demonstrations in the [YouTube channel](https://www.youtube.com/@FranciscoAnaya-mi3le).  
