### Parameter Tuning 

For LoD2 building reconstruction on custom datasets, adjustments to specific parameters may be necessary. Three primary parameters—**Number** and  **Density** —are key to optimizing the reconstruction process for varying dataset characteristics.

#### 1. **Number** (Plane Detection)
This parameter defines the granularity of plane detection:
- **Lower values** yield more detailed plane detection, increasing computation time.
- **Higher values** simplify output by reducing detected detail, especially beneficial if excessive candidate faces are generated.

**Adjustment Guidelines**:
- Increase **Number** if an overabundance of candidate faces is detected.
- Decrease **Number** to capture finer details in smaller structures.

#### 2. **Density** (Height Map Generation)
**Density** affects the resolution of the height map and the precision of line detection in footprint polygons. Recommended values range between \[0.15, 0.3\]:
- For **dense point clouds**, a lower **Density** improves resolution in the height map and enhances line detection accuracy.
- For **sparse point clouds**, a higher **Density** compensates for lower point availability.

**Adjustment Guidelines**:
- If an excessive number of lines are detected in the footprint polygon, increase **Density** to streamline results.

[//]: # (#### 3. **Ground** &#40;Footprint Generation&#41;)

[//]: # (The **Ground** parameter defines the Z-value &#40;height&#41; of the footprint polygon, applicable only if no pre-existing footprint data is available.)

[//]: # ()
[//]: # (**Adjustment Guidelines**:)

[//]: # (- If you have access to the complete raw point cloud data, including both the roof and ground points, it would be advisable to directly inspect the data and set the ground height based on the actual ground points.)

[//]: # (- If no ground-level data is available, an alternative approach is to experiment with different values. You can start with value of 0.0 and adjust as needed.)


#### Parameter Adjustment in GUI Mode
In GUI mode, parameters can be adjusted interactively:
- Click **Manual** and adjust **Number** and **Density**.


<p align="center"> 
     <img src="./images/GUI_mode.png" width="300"> 
</p>

#### Parameter Adjustment in CLI Mode
In CLI mode, parameters can be manually modified within the source code as follows:
- **Number**: Modify in [code/method/method_global.cpp, Line11](https://github.com/tudelft3d/City3D/tree/main/code/method/method_global.cpp#L11).
- **Density**: Modify in [code/method/method_global.cpp, Line12](https://github.com/tudelft3d/City3D/tree/main/code/method/method_global.cpp#L12).
