```
cd ${YOUR_WORKSPACE_PATH}/src
git clone git@github.com:HKUST-Aerial-Robotics/FUEL.git
cd ../ 
catkin_make
```

After compilation you can start a sample exploration demo. Firstly run ```Rviz``` for visualization: 

```
source devel/setup.bash && roslaunch exploration_manager rviz.launch
```
then run the simulation (run in a new terminals): 
```
source devel/setup.bash && roslaunch exploration_manager exploration.launch
```

By default you can see an office-like environment. Trigger the quadrotor to start exploration by the ```2D Nav Goal``` tool in ```Rviz```. A sample is shown below, where unexplored structures are shown in grey and explored ones are shown in colorful voxels. The FoV and trajectories of the quadrotor are also displayed.

<!-- You will find a cluttered scene to be explored (20m x 12m x 2m) and the drone . You can trigger the exploration to start by  A sample simulation is shown in the figure. The unknown obstacles are shown in grey, while the frontiers are shown as colorful voxels. The planned and executed trajectories are also displayed. -->

 <p id="demo1" align="center">
  <img src="files/office.gif" width = "600" height = "325"/>
 </p>


## Exploring Different Environments

The exploration environments in our simulator are represented by [.pcd files](https://pointclouds.org/documentation/tutorials/pcd_file_format.html).
We provide several sample environments, which can be selected in [simulator.xml](fuel_planner/exploration_manager/launch/simulator.xml):


```xml
  <!-- Change office.pcd to specify the exploration environment -->
  <!-- We provide office.pcd, office2.pcd, office3.pcd and pillar.pcd in this repo -->
  <node pkg ="map_generator" name ="map_pub" type ="map_pub" output = "screen" args="$(find map_generator)/resource/office.pcd"/>    
```

Other examples are listed below.

_office2.pcd_:
<p id="demo2" align="center">
<img src="files/office2.gif" width = "600" height = "325"/>
</p>

_office3.pcd_:
<p id="demo3" align="center">
<img src="files/office3.gif" width = "600" height = "325"/>
</p>

_pillar.pcd_:
<p id="demo4" align="center">
<img src="files/pillar.gif" width = "320" height = "325"/>
</p>

If you want to use your own environments, simply place the .pcd files in [map_generator/resource](uav_simulator/map_generator/resource), and follow the comments above to specify it.
You may also need to change the bounding box of explored space in [exploration.launch](https://github.com/HKUST-Aerial-Robotics/FUEL/blob/main/fuel_planner/exploration_manager/launch/exploration.launch):

```xml
    <arg name="box_min_x" value="-10.0"/>
    <arg name="box_min_y" value="-15.0"/>
    <arg name="box_min_z" value=" 0.0"/>
    <arg name="box_max_x" value="10.0"/>
    <arg name="box_max_y" value="15.0"/>
    <arg name="box_max_z" value=" 2.0"/>
```
Normally, a file named __tmp.pcd__ will be saved at ```~/```. You may replace ```~/``` with any locations you want.
Lastly, you can use this file for exploration, as mentioned [here](#exploring-different-environments).

## Acknowledgements
  We use **NLopt** for non-linear optimization and use **LKH** for travelling salesman problem.
