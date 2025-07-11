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
