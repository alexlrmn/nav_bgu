# Navigation    

Table of contents
=================

* [Gmapping](#gmapping)
  * [Editing a map](#editing-a-map)
* [Rtabmap](#rtabmap)
  * [Nodes](#nodes)
  * [Viewing the databases](#viewing-the-databases)
* [Octopmap](#octomap)

### [Gmapping]

The Gmapping package is the basic and most maintained package for creating a 2D map. It uses the LaserScan message that the Lidar laser is publishing and the information about the Odometry the come from the wheel encoders to create a map of the environment.

An example of a launch file that uses gmapping is located in the launch folder under the name of: 
> mapping_armadillo.launch

To save a map open a new terminal and enter the next command:

```
rosrun map_server map_saver -f /complete_path/file_name
```
You can navigate to the folder you want to save the file in a specify just the file name.

#### Editing a map
Gimp is a simple and very handy image editing application. It is very important to save the edited image in the same format as it came. You can do that by right clicking on the image as choose the option "export as", and choose the .pgm file type.
#### Issues to take care of:
*  While driving in places where there are no features (for example hallways). The robot is having trouble localizing itself, it seems that the laser messages stay very similar and the package doesn't take into account the Odomtery at this kind of situation. <br/>
Possible solution: Add temporary features like boxes and chairs for the laser scan, and then edit them out of the map. <br />

### [Rtabmap]
Rtabmap is a 3D mapping a localization package that fuses the LaserScan messages, Odometry information and the color and depth images from the camera to create a Database that can be used for navigation. The database that is created contains a graph which is a 2D map and a set of color and depth images. 
 
* The 2D map can be used separately for navigation with another package, it can be obtained the same way that was mentioned  in Gmapping. 
* The set of color and depth images are used for the localization. The rtabmap package selects images by the number of important features in the image (important features can be table edges, labels on a wall or even cracks in the floor/wall), new images received by the camera are analysed and the features extracted are compared to the images in the database to locate a loop closure to optimize the map.

#### Nodes
The rtabmap package is providing two nodes, rtabmap and rgbd_odometry. 

* The rtabmap node is the node responsible for mapping and localization.

* The rgbd_odometry node is used to replace the Odometry information that is received by the the wheel encoders with information calculated using the PointCloud2 message which is message created from the images received by the camera. 

For both nodes there's a list of parameter that can be tuned to improve functionality. You can see the entire list of parameters and their description in the launch/rtabmap/ forlder under the names of params_rtabmap for the rtabmap node and paramas_rtabmap_odom for the rgbd_rtabmap node.

An sample of launch files for rtabmap localization and mapping can be found at the launch/rtabmap folder under the names:
> rtabmap.launch
> rtabmap_mapping.launch
> rtabmap_nav.launch
tand alone tand alone 
The file rtabmap.launch is an example of all the configuration that are required to launch the nodes (camera topics, scan topic, etc.) . The mapping and navigation files just call the rtabmap.launch file with different modes: localization/mapping which are determined by a parameter. It is important to remember that while running rtabmap in mapping mode, a database is created and saved is the same directory as specified by the parameter "database path", if a database already exists in the folder specified, rtabmap will override it.  The default path is:
>  ~/.ros/rtabmap.db

#### Viewing the databases
Rtabmap provides two ways to view the databases created.
```
rtabmap ~/.ros/rtabmap.db
rtabmap-databaseViewer ~/.ros/rtabmap.db
```
Using the rtabmap-databaseViewer provides some ways to manipulate the database (still needs to be looked into).

### [Octomap]
A 3D mapping package that mainly uses the images provided by a camera to create a 3D point cloud map. The map can be projected to a 2D map which will include floating obstacles as walls.

A sample launch file can be found at the launch/octomap folder under the name:
> octomap.launch

It seems that while mapping octomap uses only the point cloud to maintain the robots location so it could have high error rate. Not suggested to use a standalone for mapping. Rtabmap is also using octomap.


[Gmapping]: <http://wiki.ros.org/gmapping>
[Rtabmap]: <https://introlab.github.io/rtabmap/>
[Octomap]: <http://wiki.ros.org/octomap>

