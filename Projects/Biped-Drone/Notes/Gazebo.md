---
tags:
  - projects
  - biped-drone
  - notes
topic: Gazebo
date_created: 2025-05-01
date_modified: 2025-05-01
backlinks:
---
To define a model is used an *SDF* file (Simulation Description Format), an XML format that describes objects and environments for robot simulators, visualization, and control.
## Defining a model
>_Essential definition of a model_

*Important*: Dimensions are in meters and angles are in radians.
```
<model name="model_name" canonical_link="canonical_link"
	<pose relative_to="element">x y z r p y</pose>
	<link name="link_name">
		<pose relative_to="model">
		<inertial>
			<mass><!--mass in kg>
			<inertia>
				<ixx> </ixx>
				<ixy> </ixy>
				<ixz> </ixz>
				<iyy> </iyy>
				<iyz> </iyz>
				<izz> </izz>
			</inertia>
		</inertial>
		<visual name="visual_name">
			<geometry>
				<!--geometry type-->
					<!--geometry parameter--> </!--geometry parameter-->
				</!--geometry type-->
			</geometry>
			<material>
				<ambient>r g b a</ambient>
				<diffuse>r g b a</ambient>
				<specular>r g b a</ambient>
			</material>
		</visual>
		<collision>
			<geometry>
				<!--geometry type-->
					<!--geometry parameter--> </!--geometry parameter-->
				</!--geometry type-->
			</geometry>
		</collision>
	</link>
</model>
```

*Geometry types*: `box` (x, y, z), `cylinder` (radius, length), `sphere` (radius), `plane` (size, normal), `image` (granularity, height, scale, uri, treshold), `mesh` (scale, uri $\rightarrow$ `stl`, optional: submesh, center, name),  `polyline` (points, height).
*Inertial calculator*: [https://amesweb.info/inertia/mass-moment-of-inertia-calculator.aspx]()
Please note that `links` are each component that are part of the model.
### Defining a frame
Inside the model:
```
<frame name="frame_name" attached_to="canonical_link">
	<pose>x y z r p y</pose>
</frame>
```
### Establishing joints
```
<joint name="joint_name" type="joint_type">
	<pose relative_to="element"/>
	<parent> </parent>
	<child> </child>
	<!--properties of the joint-->
</joint>
```

*Joint types*: `revolute` (upper/lower limits), `continuous`, `prismatic` (slides along the axis), `fixed` (no movement), `floating`, `planar` (motion in a plane perpendicular to the axis).
## Robot movement
To be able to move the robot, it is necessary to specify the topic:
```
<plugin
    filename="libignition-gazebo-diff-drive-system.so"
    name="ignition::gazebo::systems::--drive-type--">
    ---
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
```
**To move from terminal**: `gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"`
## Defining a world
Every SDF world should start with the tags:
```
<?xml version="1.0" ?>
<sdf version="1.8">
    <world name="world_demo">
    ...
    ...
    </world>
</sdf>
```
Setup the `physics`:

```
<physics name="1ms" type="ignored">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
</physics>
```
### Plugins
>_For model control_

`Physics`plugin:
```
<plugin
    filename="gz-sim-physics-system"
    name="gz::sim::systems::Physics">
</plugin>
```
`UserCommands`plugin:
```
<plugin
    filename="gz-sim-user-commands-system"
    name="gz::sim::systems::UserCommands">
</plugin>
```
`SceneBroadcaster` plugin:
```
<plugin
    filename="gz-sim-scene-broadcaster-system"
    name="gz::sim::systems::SceneBroadcaster">
</plugin>
```
>_For world control_

`World Control` plugin:
```
<!-- World control -->
<plugin filename="WorldControl" name="World control">
    <gz-gui>
        <title>World control</title>
        <property type="bool" key="showTitleBar">false</property>
        <property type="bool" key="resizable">false</property>
        <property type="double" key="height">72</property>
        <property type="double" key="width">121</property>
        <property type="double" key="z">1</property>

        <property type="string" key="state">floating</property>
        <anchors target="3D View">
        <line own="left" target="left"/>
        <line own="bottom" target="bottom"/>
        </anchors>
    </gz-gui>

    <play_pause>true</play_pause>
    <step>true</step>
    <start_paused>true</start_paused>
    <service>/world/world_demo/control</service>
    <stats_topic>/world/world_demo/stats</stats_topic>
</plugin>
```
`World Stats` plugin:
```
<!-- World statistics -->
<plugin filename="WorldStats" name="World stats">
    <gz-gui>
        <title>World stats</title>
        <property type="bool" key="showTitleBar">false</property>
        <property type="bool" key="resizable">false</property>
        <property type="double" key="height">110</property>
        <property type="double" key="width">290</property>
        <property type="double" key="z">1</property>

        <property type="string" key="state">floating</property>
        <anchors target="3D View">
        <line own="right" target="right"/>
        <line own="bottom" target="bottom"/>
        </anchors>
    </gz-gui>

    <sim_time>true</sim_time>
    <real_time>true</real_time>
    <real_time_factor>true</real_time_factor>
    <iterations>true</iterations>
    <topic>/world/world_demo/stats</topic>

</plugin>
```
`Entity tree`plugin:
```
<!-- Entity tree -->
<plugin filename="EntityTree" name="Entity tree">
</plugin>
```
>_Sensor plugins_
>_Note:_ Use with `Physics` and `SceneBroadcaster` plugin

`imu sensor`plugin:

```
<plugin filename="gz-sim-imu-system"
        name="gz::sim::systems::Imu">
</plugin>
```
```  
<!--Add the sensor to the robot-->
<sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>1</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
</sensor>
```
`contact sensor` plugin:
```
<plugin filename="gz-sim-contact-system"
        name="gz::sim::systems::Contact">
</plugin>
```
`lidar sensor`plugin:
```
<frame name="lidar_frame" attached_to='chassis'>
    <pose>0.8 0 0.5 0 0 0</pose>
</frame>
```
```
<plugin
  filename="gz-sim-sensors-system"
  name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```
 ```
 <sensor name='gpu_lidar' type='gpu_lidar'>"
    <pose relative_to='lidar_frame'>0 0 0 0 0 0</pose>
    <topic>lidar</topic>
    <update_rate>10</update_rate>
    <ray>
        <scan>
            <horizontal>
                <samples>640</samples>
                <resolution>1</resolution>
                <min_angle>-1.396263</min_angle>
                <max_angle>1.396263</max_angle>
            </horizontal>
            <vertical>
                <samples>1</samples>
                <resolution>0.01</resolution>
                <min_angle>0</min_angle>
                <max_angle>0</max_angle>
            </vertical>
        </scan>
        <range>
            <min>0.08</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
        </range>
    </ray>
    <always_on>1</always_on>
    <visualize>true</visualize>
</sensor>
```


### GUI
Under the `<gui>` tab we should specify anything related to the GUI.
```
<gui fullscreen="0">
    ...
    ...
</gui>
```
>_For example:
```
<!-- 3D scene -->
<plugin filename="GzSceneManager" name="Scene Manager">
  <gz-gui>
    <property key="resizable" type="bool">false</property>
    <property key="width" type="double">5</property>
    <property key="height" type="double">5</property>
    <property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property>
  </gz-gui>
</plugin>
```
*3D World displaying*: `MinimalScene` and `GzSceneManager`.
*Rendering engine*: `ogre`or `ogre2`.
Setup the `light`:
```
<light type="directional" name="sun">
    <cast_shadows>true</cast_shadows>
    <pose>0 0 10 0 0 0</pose>
    <diffuse>0.8 0.8 0.8 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
    </attenuation>
    <direction>-0.5 0.1 -0.9</direction>
</light>
```
## Including models
To include a model inside the `URI`:
```
<include>
    <uri>
    <!--link to the model-->
    </uri>
</include>
```
*Model link*: 
- Link: `https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke`
- File: `model://Coke`
>_If want to spawn multiple_
```
<include>
	<name>Coke0</name>
	<pose>0 0 0 0 0 0</pose>
	<uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke</uri>
</include>
```
```
<include>
	<name>Coke1</name>
	<pose>0 0.1 0 0 0 0</pose>
	<uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke</uri>
</include>
```



