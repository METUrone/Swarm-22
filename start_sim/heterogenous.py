from re import L
import sys

iris_count = 0
turtlebot_count = 0
vtol_count = 0


if len(sys.argv) == 2:
    iris_count = sys.argv[1]
elif len(sys.argv) > 2 and len(sys.argv) % 2 != 0:
    for i in range(1, len(sys.argv)):
        
        if sys.argv[i] == "-i":
            i += 1
            iris_count = sys.argv[i]
        elif sys.argv[i] == "-t":
            i += 1
            turtlebot_count = sys.argv[i]
        elif sys.argv[i] == "-v":
            i += 1
            vtol_count = sys.argv[i]
else:
    print("INVALID ARGUMENT!")
    sys.exit()

print("iris: {}\nturtle: {}\nvtol: {}\n".format(iris_count, turtlebot_count, vtol_count))

tb_3_args = ""
for i in range(int(turtlebot_count)):
    tb_3_args +="""
    <arg name="{}_tb3"  default="tb3_{}"/>
    <arg name="{}_tb3_x_pos" default="{}"/>
    <arg name="{}_tb3_y_pos" default="{}"/>
    <arg name="{}_tb3_z_pos" default="0"/>
    <arg name="{}_tb3_yaw"   default="0"/>
    
    """.format(i,i,i,i*2+2,i,i*2,i,i)

iris = ""

for i in range(int(iris_count)):
    iris += """<!-- UAV{}-->
    <group ns="uav{}">
        <!-- MAVROS and vehicle configs -->
        <arg name="ID" value="{}"/>
        <arg name="fcu_url" default="udp://:{}@localhost:{}"/>
        <!-- PX4 SITL and vehicle spawn -->
        <include file="$(find px4)/launch/single_vehicle_spawn.launch">
            <arg name="x" value="{}"/>
            <arg name="y" value="{}"/>
            <arg name="z" value="0"/>
            <arg name="R" value="0"/>
            <arg name="P" value="0"/>
            <arg name="Y" value="0"/>
            <arg name="vehicle" value="$(arg vehicle)"/>
            <arg name="mavlink_udp_port" value="{}"/>
            <arg name="mavlink_tcp_port" value="{}"/>
            <arg name="ID" value="$(arg ID)"/>
        </include>
        <!-- MAVROS -->
        <include file="$(find mavros)/launch/px4.launch">
            <arg name="fcu_url" value="$(arg fcu_url)"/>
            <arg name="gcs_url" value=""/>
            <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
            <arg name="tgt_component" value="1"/>
        </include>
    </group>
    
    """.format(i,i,i,14540+i, 1580+i, i*2,i*2,14560+i, 4560+i)

vtol = ""

for i in range(int(iris_count), int(vtol_count)+int(iris_count)):
    vtol += """<!-- UAV{}-->
    <group ns="uav{}">
        <!-- MAVROS and vehicle configs -->
        <arg name="ID" value="{}"/>
        <arg name="fcu_url" default="udp://:{}@localhost:{}"/>
        <!-- PX4 SITL and vehicle spawn -->
        <include file="$(find px4)/launch/single_vehicle_spawn.launch">
            <arg name="x" value="{}"/>
            <arg name="y" value="{}"/>
            <arg name="z" value="0"/>
            <arg name="R" value="0"/>
            <arg name="P" value="0"/>
            <arg name="Y" value="0"/>
            <arg name="vehicle" value="$(arg vehicle2)"/>
            <arg name="mavlink_udp_port" value="{}"/>
            <arg name="mavlink_tcp_port" value="{}"/>
            <arg name="ID" value="$(arg ID)"/>
        </include>
        <!-- MAVROS -->
        <include file="$(find mavros)/launch/px4.launch">
            <arg name="fcu_url" value="$(arg fcu_url)"/>
            <arg name="gcs_url" value=""/>
            <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
            <arg name="tgt_component" value="1"/>
        </include>
    </group>
    
    """.format(i,i,i,14540+i, 1580+i, (i-int(iris_count))*2-2,(i-int(iris_count))*2,14560+i, 4560+i)

turtlebot = ""

for i in range(int(turtlebot_count)):
    turtlebot += """
    <group ns = "$(arg {}_tb3)">
    <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />

    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">
      <param name="publish_frequency" type="double" value="50.0" />
      <param name="tf_prefix" value="$(arg {}_tb3)" />
    </node>

    <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -model $(arg {}_tb3) -x $(arg {}_tb3_x_pos) -y $(arg {}_tb3_y_pos) -z $(arg {}_tb3_z_pos) -Y $(arg {}_tb3_yaw) -param robot_description" />
  </group>
  
  """.format(i,i,i,i,i,i,i)

str = """<?xml version="1.0"?>
<launch>

    <!-- MAVROS posix SITL environment launch script -->
    <!-- launches Gazebo environment and 2x: MAVROS, PX4 SITL, and spawns vehicle -->
    <!-- vehicle model and world -->
    
    <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
    {}
    
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="iris"/>
    <arg name="vehicle2" default="standard_vtol"/>
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/mcmillian_airfield.world"/>
    <!-- gazebo configs -->
    <arg name="gui" default="true"/>
    <arg name="debug" default="false"/>
    <arg name="verbose" default="false"/>
    <arg name="paused" default="false"/>
    <!-- Gazebo sim -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="gui" value="$(arg gui)"/>
        <arg name="world_name" value="$(arg world)"/>
        <arg name="debug" value="$(arg debug)"/>
        <arg name="verbose" value="$(arg verbose)"/>
        <arg name="paused" value="$(arg paused)"/>
        <arg name="use_sim_time" value="true"/>
        <arg name="headless" value="false"/>
    </include>

    {}

    {}
    
    {}


</launch>

""".format(tb_3_args, iris, vtol, turtlebot)

   
f = open("../../PX4-Autopilot/launch/multi_uav_mavros_sitl.launch", "w")
f.write(str)
f.close()
print("success")