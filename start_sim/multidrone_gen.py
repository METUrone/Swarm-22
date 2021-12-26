import sys 

begin = """<?xml version="1.0"?>
<launch>
    <!-- MAVROS posix SITL environment launch script -->
    <!-- launches Gazebo environment and 2x: MAVROS, PX4 SITL, and spawns vehicle -->
    <!-- vehicle model and world -->
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="iris"/>
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/empty.world"/>
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
    </include>
"""

end = """
</launch>
"""

str = """
<!-- UAV{}-->
    <group ns="uav{}">
        <!-- MAVROS and vehicle configs -->
        <arg name="ID" value="{}"/>
        <arg name="fcu_url" default="udp://:{}@localhost:{}"/>
        <!-- PX4 SITL and vehicle spawn -->
        <include file="$(find px4)/launch/single_vehicle_spawn.launch">
            <arg name="x" value="{}"/>
            <arg name="y" value="{}"/>
            <arg name="z" value="{}"/>
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

"""

n = int(sys.argv[1])

str2=""

for i in range(n):
    global str2, str
    str2 = str2 + str.format(i, i, i, i+14540, i+14580, i, i, i, i+14560, i+4560)

result = begin + str2 + end

print(result)

try:
    f = open("multi_uav_mavros_sitl.launch", "w")
    f.write(result)
    f.close()
    print("success")
except(e):
    print(e)
