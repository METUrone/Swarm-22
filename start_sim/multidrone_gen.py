import sys 

try:
    if len(sys.argv) == 2:
        vehicle = "iris"
    elif sys.argv[2] == "-i":
        vehicle = "iris"
    elif sys.argv[2] == "-v":
        vehicle = "standard_vtol"
except IndexError:
    ()

begin = """<?xml version="1.0"?>
<launch>
    <!-- MAVROS posix SITL environment launch script -->
    <!-- launches Gazebo environment and 2x: MAVROS, PX4 SITL, and spawns vehicle -->
    <!-- vehicle model and world -->
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="{}"/>
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
    </include>
""".format("standard_vtol")

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

"""

n = int(sys.argv[1])

str2=""

for i in range(n):

    str2 = str2 + str.format(i, i, i, i+14540, i+14580, i*2, i*2, i+14560, i+4560)

result = begin + str2 + end

print(result)


f = open("../../PX4-Autopilot/launch/multi_uav_mavros_sitl.launch", "w")
f.write(result)
f.close()
print("success")

