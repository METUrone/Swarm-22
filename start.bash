
source devel/setup.bash

commandd="rosrun swarm takeoff.py 0 "
echo "$1"
END=$1
for((i=1;i<END;i++))
do
    echo "$i"
    commandd+="& rosrun swarm takeoff.py "$i" "
done
echo "$commandd"
eval "$commandd"