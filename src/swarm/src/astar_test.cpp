#include <ros/ros.h>
#include <swarm/AStarReq.h>
#include <swarm/Pair.h>
#include <array>

int main(int argc, char **argv)
{
    //Init ROS related stuff
    ros::init(argc,argv,"astar_test");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<swarm::AStarReq>("/astar/calc");

    std::array<std::array<int, 20>, 20> grid{
		{ { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }},
		  { { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }}}
	};
    swarm::AStarReq srv;
    srv.request.start.first = 0;
    srv.request.start.second = 0;
    srv.request.destination.first = 19;
    srv.request.destination.second = 19;
    
    for(int i = 0;i < 20;i++){
        for(int j = 0;j < 20;j++){
            printf("%d",grid[i][j]);
        }
        printf("\n");
    }
    std::array<int,400> flat;
    for(int i = 0;i < 20;i++){
        for(int j = 0;j < 20;j++){
            srv.request.flattenedGrid[i*20+j] = grid[i][j];
        }
    }
    if(client.call(srv)){
        if(srv.response.pathFound){
            for (swarm::Pair &p : srv.response.path) {
                grid[p.first][p.second] = 2;
                printf("-> (%d,%d) ", p.first, p.second);
            }
            printf("\n");
            for(int i = 0;i < 20;i++){
                for(int j = 0;j < 20;j++){
                    printf("%d",grid[i][j]);
                }
                printf("\n");
            }
        }else{
            ROS_INFO("Path Not Found");
        }
    }else{
        ROS_INFO("Service couldn't get called");
    }
    return 0;
}