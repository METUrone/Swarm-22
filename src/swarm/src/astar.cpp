// A C++ Program to implement A* Search Algorithm
#include "ros/ros.h"
#include "math.h"
#include <array>
#include <vector>
#include <chrono>
#include <cstring>
#include <iostream>
#include <queue>
#include <set>
#include <stack>
#include <tuple>
#include <swarm/AStarReq.h>
using namespace std;

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;
// Creating a shortcut for tuple<int, int, int> type
typedef tuple<double, int, int> Tuple;

// A structure to hold the necessary parameters
struct cell {
	// Row and Column index of its parent
	Pair parent;
	// f = g + h
	double f, g, h;
	cell()
		: parent(-1, -1)
		, f(-1)
		, g(-1)
		, h(-1)
	{
	}
};

// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
template <size_t ROW, size_t COL>
bool isValid(const array<array<int, COL>, ROW>& grid,
			const Pair& point)
{ // Returns true if row number and column number is in
// range
	if (ROW > 0 && COL > 0)
		return (point.first >= 0) && (point.first < ROW)
			&& (point.second >= 0)
			&& (point.second < COL);

	return false;
}

// A Utility Function to check whether the given cell is
// blocked or not
template <size_t ROW, size_t COL>
bool isUnBlocked(const array<array<int, COL>, ROW>& grid,
				const Pair& point)
{
	// Returns true if the cell is not blocked else false
	return isValid(grid, point)
		&& grid[point.first][point.second] == 1;
}

// A Utility Function to check whether destination cell has
// been reached or not
bool isDestination(const Pair& position, const Pair& dest)
{
	return position == dest;
}

// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(const Pair& src, const Pair& dest)
{
	// h is estimated with the two points distance formula
	return sqrt(pow((src.first - dest.first), 2.0)
				+ pow((src.second - dest.second), 2.0));
}

swarm::Pair pair2msg(const Pair &p){
    swarm::Pair msg;
    msg.first = p.first;
    msg.second = p.second;
    return msg;
}

// A Utility Function to trace the path from the source to
// destination
template <size_t ROW, size_t COL>
vector<swarm::Pair> tracePath(
	const array<array<cell, COL>, ROW>& cellDetails,
	const Pair& dest)
{
	vector<swarm::Pair> Path;
    Path.reserve(30);

	Path.push_back(pair2msg(dest));
	int row = dest.first;
	int col = dest.second;
	Pair next_node = cellDetails[row][col].parent;
	do {
		next_node = cellDetails[row][col].parent;
		Path.push_back(pair2msg(next_node));
		row = next_node.first;
		col = next_node.second;
	} while (cellDetails[row][col].parent != next_node);
    return Path;
}

// A Function to find the shortest path between a given
// source cell to a destination cell according to A* Search
// Algorithm
template <size_t ROW, size_t COL>
swarm::AStarReq::Response aStarSearch(const array<array<int, COL>, ROW>& grid,
				const Pair& src, const Pair& dest)
{
    swarm::AStarReq::Response res;
    res.pathFound = false;
	// If the source is out of range
	if (!isValid(grid, src)) {
		ROS_WARN("Source is invalid\n");
		return res;
	}

	// If the destination is out of range
	if (!isValid(grid, dest)) {
		ROS_WARN("Destination is invalid\n");
		return res;
	}

	// Either the source or the destination is blocked
	if (!isUnBlocked(grid, src)
		|| !isUnBlocked(grid, dest)) {
		ROS_WARN("Source or the destination is blocked\n");
		return res;
	}

	// If the destination cell is the same as source cell
	if (isDestination(src, dest)) {
		ROS_WARN("We are already at the destination\n");
		return res;
	}

	// Create a closed list and initialise it to false which
	// means that no cell has been included yet This closed
	// list is implemented as a boolean 2D array
	bool closedList[ROW][COL];
	memset(closedList, false, sizeof(closedList));

	// Declare a 2D array of structure to hold the details
	// of that cell
	array<array<cell, COL>, ROW> cellDetails;

	int i, j;
	// Initialising the parameters of the starting node
	i = src.first, j = src.second;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
	cellDetails[i][j].parent = { i, j };

	/*
	Create an open list having information as-
	<f, <i, j>>
	where f = g + h,
	and i, j are the row and column index of that cell
	Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	This open list is implemented as a set of tuple.*/
	std::priority_queue<Tuple, std::vector<Tuple>,
						std::greater<Tuple> >
		openList;

	// Put the starting cell on the open list and set its
	// 'f' as 0
	openList.emplace(0.0, i, j);

	// We set this boolean value as false as initially
	// the destination is not reached.
	while (!openList.empty()) {
		const Tuple& p = openList.top();
		// Add this vertex to the closed list
		i = get<1>(p); // second element of tuple
		j = get<2>(p); // third element of tuple

		// Remove this vertex from the open list
		openList.pop();
		closedList[i][j] = true;
		/*
				Generating all the 8 successor of this cell
						N.W N N.E
						\ | /
						\ | /
						W----Cell----E
								/ | \
						/ | \
						S.W S S.E

				Cell-->Popped Cell (i, j)
				N --> North	 (i-1, j)
				S --> South	 (i+1, j)
				E --> East	 (i, j+1)
				W --> West		 (i, j-1)
				N.E--> North-East (i-1, j+1)
				N.W--> North-West (i-1, j-1)
				S.E--> South-East (i+1, j+1)
				S.W--> South-West (i+1, j-1)
		*/
		for (int add_x = -1; add_x <= 1; add_x++) {
			for (int add_y = -1; add_y <= 1; add_y++) {
				Pair neighbour(i + add_x, j + add_y);
				// Only process this cell if this is a valid
				// one
				if (isValid(grid, neighbour)) {
					// If the destination cell is the same
					// as the current successor
					if (isDestination(
							neighbour,
							dest)) { // Set the Parent of
									// the destination cell
						cellDetails[neighbour.first]
								[neighbour.second]
									.parent
							= { i, j };
						ROS_INFO("The destination cell is "
							"found\n");
						res.path = tracePath(cellDetails, dest);
                        res.pathFound = true;
						return res;
					}
					// If the successor is already on the
					// closed list or if it is blocked, then
					// ignore it. Else do the following
					else if (!closedList[neighbour.first]
										[neighbour.second]
							&& isUnBlocked(grid,
											neighbour)) {
						double gNew, hNew, fNew;
						gNew = cellDetails[i][j].g + 1.0;
						hNew = calculateHValue(neighbour,
											dest);
						fNew = gNew + hNew;

						// If it isn’t on the open list, add
						// it to the open list. Make the
						// current square the parent of this
						// square. Record the f, g, and h
						// costs of the square cell
						//			 OR
						// If it is on the open list
						// already, check to see if this
						// path to that square is better,
						// using 'f' cost as the measure.
						if (cellDetails[neighbour.first]
									[neighbour.second]
										.f
								== -1
							|| cellDetails[neighbour.first]
										[neighbour.second]
											.f
								> fNew) {
							openList.emplace(
								fNew, neighbour.first,
								neighbour.second);

							// Update the details of this
							// cell
							cellDetails[neighbour.first]
									[neighbour.second]
										.g
								= gNew;
							cellDetails[neighbour.first]
									[neighbour.second]
										.h
								= hNew;
							cellDetails[neighbour.first]
									[neighbour.second]
										.f
								= fNew;
							cellDetails[neighbour.first]
									[neighbour.second]
										.parent
								= { i, j };
						}
					}
				}
			}
		}
	}

	// When the destination cell is not found and the open
	// list is empty, then we conclude that we failed to
	// reach the destination cell. This may happen when the
	// there is no way to destination cell (due to
	// blockages)
	ROS_WARN("Failed to find the Destination Cell");
    return res;
}

bool service_handler(swarm::AStarReq::Request &req,
                    swarm::AStarReq::Response &res)
{
    array<array<int, 20>, 20> grid;
    for(int i = 0;i < 20;i++){
        for(int j = 0;j < 20;j++){
            grid[i][j] = req.flattenedGrid[i*20+j];
        }
    }
    Pair src(req.start.first,req.start.second);
	Pair dest(req.destination.first,req.destination.second);

	res = aStarSearch(grid, src, dest);
    return true;
}

// Driver program to test above function
int main(int argc, char **argv)
{
    //Init ROS related stuff
    ros::init(argc,argv,"astar_handler");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("astar/calc",service_handler);
	ROS_INFO("Ready");
    /*
    // Description of the Grid-
	//1--> The cell is not blocked
	//0--> The cell is blocked
	array<array<int, 10>, 9> grid{
		{ { { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 } },
		  { { 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 } },
		  { { 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 } },
		  { { 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 } },
		  { { 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 } },
		  { { 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 } },
		  { { 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 } },
		  { { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 } },
		  { { 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 } } }
	};

	// Source is the left-most bottom-most corner
	Pair src(8, 0);

	// Destination is the left-most top-most corner
	Pair dest(0, 0);

	aStarSearch(grid, src, dest);
    */
    ros::spin();
	return 0;
}