//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

// Set MANHATTAN_DISTANCE to determine the heuristic function to use
// true = Manhattan distance
// false = Euclidean distance
#define MANHATTAN_DISTANCE false

// Set F_TIE_BEHAVIOR to control how nodes are sorted if their f values are equal
// 0 = default
// 1 = favor smaller g value
// 2 = favor larger g value
#define F_TIE_BEHAVIOR 0

// Set DIAGONAL_COST to increase cost of diagonal neighbors
// Default value is 1
// Set F_TIE_BEHAVIOR = 2 for values other than 1
#define DIAGONAL_COST 1

// Set HEURISTIC_WEIGHT to increase the weight of heuristics
// Default value is 1
// Set F_TIE_BEHAVIOR = 2 for values other than 1
#define HEURISTIC_WEIGHT 1

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	// Custom comparator used to order nodes in openset and closedset
	struct nodePtrComp
	{
		bool operator()(const AStarPlannerNode *a, const AStarPlannerNode *b) const
		{
			switch (F_TIE_BEHAVIOR)
			{
			case 0:
				return *a < *b;
			case 1:
				// Favor smaller g value
				return (a->f == b->f) ? (a->g < b->g) : (*a < *b);
			case 2:
				// Favor larger g value
				return (a->f == b->f) ? (a->g > b->g) : (*a < *b);
			default:
				return *a < *b;
			}
		}
	};

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		std::set<AStarPlannerNode*, nodePtrComp> closedset, openset;
		std::map<int, AStarPlannerNode> nodes;

		int startIndex = gSpatialDatabase->getCellIndexFromLocation(start);
		int goalIndex = gSpatialDatabase->getCellIndexFromLocation(goal);

		// Create start node and add it to openset
		nodes[startIndex] = AStarPlannerNode(getPointFromGridIndex(startIndex), 0.0, heuristic_cost_estimate(start, goal), NULL);
		openset.insert(&nodes[startIndex]);

		// Create goal node
		nodes[goalIndex] = AStarPlannerNode(getPointFromGridIndex(goalIndex), DBL_MAX, DBL_MAX, NULL);

		// Loop while openset contains nodes
		while (!openset.empty())
		{
			// Choose first node in openset(this has the lowest f value)
			AStarPlannerNode *current = *openset.begin();

			// Reached goal
			if (*current == nodes[goalIndex])
			{
				AStarPlannerNode *curr = current;
				agent_path.clear();

				std::cout << "\n\nPath length: " << curr->g << "\n";
				std::cout << "Number of expanded nodes: " << closedset.size() << "\n\n";

				// Reconstruct path
				while (curr != NULL)
				{
					agent_path.insert(agent_path.begin(), curr->point);
					curr = curr->parent;
				}

				return true;
			}

			// Move current from openset to closedset
			openset.erase(openset.begin());
			closedset.insert(current);

			// Loop through neighbors
			unsigned int x, z;
			gSpatialDatabase->getGridCoordinatesFromIndex(gSpatialDatabase->getCellIndexFromLocation(current->point), x, z);
			int x_range_min, x_range_max, z_range_min, z_range_max;

			x_range_min = MAX(x - GRID_STEP, 0);
			x_range_max = MIN(x + GRID_STEP, gSpatialDatabase->getNumCellsX());

			z_range_min = MAX(z - GRID_STEP, 0);
			z_range_max = MIN(z + GRID_STEP, gSpatialDatabase->getNumCellsZ());

			for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
			{
				for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
				{
					int neighborIndex = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
					Util::Point neighborPoint = getPointFromGridIndex(neighborIndex);

					// Skip node if it is the current node
					if (current->point == neighborPoint)
						continue;

					// Skip node if it is in closedset
					if (nodes.find(neighborIndex) != nodes.end())
					{
						if (closedset.find(&nodes[neighborIndex]) != closedset.end())
							continue;
					}

					if (canBeTraversed(neighborIndex))
					{
						double tentative_g_score;

						// Calculate tentative g score
						if ((current->point.x != neighborPoint.x) && (current->point.z != neighborPoint.z))
						{
							// Diagonal node
							tentative_g_score = current->g + DIAGONAL_COST * Util::distanceBetween(current->point, Util::Point(i, 0, j));
						}
						else
						{
							// Not diagonal node
							tentative_g_score = current->g + Util::distanceBetween(current->point, Util::Point(i, 0, j));
						}

						if (nodes.find(neighborIndex) == nodes.end())
						{
							// Create new node if it was never visited before
							nodes[neighborIndex] = AStarPlannerNode(neighborPoint, DBL_MAX, DBL_MAX, current);
						}

						AStarPlannerNode *neighbor = &nodes[neighborIndex];

						// Tentative g score is less than neighbor's previous g score
						if (tentative_g_score < neighbor->g)
						{
							neighbor->parent = current;
							neighbor->g = tentative_g_score;
							neighbor->f = neighbor->g + heuristic_cost_estimate(neighborPoint, goal);

							// Add neighbor to openset if it's not in openset
							if (openset.find(neighbor) == openset.end())
							{
								openset.insert(neighbor);
							}
						}
					}
				}
			}
		}

		return false;
	}

	double AStarPlanner::heuristic_cost_estimate(Util::Point start, Util::Point goal)
	{
		if (MANHATTAN_DISTANCE)
		{
			return HEURISTIC_WEIGHT * abs(start.x - goal.x) + abs(start.z - goal.z);
		}
		else
		{
			return HEURISTIC_WEIGHT * Util::distanceBetween(start, goal);
		}
	}
}