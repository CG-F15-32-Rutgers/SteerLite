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
#define WEIGHT 1
#define COST 1
#define heuristic(X,Y) heuristic_E(X, Y)
#define FAVOR_LARGE true

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

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

	double AStarPlanner::heuristic_E(int start, int end) 
	{
		unsigned int startx, startz, endx, endz;
		gSpatialDatabase->getGridCoordinatesFromIndex(start, startx, startz);
		gSpatialDatabase->getGridCoordinatesFromIndex(end, endx, endz);
		return WEIGHT*((double)sqrt((startx - endx)*(startx - endx) + (startz - endz)*(startz - endz)));
	}

	double AStarPlanner::heuristic_M(int start, int end)
	{
		unsigned int startx, startz, endx, endz;
		gSpatialDatabase->getGridCoordinatesFromIndex(start, startx, startz);
		gSpatialDatabase->getGridCoordinatesFromIndex(end, endx, endz);
		return WEIGHT*((abs((double)startx - endx) + abs((double)startz - endz)));
	}

	void AStarPlanner::neighbors(int current, int goal, std::set<int>& open_set, std::set<int> closed_set, std::map<int, double>& g_score, std::map<int, double>& f_score, std::map<int, int>& came_from)
	{
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current, x, z);
		for (int i = MAX(x - 1, 0); i<MIN(x + 2, gSpatialDatabase->getNumCellsX()); i += GRID_STEP) 
		{
			for (int j = MAX(z - 1, 0); j<MIN(z + 2, gSpatialDatabase->getNumCellsZ()); j += GRID_STEP) 
			{
				int neighbor = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				if (canBeTraversed(neighbor) && closed_set.count(neighbor) == 0) 
				{
					double tentative_g;
					if ((i == x) || (j == z)) 
						tentative_g = g_score[current] + (gSpatialDatabase->getTraversalCost(neighbor));
					else 
						tentative_g = g_score[current] + COST*gSpatialDatabase->getTraversalCost(neighbor);

					if (tentative_g < g_score[neighbor])
					{
						g_score[neighbor] = tentative_g;
						f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal);
						if (open_set.count(neighbor) == 1)
							open_set.erase(open_set.find(neighbor));

						open_set.insert(neighbor);
						came_from[neighbor] = current;
					}
				}
			}
		}
	}

	bool AStarPlanner::reconstruct(std::vector<Util::Point>& agent_path, int current, int start, std::map<int, int>& came_from) 
	{
		int tmp = current;
		std::vector<Util::Point> path;
		path.push_back(getPointFromGridIndex(tmp));
		while (tmp != start) 
		{
			tmp = came_from[tmp];
			path.push_back(getPointFromGridIndex(tmp));
		}
		for (int i = path.size() - 1; i >= 0; --i) 
			agent_path.push_back(path.at(i));

		std::cout << "\nPath length: " << path.size() << '\n';
		return true;
	}

	int AStarPlanner::getCurrent(std::set<int> open_set, std::map<int, double> g_score, std::map<int, double> f_score) 
	{
		std::set<int>::iterator iter;
		double tmp = INFINITY;
		for (std::set<int>::iterator i = open_set.begin(); i != open_set.end(); ++i) 
		{
			if (f_score[(*i)] < tmp) 
			{
				tmp = f_score[(*i)];
				iter = i;
			}
			else if (f_score[(*i)] == tmp) 
			{
				if (FAVOR_LARGE) 
				{
					if (g_score[(*iter)] < g_score[(*i)]) 
						iter = i;
				}
				else {
					if (g_score[(*iter)] > g_score[(*i)]) 
						iter = i;
				}
			}
		}
		return (*iter);
	}

	void AStarPlanner::init_score(std::map<int, double>& g_score, std::map<int, double>& f_score, SteerLib::GridDatabase2D * _gSpatialDatabase)
	{
		gSpatialDatabase = _gSpatialDatabase;
		for (int i = 0; i < gSpatialDatabase->getNumCellsX(); ++i) 
		{
			for (int j = 0; j < gSpatialDatabase->getNumCellsZ(); ++j)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				g_score[index] = INFINITY;
				f_score[index] = INFINITY;
			}
		}
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		std::set<int> open_set;
		std::set<int> closed_set;

		std::map<int, double> g_score;
		std::map<int, double> f_score;
		std::map<int, int> came_from;
		init_score(g_score, f_score, gSpatialDatabase);

		int _start = gSpatialDatabase->getCellIndexFromLocation(start);
		int _goal = gSpatialDatabase->getCellIndexFromLocation(goal);
		g_score[_start] = 0.0;
		f_score[_start] = g_score[_start] + heuristic(_start, _goal);
		open_set.insert(_start);

		while (!open_set.empty()) 
		{
			int current = getCurrent(open_set, g_score, f_score);
			closed_set.insert(current);
			open_set.erase(open_set.find(current));

			if (current == _goal) 
			{
				std::cout << "\nExpanded Nodes: " << closed_set.size();
				return reconstruct(agent_path, current, _start, came_from);
			}

			neighbors(current, _goal, open_set, closed_set, g_score, f_score, came_from);
		}
		//TODO
		std::cout << "\nIn A*";

		return false;
	}
}