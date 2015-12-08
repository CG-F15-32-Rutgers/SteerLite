//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"

namespace SteerLib
{

	/*
		@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
		@attributes 
		f : the f value of the node
		g : the cost from the start, for the node
		point : the point in (x,0,z) space that corresponds to the current node
		parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
		@operators 
		The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/
	class STEERLIB_API AStarPlannerNode{
		public:
			double f;
			double g;
			Util::Point point;
			AStarPlannerNode* parent;
			AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
			{
				f = _f;
				point = _point;
				g = _g;
				parent = _parent;
			}
			bool operator<(AStarPlannerNode other) const
		    {
		        return this->f < other.f;
		    }
		    bool operator>(AStarPlannerNode other) const
		    {
		        return this->f > other.f;
		    }
		    bool operator==(AStarPlannerNode other) const
		    {
		        return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		    }

	};

	

	class STEERLIB_API AStarPlanner{
		public:
			AStarPlanner();
			~AStarPlanner();
			int AstarWeight = 1;
			// NOTE: There are four indices that need to be disambiguated
			// -- Util::Points in 3D space(with Y=0)
			// -- (double X, double Z) Points with the X and Z coordinates of the actual points
			// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
			// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
			// When navigating the space or the Grid, do not mix the above up

			/*
				@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
				The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
				and checks cells in bounding box area
				[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
				[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
				This function also contains the griddatabase call that gets traversal costs.
			*/
			bool canBeTraversed ( int id );
			/*
				@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
			*/
			Util::Point getPointFromGridIndex(int id);
			double AStarPlanner::heuristic(Util::Point a, Util::Point b);

			std::vector<Util::Point> AStarPlanner::reconstruct(AStarPlannerNode _Node, Util::Point start);
			/*
				@function computePath
				DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
				This function executes an A* query
				@parameters
				agent_path : The solution path that is populated by the A* search
				start : The start point
				goal : The goal point
				_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
				append_to_path : An optional argument to append to agent_path instead of overwriting it.
			*/

			bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path = false);
	private:
		SteerLib::GridDatabase2D * gSpatialDatabase;
		//Heuristic function from startIndex to endIndex
		double heuristic_E(int start, int end);
		double heuristic_M(int start, int end);
		//Expands at currentNode, adding eligible neighbors to openset and modifying f,g scores and parent node
		void neighbors(int current, int goal, std::set<int>& open_set, std::set<int> closed_set, std::map<int, double>& g_score, std::map<int, double>& f_score, std::map<int, int>& came_from);
		//Adds found path to agent_path
		bool reconstruct(std::vector<Util::Point>& agent_path, int current, int start, std::map<int, int>& came_from);
		//Finds node with lowest fscore in openset
		int getCurrent(std::set<int> open_set, std::map<int, double> g_score, std::map<int, double> f_score);
		void AStarPlanner::init_score(std::map<int, double>& g_score, std::map<int, double>& f_score, SteerLib::GridDatabase2D * _gSpatialDatabase);
	};




}


#endif
