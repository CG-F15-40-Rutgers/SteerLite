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



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO		
		std::set<AStarPlannerNode*, nodePtrComp> closedset, openset;
		std::map<int, AStarPlannerNode> nodes;

		int startIndex = gSpatialDatabase->getCellIndexFromLocation(start);
		int goalIndex = gSpatialDatabase->getCellIndexFromLocation(goal);

		nodes[startIndex] = AStarPlannerNode(getPointFromGridIndex(startIndex), 0.0, heuristic_cost_estimate(start, goal), NULL);
		openset.insert(&nodes[startIndex]);

		nodes[goalIndex] = AStarPlannerNode(getPointFromGridIndex(goalIndex), DBL_MAX, DBL_MAX, NULL);

		while (!openset.empty())
		{
			AStarPlannerNode *current = *openset.begin();

			// Reached goal
			if (*current == nodes[goalIndex])
			{
				AStarPlannerNode *curr = current;
				agent_path.clear();

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

					if (current->point == neighborPoint)
						continue;

					// Continue if neighbor is in closedset
					if (nodes.find(neighborIndex) != nodes.end())
					{
						if (closedset.find(&nodes[neighborIndex]) != closedset.end())
							continue;
					}

					if (canBeTraversed(neighborIndex))
					{
						double tentative_g_score = current->g + Util::distanceBetween(current->point, neighborPoint);

						if (nodes.find(neighborIndex) == nodes.end())
						{
							nodes[neighborIndex] = AStarPlannerNode(neighborPoint, DBL_MAX, DBL_MAX, NULL);
						}

						AStarPlannerNode *neighbor = &nodes[neighborIndex];

						if (tentative_g_score < neighbor->g)
						{
							neighbor->parent = current;
							neighbor->g = tentative_g_score;
							neighbor->f = neighbor->g + heuristic_cost_estimate(neighborPoint, goal);
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
		return Util::distanceBetween(start, goal);
	}
}