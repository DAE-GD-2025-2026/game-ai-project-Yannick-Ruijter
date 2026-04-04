#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
{
public:
	//=== SSFA Functions ===
	//--- References ---
	//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
	//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
	static std::vector<NavLine> FindPortals(std::vector<Node*> const & Path, TriPolygon const & NavPoly)
	{
		//Container
		std::vector<NavLine> Portals = {};
		Portals.emplace_back(Path.front()->GetPosition(), Path.back()->GetPosition());
		//For each node received, get it's corresponding line
		for (int i = 1; i < Path.size() - 1; ++i)
		{
			for (auto const& edge: NavPoly.GetEdges())
			{
				FVector2D nodePosition = FVector2D{edge.GetP1(NavPoly) + edge.GetP2(NavPoly)} / 2;
				if (nodePosition != Path[i]->GetPosition()) continue;
				auto toP1{FVector2D{edge.GetP1(NavPoly)} - Path[0]->GetPosition()};
				auto toP2{FVector2D{edge.GetP2(NavPoly)} - Path[0]->GetPosition()};
				
				double innerAngle{FVector2D::CrossProduct(toP1, toP2)};
				if (innerAngle < 0) Portals.emplace_back(FVector2D{edge.GetP1(NavPoly)}, FVector2D{edge.GetP2(NavPoly)});
				else Portals.emplace_back(FVector2D{edge.GetP2(NavPoly)}, FVector2D{edge.GetP1(NavPoly)});
			}
		}
			//Redetermine it's "orientation" based on the required path (left-right vs right-left) - p1 should be right point

			//Store portal

		//Add degenerate portal to force end evaluation
		//Portals.emplace_back(FVector2D::ZeroVector, FVector2D::ZeroVector);
		return Portals;
	}

	static std::vector<FVector2D> OptimizePortals( std::vector<NavLine> const & Portals, TriPolygon const & NavPoly)
	{
		std::vector<FVector2D> Path{};	
		Path.emplace_back(Portals.front().P1);
		FVector2D previousRightLeg = FVector2D::ZeroVector;
		FVector2D previousLeftLeg = FVector2D::ZeroVector;
		//do everything in a while true
		//increment the legs one by one if the conditions have been met
		
		//P1 == right point of portal, P2 == left point of portal
		//NavPoly.
		for (int i = 1; i < Portals.size(); ++i)
		{
			auto currentPortal = Portals[i];	
			auto newRightLeg{currentPortal.P1 - Path.back()};
			auto newLeftLeg{currentPortal.P2 - Path.back()};
			//--- RIGHT CHECK ---
			//1. See if moving funnel inwards - RIGHT
			//do going inwards check (i have no clue how to do this at all)
				//2. See if new line degenerates a line segment - RIGHT
					//Leftleg becomes new apex point
				//else
					//Calculate new legs (if not the end)
			
			//--- LEFT CHECK ---
			//1. See if moving funnel inwards - LEFT
				//2. See if new line degenerates a line segment - LEFT
					//Rightleg becomes new apex point
				//else
					//Calculate new legs (if not the end)
			
			/*if (FVector2D::CrossProduct(previousRightLeg, newRightLeg) >= 0)
			{
				double cross{FVector2D::CrossProduct(newRightLeg, newLeftLeg)};
				if (cross > 0)
				{
					Path.emplace_back(Portals[i].P2);
				}
				else
				{
					continue;
				}
			}
			
			if (FVector2D::CrossProduct(previousLeftLeg, newLeftLeg) >= 0)
			{
				double cross{FVector2D::CrossProduct(newLeftLeg, previousLeftLeg)};
				//2. See if new line degenerates a line segment - RIGHT
				if (cross > 0)
				{
					//Leftleg becomes new apex point
					Path.emplace_back(Portals[i].P2);
				}
				else
				{
					//Calculate new legs (if not the end)
					continue;
				}
			}
*/

		}
		// Add last path point

		Path.emplace_back(Portals.front().P2);
		return Path;
	}
private:
	SSFA() {};
	~SSFA() {};
};
}
