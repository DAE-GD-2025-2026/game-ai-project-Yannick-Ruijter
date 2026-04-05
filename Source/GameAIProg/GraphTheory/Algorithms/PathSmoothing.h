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
		Portals.emplace_back(Path.front()->GetPosition(), Path.front()->GetPosition());
		FVector2D currentPoint{Path.front()->GetPosition()};
		//For each node received, get it's corresponding line
		for (int i = 1; i < Path.size() - 1; ++i)
		{
			for (auto const& edge: NavPoly.GetEdges())
			{
				FVector2D nodePosition = FVector2D{edge.GetP1(NavPoly) + edge.GetP2(NavPoly)} / 2;
				if (nodePosition != Path[i]->GetPosition()) continue;
				auto toP1{FVector2D{edge.GetP1(NavPoly)} - currentPoint};
				auto toP2{FVector2D{edge.GetP2(NavPoly)} - currentPoint};
				double innerAngle{FVector2D::CrossProduct(toP1, toP2)};
				currentPoint = nodePosition;
				if (innerAngle < 0) Portals.emplace_back(FVector2D{edge.GetP2(NavPoly)}, FVector2D{edge.GetP1(NavPoly)});
				else Portals.emplace_back(FVector2D{edge.GetP1(NavPoly)}, FVector2D{edge.GetP2(NavPoly)});
				break;
			}
		}

		Portals.emplace_back(Path.back()->GetPosition(), Path.back()->GetPosition());
		return Portals;
	}
		
	static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const& Portals, TriPolygon const& NavPoly)
	{
	    std::vector<FVector2D> Path{};
	    Path.emplace_back(Portals.front().P1);

	    if (Portals.size() == 1) {
	        Path.emplace_back(Portals.back().P2);
	        return Path;
	    }

	    FVector2D apexPoint = Portals.front().P1;
	    FVector2D currentRightLeg = Portals[1].P1 - apexPoint;
	    FVector2D currentLeftLeg  = Portals[1].P2 - apexPoint;

	    int apexIndex = 0;
	    int rightLegIndex = 1;
	    int leftLegIndex = 1;
		//start at 2 since we the size is at least 2 and we already store the first portal's legs
	    for (int i = 2; i < Portals.size(); ++i)
	    {
	        auto const& currentPortal = Portals[i];
	        FVector2D newRightLeg = currentPortal.P1 - apexPoint;
	        FVector2D newLeftLeg  = currentPortal.P2 - apexPoint;

	        //if the right leg is going inwards
	        if (FVector2D::CrossProduct(currentRightLeg, newRightLeg) >= 0)
	        {
	            //if the right leg is crossing the left leg
	            if (FVector2D::CrossProduct(currentLeftLeg, newRightLeg) >= 0)
	            {
	                apexPoint = Portals[leftLegIndex].P2;
	                Path.emplace_back(apexPoint);

	                apexIndex = leftLegIndex;
	                i = leftLegIndex + 1;
	                rightLegIndex = i;
	                leftLegIndex = i;

	            	//to prevent Out of bounds reading
	                if (i < Portals.size()) break;
                    currentRightLeg = Portals[i].P1 - apexPoint;
                    currentLeftLeg  = Portals[i].P2 - apexPoint;
	                continue;
	            }
	            else
	            {
	                currentRightLeg = newRightLeg;
	                rightLegIndex   = i;
	            }
	        }

	        //if the left leg is going inwards
	        if (FVector2D::CrossProduct(newLeftLeg, currentLeftLeg) >= 0)
	        {
	            //if the left leg is crossing the right leg
	            if (FVector2D::CrossProduct(newLeftLeg, currentRightLeg) >= 0)
	            {
	                apexPoint = Portals[rightLegIndex].P1;
	                Path.emplace_back(apexPoint);

	                apexIndex = rightLegIndex;
	                i = rightLegIndex + 1;
	                rightLegIndex = i;
	                leftLegIndex  = i;

	                if (i >= Portals.size()) break;
                    currentRightLeg = Portals[i].P1 - apexPoint;
                    currentLeftLeg  = Portals[i].P2 - apexPoint;
	                continue;
	            }
	            else
	            {
	                currentLeftLeg = newLeftLeg;
	                leftLegIndex   = i;
	            }
	        }
	    }

	    Path.emplace_back(Portals.back().P2);
	    return Path;
	}
private:
	SSFA() {};
	~SSFA() {};
};
}


/*
*apex = portals[0].left; // start
left = portals[0].left;
right = portals[0].right;

for (i = 1; i < portals.size(); ++i)
{
newLeft = portals[i].left;
newRight = portals[i].right;

// update right
if (cross(right - apex, newRight - apex) <= 0)
{
if (apex == right || cross(left - apex, newRight - apex) > 0)
{
	right = newRight;
}
else
{
	// collapse → add left
	path.push_back(left);
	apex = left;
	reset funnel
	restart loop
}
}

// update left (mirror logic)
}
 */