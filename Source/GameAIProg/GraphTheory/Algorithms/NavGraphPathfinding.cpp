#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//Create the path to return
	std::vector<FVector2D> finalPath{};
	
	auto navPoly = pNavGraph->GetNavPolygon();
	auto startTriangle = navPoly->GetTriangleAtPosition(startPos, true);
	auto endTriangle = navPoly->GetTriangleAtPosition(endPos, true);
	//Get the start and endTriangle
	if (startTriangle == endTriangle) return std::vector{startPos, endPos};
	if (startTriangle == nullptr || endTriangle == nullptr) return std::vector<FVector2D>();
	//We have valid start/end triangles and they are not the same
	//=> Start looking for a path
	//Copy the graph
	auto graphCopy{pNavGraph->Clone()};
	//Create Extra node for the Start Node (Agent's position
	auto testignnode = std::make_unique<NavGraphNode>(startPos, -1);
	int startingId = graphCopy->AddNode(std::move(testignnode));
	for (auto const& edge : startTriangle->GetEdges())
	{
		auto nodeIndex = navPoly->FindEdgeIndex(edge);
		if (!nodeIndex.has_value()) continue;
		int nodeId = graphCopy->GetNodeIdFromEdgeIndex(nodeIndex.value());
		if (nodeId < 0) continue;
		auto nodeFromEdge = graphCopy->GetNode(nodeId).get();
		auto connection = std::make_unique<Connection>(startingId, nodeId);
		connection->SetWeight(FVector2D::Distance(startPos, nodeFromEdge->GetPosition()));
		graphCopy->AddConnection(std::move(connection));
	}
	//Create extra node for the endNode
	int endID = graphCopy->AddNode(std::make_unique<NavGraphNode>(endPos, graphCopy->GetNodes().size()));
	for (auto const& edge : endTriangle->GetEdges())
	{
		auto nodeIndex = navPoly->FindEdgeIndex(edge);
		if (!nodeIndex.has_value()) continue;
		int nodeId = graphCopy->GetNodeIdFromEdgeIndex(nodeIndex.value());
		if (nodeId < 0) continue;
		auto nodeFromEdge = graphCopy->GetNode(nodeId).get();
		auto connection = std::make_unique<Connection>(endID, nodeId);
		connection->SetWeight(FVector2D::Distance(endPos, nodeFromEdge->GetPosition()));
		graphCopy->AddConnection(std::move(connection));
	}
	//Run A star on new graph
	AStar pathFinding{graphCopy.get(), HeuristicFunctions::Chebyshev};
	auto nodePath = pathFinding.FindPath(graphCopy->GetNode(startingId).get(), graphCopy->GetNode(endID).get());
	for (auto const& node : nodePath)
	{
		finalPath.emplace_back(node->GetPosition());
	}
	//Debug Visualisation

	// Extra: Run optimiser on new graph (First check if everything works without SSFA!)
	debugPortals = SSFA::FindPortals(nodePath, *pNavGraph->GetNavPolygon());
	finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}