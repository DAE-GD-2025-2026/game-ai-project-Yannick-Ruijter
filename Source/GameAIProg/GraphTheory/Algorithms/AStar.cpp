#include "AStar.h"

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*>AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};
	std::vector<NodeRecord> openList{};
	std::vector<NodeRecord> closedList{};
	NodeRecord currentNodeRecord{pStartNode, nullptr, 0.f, GetHeuristicCost(pStartNode, pGoalNode)};
	openList.push_back(currentNodeRecord);
	while (!openList.empty())
	{
		currentNodeRecord = *std::min_element(openList.begin(), openList.end());
		if (currentNodeRecord.pNode == pGoalNode) break;
		auto connections = pGraph->FindConnectionsFrom(currentNodeRecord.pNode->GetId());
		for (auto connection : connections)
		{
			bool addToList = true;
			float currentGCost{currentNodeRecord.costSoFar + connection->GetWeight()};
			for (auto const& node : closedList)
			{
				if (node.pNode->GetId() == connection->GetToId())
				{
					if (currentGCost >= node.costSoFar)
					{
						addToList = false;
						break;
					}
					closedList.erase(std::ranges::find(closedList.begin(), closedList.end(), node));
				}
			}
			for (auto const& node : openList)
			{
				if (node.pNode->GetId() == connection->GetToId())
				{
					if (currentGCost >= node.costSoFar)
					{
						addToList = false;
						break;
					}
					openList.erase(std::ranges::find(openList.begin(), openList.end(), node));
				}
			}
			if (!addToList) continue;
			openList.emplace_back(pGraph->GetNode(connection->GetToId()).get(), connection, currentGCost, GetHeuristicCost(pGraph->GetNode(connection->GetToId()).get(), pGoalNode));
		}
		openList.erase(std::ranges::find(openList.begin(), openList.end(), currentNodeRecord));
		closedList.emplace_back(currentNodeRecord);
	}
	if (openList.empty())
	{
		path.emplace_back(pStartNode);
		return path;
	}
	while (currentNodeRecord.pConnection)
	{
		path.emplace_back(currentNodeRecord.pNode);
		for (auto const& node : closedList)
		{
			if (node.pNode->GetId() == currentNodeRecord.pConnection->GetFromId())
			{
				currentNodeRecord = node;
				break;
			}
		}
	}
	path.emplace_back(pStartNode);
	std::ranges::reverse(path.begin(), path.end());
	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}