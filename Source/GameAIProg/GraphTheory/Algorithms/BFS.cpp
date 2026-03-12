#include "BFS.h"

#include <queue>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::queue<Node*> Queue;
	std::map<Node*, Node*> ClosedList;
	ClosedList.insert({pStartNode, nullptr});
	Node* CurrentNode{pStartNode};
	Queue.push(CurrentNode);
	std::vector<Node*> path;
	while (true)
	{	
		if (Queue.empty()) break;
		CurrentNode = Queue.front();
		Queue.pop();
		if (CurrentNode == pDestinationNode) return ReconstructPath(ClosedList, pStartNode, pDestinationNode);
		
		for (auto const& connection : pGraph->FindConnectionsFrom(CurrentNode->GetId()))
		{
			Node* ToNode{pGraph->GetNode(connection->GetToId()).get()};
			if (!ClosedList.contains(ToNode))
			{
				ClosedList.emplace(ToNode, CurrentNode);
				Queue.push(ToNode);
			}
		}
	}
	return std::vector<Node*>();
}


std::vector<Node*> BFS::ReconstructPath(std::map<Node*, Node*> parentMap, Node* const pStartNode, Node* const pDestinationNode) const
{
	Node* CurrentNode{pDestinationNode};
	std::vector<Node*> path;
	while (CurrentNode != pStartNode)
	{
		path.emplace_back(CurrentNode);
		CurrentNode = parentMap.at(CurrentNode);
	}
	path.emplace_back(CurrentNode);
	std::reverse(path.begin(), path.end());
	return path;
}
