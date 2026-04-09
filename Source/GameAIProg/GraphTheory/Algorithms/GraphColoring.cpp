#include "GraphColoring.h"

#include <queue>
#include <stack>

GameAI::GraphColoring::GraphColoring(GameAI::Graph* graph, UWorld* world)
	:m_pGraph{graph}, m_pWorld{world}, coloredNodes{}
	,colors{FColor::Red, FColor::Blue, FColor::Green, FColor::Yellow, FColor::Purple, FColor::White, FColor::Black, FColor::Cyan, FColor::Orange}
{
}

void GameAI::GraphColoring::RecalculateColors()
{
	coloredNodes.clear();
	std::vector<GameAI::Node*> nodes{};
	for (auto const& node : m_pGraph->GetNodes())
		nodes.emplace_back(node.get());
	for (int i = 0; i < colors.size(); i++)
	{
		coloredNodes.emplace_back(nodes[0], i);
		if (TryAssignColors(nodes, 1))
		{
			break;
		}
		coloredNodes.pop_back();
	}
}

void GameAI::GraphColoring::DrawColors() const
{
	for (auto const& node : coloredNodes)
	{
		DrawDebugCircle(m_pWorld, FVector{node.node->GetPosition(), 6.f}, 20, 10, colors[node.colorIndex]
			, false, -1, 0, 0, FVector{1,0,0}, FVector{0,1,0});
	}
}

bool GameAI::GraphColoring::TryAssignColors(std::vector<GameAI::Node*>& nodes, int nodeIndex)
{
	if (nodeIndex == nodes.size()) return true;
	auto neighbors = m_pGraph->FindConnectionsFrom(nodes[nodeIndex]->GetId());
	for (int i{}; i < colors.size(); i++)
	{
		if (CheckColorValidity(nodes[nodeIndex], i))
		{
			coloredNodes.emplace_back(nodes[nodeIndex], i);
			
			if (TryAssignColors(nodes, nodeIndex + 1))
				return true;
			
			coloredNodes.pop_back();
		}
	}
	return false;
}

bool GameAI::GraphColoring::CheckColorValidity(GameAI::Node* node, int colorIndex)
{
	std::vector<int> neighboringColorIndices{};
	for (auto const& connection : m_pGraph->FindConnectionsFrom(node->GetId()))
	{
		GameAI::Node* toNode = m_pGraph->GetNode(connection->GetToId()).get();
		
		auto it = std::ranges::find_if(coloredNodes,
			[toNode](auto const& c){ return c.node == toNode; });
		if (it != coloredNodes.end())
			neighboringColorIndices.push_back(it->colorIndex);
	}
	std::ranges::sort(neighboringColorIndices);
	for (auto const& index : neighboringColorIndices)
	{
		if (index == colorIndex) return false;
	}	
	return true;
}

//greedy BFS algorithm
/*void GameAI::GraphColoring::RecalculateColors()
{
	coloredNodes.clear();
	//int colorCount = 1;
	std::vector<GameAI::Node*> remainingNodes;
	for (auto const& node : m_pGraph->GetNodes())
		remainingNodes.push_back(node.get());

	if (remainingNodes.empty()) return;
	GameAI::Node* currentNode{nullptr};
	std::queue<GameAI::Node*> queue;
	std::stack<GameAI::Node*> stack;
	queue.push(remainingNodes[0]);
	while (!remainingNodes.empty() || !queue.empty())
	{
		if (queue.empty())
			queue.push(remainingNodes[0]);
		
		currentNode = queue.front();
		queue.pop();

		auto it = std::ranges::find_if(remainingNodes,
			[currentNode](auto const& n){ return n == currentNode; });
		if (it != remainingNodes.end())
			remainingNodes.erase(it);

		std::vector<int> neighboringColorIndices{};
		for (auto const& connection : m_pGraph->FindConnectionsFrom(currentNode->GetId()))
		{
			GameAI::Node* toNode = m_pGraph->GetNode(connection->GetToId()).get();

			auto remainIt = std::ranges::find_if(remainingNodes,
				[toNode](auto const& n){ return n == toNode; });

			if (remainIt != remainingNodes.end())
			{
				queue.push(toNode);
			}
			else
			{
				auto coloredIt = std::ranges::find_if(coloredNodes,
					[toNode](auto const& c){ return c.node == toNode; });
				if (coloredIt != coloredNodes.end())
					neighboringColorIndices.push_back(coloredIt->colorIndex);
			}
		}
		std::ranges::sort(neighboringColorIndices);
		int colorIndex = 0;
		for (auto const& index : neighboringColorIndices)
		{
			if (index == colorIndex) colorIndex++;
		}
		coloredNodes.emplace_back(currentNode, colorIndex);
	}
}*/
