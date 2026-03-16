#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

std::vector<TriPolygon::Triangle> GameAI::NavGraph::GetTrianglesFromEdgeIndex(int EdgeIdx) const
{
	std::vector<TriPolygon::Triangle> result;
	for (auto const& triangle: pNavPoly->GetTriangles())
	{
		auto edges = triangle.GetEdges();
		for (auto const& edge: edges)
		{
			if (pNavPoly->FindEdgeIndex(edge) == EdgeIdx)
			{
				result.emplace_back(triangle);
				break;
			}
		}
	}
	return result;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	//1. Go over all the edges of the navigation mesh and create nodes
			// Create node here
	for (auto const& edge: pNavPoly->GetEdges())
	{
		auto index = pNavPoly->FindEdgeIndex(edge);
		if (GetTrianglesFromEdgeIndex(index.value()).size() > 1)
		{
			FVector2D position = FVector2D{edge.GetP1(*pNavPoly.get()) + edge.GetP2(*pNavPoly.get())} / 2.f;
			AddNode(std::make_unique<NavGraphNode>(position, index.value()));
		}
	}
	
	//2. Create connections now that every node is created	
		//2 valid nodes -> 1 connection
		//3 valid nodes -> 3 connections
	for (auto const& triangle: pNavPoly->GetTriangles())
	{
		auto edges = triangle.GetEdges();
		std::vector<int> nodesFound{};
		for (auto const& edge: edges)
		{
			auto edgeIndex = pNavPoly->FindEdgeIndex(edge);
			auto nodeId = GetNodeIdFromEdgeIndex(edgeIndex.value());
			if (nodeId != Graphs::InvalidNodeId)
			{
				nodesFound.emplace_back(nodeId);
			}
		}
		if (nodesFound.size() == 2) AddConnection(nodesFound[0], nodesFound[1]);
		else if (nodesFound.size() == 3)
		{
			AddConnection(nodesFound[0], nodesFound[1]);
			AddConnection(nodesFound[1], nodesFound[2]);
			AddConnection(nodesFound[2], nodesFound[0]);
		}
	}
		
	//3. Set the connections cost to the actual distance
	for (auto const& connection: GetConnections())
	{
		auto distance = FVector2D::Distance(GetNode(connection->GetFromId())->GetPosition(), GetNode(connection->GetToId())->GetPosition());
		connection->SetWeight(distance);
	}
}
