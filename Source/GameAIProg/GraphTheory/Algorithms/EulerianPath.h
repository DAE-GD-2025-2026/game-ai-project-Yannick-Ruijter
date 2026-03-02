#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;
		
		void FindChildConnections(Node* node, std::vector<bool>& notesFound, int& nrOfNotesFound) const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		if (!IsConnected()) return Eulerianity::notEulerian;
		int OddNotesFound{ 0 };
		for (auto Node : m_pGraph->GetActiveNodes())
		{
			if (m_pGraph->FindConnectionsFrom(Node->GetId()).size() % 2 == 1) ++OddNotesFound;
			if (OddNotesFound > 2) return Eulerianity::semiEulerian;
		}

		if (OddNotesFound == 2 && m_pGraph->GetNodes().size() != 2) return Eulerianity::semiEulerian;
		
		return Eulerianity::eulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{ Graphs::InvalidNodeId };
		
		// TODO Check if there can be an Euler path
		if (!IsConnected()) return Path;
		
		// TODO Start algorithm loop
		std::stack<int> nodeStack;

		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex ) const
	{
		// TODO Mark the visited node

		// TODO Ask the graph for the connections from that node
		// TODO recursively visit any valid connected nodes that were not visited before
		// TODO Tip: use an index-based for-loop to find the correct index
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;
		
		std::vector<bool> visited(Nodes.size(), false);
		int NrOfNotesFound{ 1 };
		// TODO choose a starting node
		visited[Nodes[0]->GetId()] = true;
		auto connections = m_pGraph->FindConnectionsFrom(Nodes[0]->GetId());
		// TODO start a depth-first-search traversal from the node that has at least one connection
		for (auto connection : connections)
		{
			if (!visited[connection->GetToId()])
			{
				++NrOfNotesFound;
				FindChildConnections(Nodes[connection->GetToId()], visited, NrOfNotesFound);
			}
			visited[connection->GetToId()] = true;
		}
		return NrOfNotesFound == Nodes.size();
	}
	
	inline void EulerianPath::FindChildConnections(Node* node, std::vector<bool>& notesFound, int& nrOfNotesFound) const
	{
		auto Connections = m_pGraph->FindConnectionsFrom(node->GetId());
		for (auto connection : Connections)
		{
			if (!notesFound[connection->GetToId()])
			{
				++nrOfNotesFound;
				FindChildConnections(m_pGraph->GetNode(connection->GetToId()), notesFound, nrOfNotesFound);
				notesFound[connection->GetToId()] = true;
			}
		}
	}
}