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
		
		void FindChildConnections(Node* node, int notesFound[], int& nrOfNotesFound) const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		// TODO If the graph is not connected, there can be no Eulerian Trail
		if (!IsConnected()) return Eulerianity::notEulerian;
		// TODO Count nodes with odd degree 

		// TODO A connected graph with more than 2 nodes with an odd degree (an odd amount of connections) is not Eulerian

		// TODO A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian (unless there are only 2 nodes)
		// TODO An Euler trail can be made, but only starting and ending in these 2 nodes

		// TODO A connected graph with no odd nodes is Eulerian
		
		return Eulerianity::notEulerian;
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
		
		int NotesFound[Nodes.size()]{};
		int NrOfNotesFound{ 1 };
		// TODO choose a starting node
		NotesFound[Nodes[0]->GetId()] = 1;
		auto connections = m_pGraph->FindConnectionsFrom(Nodes[0]->GetId());
		// TODO start a depth-first-search traversal from the node that has at least one connection
		for (auto connection : connections)
		{
			if (NotesFound[connection->GetToId()] == 0)
			{
				++NrOfNotesFound;
				FindChildConnections(Nodes[connection->GetToId()], NotesFound, NrOfNotesFound);
			}
			NotesFound[connection->GetToId()] = 1;
		}
		return NrOfNotesFound == Nodes.size();
	}
	
	inline void EulerianPath::FindChildConnections(Node* node, int notesFound[], int& nrOfNotesFound) const
	{
		auto Connections = m_pGraph->FindConnectionsFrom(node->GetId());
		for (auto connection : Connections)
		{
			if (notesFound[connection->GetToId()] == 0)
			{
				++nrOfNotesFound;
				FindChildConnections(m_pGraph->GetNode(connection->GetToId()), notesFound, nrOfNotesFound);
				notesFound[connection->GetToId()] = 1;
			}
		}
	}
}