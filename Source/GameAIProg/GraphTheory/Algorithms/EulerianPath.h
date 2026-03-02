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
		void VisitAllNodesDFS(Node* node, std::vector<bool>& notesFound, int& nrOfNotesFound) const;
		bool IsConnected() const;

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
		
		if (IsEulerian() == Eulerianity::notEulerian) return Path;
		
		std::stack<int> nodeStack;
		
		Node* currentNode = Nodes[0];
		bool pathFound{ false };
		while (!pathFound)
		{
			nodeStack.push(currentNode->GetId());
			std::vector<Connection*> outGoingConnections = graphCopy.FindConnectionsFrom(currentNode->GetId());
			if (outGoingConnections.size() == 0)
			{
				std::vector<Connection*> incomingConnections = graphCopy.FindConnectionsTo(currentNode->GetId());
				Path.push_back(currentNode);
				nodeStack.pop();
				currentNode = graphCopy.GetNode(incomingConnections[0]->GetFromId());
				graphCopy.RemoveConnection(incomingConnections[0]);
			}
			else
			{
				currentNode = graphCopy.GetNode(outGoingConnections[0]->GetFromId());
			}
		}
		
		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(Node* node, std::vector<bool>& notesFound, int& nrOfNotesFound) const
	{
		//fix this
		auto Connections = m_pGraph->FindConnectionsFrom(node->GetId());
		for (auto connection : Connections)
		{
			if (!notesFound[connection->GetToId()])
			{
				++nrOfNotesFound;
				VisitAllNodesDFS(m_pGraph->GetNode(connection->GetToId()), notesFound, nrOfNotesFound);
				notesFound[connection->GetToId()] = true;
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;
		
		std::vector<bool> visited(Nodes.size(), false);
		int NrOfNotesFound{ 1 };
		visited[Nodes[0]->GetId()] = true;
		auto connections = m_pGraph->FindConnectionsFrom(Nodes[0]->GetId());
		for (auto connection : connections)
		{
			if (!visited[connection->GetToId()])
			{
				++NrOfNotesFound;
				VisitAllNodesDFS(Nodes[connection->GetToId()], visited, NrOfNotesFound);
			}
			visited[connection->GetToId()] = true;
		}
		return NrOfNotesFound == Nodes.size();
	}
}