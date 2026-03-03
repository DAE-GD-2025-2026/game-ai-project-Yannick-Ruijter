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
			if (OddNotesFound > 2) return Eulerianity::notEulerian;
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
		
		if (eulerianity == Eulerianity::notEulerian) return Path;
		
		std::stack<int> nodeStack;
		
		Node* currentNode = Nodes[0];
		Node* previousNode = nullptr;
		bool pathFound{ false };
		if (eulerianity == Eulerianity::semiEulerian)
		{
			for (auto node : Nodes)
			{
				if (graphCopy.FindConnectionsFrom(node->GetId()).size() % 2 == 1)
				{
					currentNode = node;
				}
			}
		}
		
		while (!pathFound)
		{
			auto Connections = graphCopy.FindConnectionsFrom(currentNode->GetId());
			nodeStack.push(currentNode->GetId());
			if (Connections.size() == 1 && previousNode != nullptr)
			{
				nodeStack.pop();
				Path.emplace_back(currentNode);
				previousNode = currentNode;
				currentNode = graphCopy.GetNode(Connections[0]->GetToId());
				graphCopy.RemoveConnection(Connections[0]);
			}
			else if (Connections.size() > 0)
			{
				for (auto connection : Connections)
				{
					if (previousNode != graphCopy.GetNode(connection->GetToId()))
					{
						previousNode = currentNode;
						currentNode = graphCopy.GetNode(connection->GetToId());
					}
				}
			}
			else
			{
				Path.emplace_back(currentNode);
				pathFound = true;
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
				notesFound[connection->GetToId()] = true;
				VisitAllNodesDFS(m_pGraph->GetNode(connection->GetToId()), notesFound, nrOfNotesFound);
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		//TODO fix this
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