#pragma once
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	struct ColoredNode
	{
		GameAI::Node* node;
		int colorIndex;
	};

	class GraphColoring
	{
	public:
		GraphColoring(GameAI::Graph* graph,UWorld* world);
		~GraphColoring() = default;
		
		void RecalculateColors();
		void DrawColors() const;
	private:
		GameAI::Graph* m_pGraph; 
		UWorld* m_pWorld;
		std::vector<ColoredNode> coloredNodes;
		std::vector<FColor> colors;
		
		bool TryAssignColors(std::vector<GameAI::Node*>& nodes, int nodeIndex);
		bool CheckColorValidity(GameAI::Node* node, int colorIndex);
	};
}
