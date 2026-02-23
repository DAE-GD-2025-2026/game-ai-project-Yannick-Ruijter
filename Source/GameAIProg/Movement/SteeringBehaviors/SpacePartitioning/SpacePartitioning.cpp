#include "SpacePartitioning.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	, SpaceWidth{Width}
	, SpaceHeight{Height}
	, NrOfRows{Rows}
	, NrOfCols{Cols}
	, NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);
	
	//calculate bounds of a cell
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;

	int NumberOfCells = CellWidth * CellHeight;
	Cells.reserve(NumberOfCells);
	for (int i = 0; i < Rows; ++i)
	{
		for (int j = 0; j < Cols; ++j)
		{
			Cells.emplace_back(Width / CellWidth * i,
				CellHeight / CellHeight * j,
				CellWidth, CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	int Index = PositionToIndex(Agent.GetPosition());
	Cells[Index].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	int const CurrentIndex = PositionToIndex(Agent.GetPosition());
	int const OldIndex = PositionToIndex(OldPos);
	if (CurrentIndex == OldIndex) return;
	Cells[CurrentIndex].Agents.emplace_back(&Agent);
	Cells[OldIndex].Agents.erase(
		std::ranges::find(Cells[OldIndex].Agents, &Agent));
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;
	int AgentCellIndex = PositionToIndex(Agent.GetPosition());
	for (int i = 0; i < Cells.size(); ++i)
	{
		if (&Cells[i] == &Cells[AgentCellIndex]) continue;
		if (!DoRectsOverlap(Cells[i].BoundingBox, Cells[AgentCellIndex].BoundingBox)) continue;
		for (auto CellAgent : Cells[i].Agents)
		{
			if (UKismetMathLibrary::Vector_Distance2DSquared(
				Agent.GetActorLocation(), CellAgent->GetActorLocation()) < QueryRadius)
			{
				Neighbors[NrOfNeighbors] = CellAgent;
				++NrOfNeighbors;
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	for (auto const& Cell : Cells)
	{
		FVector2D BoxCenter{
			Cell.BoundingBox.Min.X + Cell.BoundingBox.Max.X / 2
			,Cell.BoundingBox.Min.Y + Cell.BoundingBox.Max.Y / 2 };
		FVector2D BoxExtent{BoxCenter - Cell.BoundingBox.Max};
		DrawDebugBox(pWorld, FVector{BoxCenter, 10.f}, FVector{BoxExtent, 0.f}, FColor::Red);
	}
}

int CellSpace::PositionToIndex(FVector2D const & Pos) const
{
	for (int i = 0; i < Cells.size(); ++i)
	{
		if (Pos.ComponentwiseAllGreaterThan(Cells[i].BoundingBox.Min) && Pos.ComponentwiseAllLessThan(Cells[i].BoundingBox.Max))
		{
			return i;
		}
	}
	return 0;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}