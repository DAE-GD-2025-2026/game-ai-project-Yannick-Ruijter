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

	int NumberOfCells = Rows * Rows;
	Cells.reserve(NumberOfCells);
	for (int i = 0; i < Rows; ++i)
	{
		for (int j = 0; j < Cols; ++j)
		{
			Cells.emplace_back(
				-Width / 2 + CellWidth * j,
				-Height / 2 + CellHeight * i,
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
	auto it = std::ranges::find(Cells[OldIndex].Agents, &Agent);
	if (it != Cells[OldIndex].Agents.end()) Cells[OldIndex].Agents.erase(it);
	
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;
	for (int cellIndex = 0; cellIndex < Cells.size(); ++cellIndex)
	{
		if (!DoRectsOverlap(Cells[cellIndex].BoundingBox, Cells[cellIndex].BoundingBox)) continue;
		for (ASteeringAgent* cellAgent : Cells[cellIndex].Agents)
		{
			if (cellAgent == &Agent) continue;

			if (UKismetMathLibrary::Vector_Distance2DSquared(
					Agent.GetActorLocation(),
					cellAgent->GetActorLocation()
				) <= QueryRadius * QueryRadius)
			{
				Neighbors[NrOfNeighbors++] = cellAgent;
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
		FVector2D BoxCenter = (Cell.BoundingBox.Min + Cell.BoundingBox.Max) * 0.5f;
		FVector2D BoxExtent = (Cell.BoundingBox.Max - Cell.BoundingBox.Min) * 0.5f;
		DrawDebugBox(pWorld, FVector{BoxCenter, 10.f}, FVector{BoxExtent, 0.f}, FColor::Red);
	}
}

int CellSpace::PositionToIndex(const FVector2D& Pos) const
{
	int col = FMath::Clamp(
		static_cast<int>((Pos.X + SpaceWidth / 2) / CellWidth),
		0, NrOfCols - 1
	);

	int row = FMath::Clamp(
		static_cast<int>((Pos.Y + SpaceHeight / 2) / CellHeight),
		0, NrOfRows - 1
	);

	return row * NrOfCols + col;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}