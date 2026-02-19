#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	FVector2D AveragePos = pFlock->GetAverageNeighborPos();
	SetTarget(FTargetData{AveragePos});
	return Seek::CalculateSteering(deltaT, pAgent);;
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	TArray<ASteeringAgent*> Neighbors = pFlock->GetNeighbors();
	const int NrOfNeighbors = pFlock->GetNrOfNeighbors();
	FVector2D TotalVelocity = FVector2D::ZeroVector;
	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		FVector2D CurrentVelocity = pAgent.GetPosition() - Neighbors[i]->GetPosition();
		float const VelocityWeight = 1.f/CurrentVelocity.Length();
		CurrentVelocity.Normalize();
		TotalVelocity += CurrentVelocity * VelocityWeight;
	}
	SetTarget(FTargetData{pAgent.GetPosition() + TotalVelocity});
	return Seek::CalculateSteering(deltaT, pAgent);
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput Alignment::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	FVector2D AverageVel = pFlock->GetAverageNeighborVelocity();
	SetTarget(FTargetData{pAgent.GetPosition() + AverageVel});
	return Seek::CalculateSteering(deltaT, pAgent);
}
