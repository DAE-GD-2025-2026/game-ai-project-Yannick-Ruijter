#include "SteeringBehaviors.h"

#include "FrameTypes.h"
#include "VectorTypes.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h"

//SEEK
//*******
// DONE: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	float constexpr MaxSpeed{600.f};
	Agent.SetMaxLinearSpeed(MaxSpeed);
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	//show a cool thing
	//add debug rendering
	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	float constexpr MaxSpeed{600.f};
	Agent.SetMaxLinearSpeed(MaxSpeed);
	Steering.LinearVelocity = Agent.GetPosition() - Target.Position;
	//show a cool thing
	//add debug rendering
	return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	//fake max speed
	float constexpr MaxSpeed{600.f};
	//radia of the circle
	float constexpr OuterRadius{500.f};
	float constexpr InnerRadius{150.f};
	//difference
	FVector2D const Difference{Target.Position - Agent.GetPosition()};
	//distance
	float const DistanceToPoint{static_cast<float>(Difference.Length())};
	//clamped lerp value to decelerate
	float LerpValue{(DistanceToPoint - InnerRadius) / (OuterRadius - InnerRadius)};
	LerpValue = std::clamp(LerpValue, 0.f, 1.f);
	//new move speed
	float const MoveSpeed{LerpValue * MaxSpeed};
	Agent.SetMaxLinearSpeed(MoveSpeed);
	Steering.LinearVelocity = Difference;
	
	//gets drawn slightly too high but whatever
	DrawDebugCircle(Agent.GetWorld()
		, Agent.GetActorLocation(), OuterRadius, 20, FColor::Red,
		false, -1, 0, 0, 
		FVector(0,1,0), FVector(1,0,0), false);
	
	DrawDebugCircle(Agent.GetWorld()
		, Agent.GetActorLocation(), InnerRadius, 20, FColor::Blue,
		false, -1, 0, 0, 
		FVector(0,1,0), FVector(1,0,0), false);
	return Steering;
}


SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	FVector2D const Difference = Target.Position - Agent.GetPosition();
	Agent.SetMaxLinearSpeed(0.f);
	Steering.LinearVelocity = Difference;
	//add debug rendering
	return Steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	float constexpr MaxSpeed{600.f};
	Agent.SetMaxLinearSpeed(MaxSpeed);
	FVector2D const Difference = Target.Position - Agent.GetPosition();
	double const DistanceToTarget = Difference.Length();
	double const TimeToReachTarget{Agent.GetMaxLinearSpeed() / DistanceToTarget};
	FVector2D const PredictedPosition{Target.LinearVelocity * TimeToReachTarget + Target.Position};
	Steering.LinearVelocity = PredictedPosition - Agent.GetPosition();
	//show a cool thing
	//add debug rendering
	return Steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	float constexpr MaxSpeed{600.f};
	Agent.SetMaxLinearSpeed(MaxSpeed);
	FVector2D const Difference = Target.Position - Agent.GetPosition();
	double const DistanceToTarget = Difference.Length();
	double const TimeToReachTarget{Agent.GetMaxLinearSpeed() / DistanceToTarget};
	FVector2D const PredictedPosition{Target.LinearVelocity * TimeToReachTarget + Target.Position};
	Steering.LinearVelocity = Agent.GetPosition() - PredictedPosition;
	//show a cool thing
	//add debug rendering
	return Steering;
}