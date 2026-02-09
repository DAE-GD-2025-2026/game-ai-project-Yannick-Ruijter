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
	
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	//show a cool thing
	//add debug rendering
	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	
	Steering.LinearVelocity = Agent.GetPosition() - Target.Position;
	//show a cool thing
	//add debug rendering
	return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	//fake max speed
	float MaxSpeed{600.f};
	//radia of the circle
	float OuterRadius{500.f};
	float InnerRadius{150.f};
	//difference
	FVector2D Difference{Target.Position - Agent.GetPosition()};
	//distance
	float DistanceToPoint{static_cast<float>(Difference.Length())};
	//clamped lerp value to decelerate
	float LerpValue{(DistanceToPoint - InnerRadius) / (OuterRadius - InnerRadius)};
	LerpValue = std::clamp(LerpValue, 0.f, 1.f);
	//new move speed
	float MoveSpeed{LerpValue * MaxSpeed};
	Agent.SetMaxLinearSpeed(MoveSpeed);
	Steering.LinearVelocity = Difference;
	
	//gets draw slightly too high but whatever
	DrawDebugCircle(Agent.GetWorld()
		, Agent.GetActorLocation(), OuterRadius, 20, FColor::Red,
		false, -1, 0, 0, 
		FVector(0,1,0), FVector(1,0,0), false);
	
	DrawDebugCircle(Agent.GetWorld()
		, Agent.GetActorLocation(), InnerRadius, 20, FColor::Blue,
		false, -1, 0, 0, 
		FVector(0,1,0), FVector(1,0,0), false);
	//DrawCircle(, Agent.GetPosition(), Radius, Radius, FColor(255, 255, 255), OuterRadius, 20);
	//show a cool thing
	//add debug rendering
	return Steering;
}


SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	//Steering.AngularVelocity = UE::Geometry::Lerp(Agent.GetRotation());
	//show a cool thing
	//add debug rendering
	return Steering;
}