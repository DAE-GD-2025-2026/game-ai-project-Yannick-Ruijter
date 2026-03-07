#include "SteeringBehaviors.h"

#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h"

//SEEK
//*******
// DONE: Do the Week01 assignment :^)

void ISteeringBehavior::SetTarget(const FTargetData& NewTarget)
{
	Target = NewTarget;
}
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	Steering.IsValid = true;
	float constexpr MaxSpeed{600.f};
	Agent.SetMaxLinearSpeed(MaxSpeed);
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	if (FMath::IsNearlyZero(Steering.LinearVelocity.Length())) /*set valid state*/Steering.IsValid = false;
	Steering.LinearVelocity.Normalize();
	if (DebugRenderingEnabled)
	{
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 10.f)
			, FVector(Agent.GetPosition() + Steering.LinearVelocity * 100, 10.f), FColor::Cyan);
	}
	//show a cool thing
	//add debug rendering
	return Steering;
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	float constexpr MaxSpeed{600.f};
	Agent.SetMaxLinearSpeed(MaxSpeed);
	float constexpr CircleRadius{100.f};
	float constexpr CircleOffset{20.f};
	FRotator Rotation{Agent.GetRotation()};
	float Temp = Agent.GetRotation();
	FVector const CircleCenter{FVector{Agent.GetPosition(), 10.f} + Agent.GetActorForwardVector() * (CircleOffset + CircleRadius)};
	double const RandomAngle{UKismetMathLibrary::RandomFloatInRange(0, 2 * PI)};
	FVector2D RandomPoint{
		CircleCenter.X + CircleRadius * cos(RandomAngle),
		CircleCenter.Y + CircleRadius * sin(RandomAngle)
	};
	
	Steering.LinearVelocity = RandomPoint - Agent.GetPosition();
	Steering.LinearVelocity.Normalize();
	//show a cool thing
	//add debug rendering
	if (DebugRenderingEnabled)
	{
		DrawDebugCircle(Agent.GetWorld()
			, CircleCenter, CircleRadius, 20, FColor::Red,
			false, -1, 0, 0, 
			FVector(0,1,0), FVector(1,0,0), false);
	}
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
	float const InnerRadius{m_Radius};
	//difference
	FVector2D Difference{Target.Position - Agent.GetPosition()};
	//distance
	float const DistanceToPoint{static_cast<float>(Difference.Length())};
	Difference.Normalize();
	Difference *= InnerRadius;
	//clamped lerp value to decelerate
	float LerpValue{(DistanceToPoint - InnerRadius) / (OuterRadius - InnerRadius)};
	LerpValue = std::clamp(LerpValue, 0.f, 1.f);
	//new move speed
	float const MoveSpeed{LerpValue * MaxSpeed};
	Agent.SetMaxLinearSpeed(MoveSpeed);
	Steering.LinearVelocity = Difference;
	//gets drawn slightly too high but whatever
	if (DebugRenderingEnabled)
	{
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 10.f)
			, FVector(Agent.GetPosition() + Steering.LinearVelocity * LerpValue, 10.f), FColor::Cyan);
		DrawDebugCircle(Agent.GetWorld()
			, Agent.GetActorLocation(), OuterRadius, 20, FColor::Red,
			false, -1, 0, 0, 
			FVector(0,1,0), FVector(1,0,0), false);
	
		DrawDebugCircle(Agent.GetWorld()
			, Agent.GetActorLocation(), InnerRadius, 20, FColor::Blue,
			false, -1, 0, 0, 
			FVector(0,1,0), FVector(1,0,0), false);		
	}
	return Steering;
}

void Arrive::SetTargetRadius(float Radius)
{
	m_Radius = Radius;
}


SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	FVector2D Difference = Target.Position - Agent.GetPosition();
	//Difference.Normalize();
	if (Difference.IsNearlyZero())
		return Steering;
    
	const float DesiredYaw = FMath::RadiansToDegrees(FMath::Atan2(Difference.Y, Difference.X));
	const float DeltaYaw = FMath::FindDeltaAngleDegrees(Agent.GetRotation(), DesiredYaw);
    if (FMath::IsNearlyZero(DeltaYaw))	Steering.AngularVelocity = 0.f;
	else if (DeltaYaw < 0.f) Steering.AngularVelocity = -Agent.GetMaxAngularSpeed();
	else if (DeltaYaw > 0.f) Steering.AngularVelocity = Agent.GetMaxAngularSpeed();
	
	if (DebugRenderingEnabled)
	{
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 10.f), FVector(Difference, 10.f), FColor::Yellow);
		DrawDebugLine(Agent.GetWorld(), FVector(Agent.GetPosition(), 10.f)
			, FVector(Agent.GetPosition(), 10.f) + Agent.GetActorForwardVector(), FColor::Cyan);
	}
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
	Steering.IsValid = true;
	float constexpr MaxSpeed{600.f};
	Agent.SetMaxLinearSpeed(MaxSpeed);
	double constexpr StartEvadeDistance{400.f};
	double constexpr EndEvadeDistance{550.f};
	FVector2D const Difference = Agent.GetPosition() - Target.Position;
	double const DistanceToTarget = Difference.Length();
	if (DistanceToTarget > StartEvadeDistance && !m_IsEvading) Steering.IsValid = false;
	else if (DistanceToTarget > EndEvadeDistance && m_IsEvading)
	{
		Steering.IsValid = false;
		m_IsEvading = false;
	}
	else m_IsEvading = true;
	
	double const TimeToReachTarget{Agent.GetMaxLinearSpeed() / DistanceToTarget};
	FVector2D const PredictedPosition{Target.LinearVelocity * TimeToReachTarget + Target.Position};
	Steering.LinearVelocity = Agent.GetPosition() - PredictedPosition;
	//show a cool thing
	//add debug rendering
	return Steering;
}