#include "Flock.h"

#include <string>

#include "FieldNotificationDeclaration.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


Flock::Flock(
	UWorld* pworld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentEvade,
	bool bTrimWorld)
	: pWorld{pworld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentEvade}
	,pSeparationBehavior{std::make_unique<Separation>(this)}
	,pCohesionBehavior{std::make_unique<Cohesion>(this)}
	,pVelMatchBehavior{std::make_unique<Alignment>(this)}
    ,pSeekBehavior{std::make_unique<Seek>()}
    ,pWanderBehavior{std::make_unique<Wander>()}
    ,pEvadeBehavior{std::make_unique<Evade>()}
{
	Neighbors.SetNum(FlockSize);
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	pPartitionedSpace = std::make_unique<CellSpace>(pWorld, WorldSize * 2, WorldSize * 2, 10, 10, FlockSize);
#endif
	
	if (pAgentToEvade == nullptr)
	{
		pAgentToEvade = pWorld->SpawnActor<ASteeringAgent>(AgentClass);
		std::vector<BlendedSteering::WeightedBehavior> behaviors{{pSeekBehavior.get(), 0.5f}, {pWanderBehavior.get(), 0.5f}};
		pEvadeAgentBlendedSteering = std::make_unique<BlendedSteering>(behaviors);
		pAgentToEvade->SetSteeringBehavior(pEvadeAgentBlendedSteering.get());
	}
	std::vector<BlendedSteering::WeightedBehavior> Behaviors{{pCohesionBehavior.get(), 0.3f}};
	Behaviors.push_back({pSeparationBehavior.get(), 0.5f});
	Behaviors.push_back({pVelMatchBehavior.get(), 0.73f});
	pFlockBlendedSteering = std::make_unique<BlendedSteering>(Behaviors);
	std::vector<ISteeringBehavior*> SteeringBehaviors{pEvadeBehavior.get(), pFlockBlendedSteering.get()};
	pPrioritySteering = std::make_unique<PrioritySteering>(SteeringBehaviors);
	for (int i = 0; i < FlockSize; ++i)
	{
		float const RandomX = FMath::RandRange(-WorldSize, WorldSize);
		float const RandomY = FMath::RandRange(-WorldSize, WorldSize);
		FVector Location{RandomX, RandomY, 10.f};
		if (ASteeringAgent* Agent = pWorld->SpawnActor<ASteeringAgent>(AgentClass, Location, FRotator::ZeroRotator))
		{
#ifdef GAMEAI_USE_SPACE_PARTITIONING
			pPartitionedSpace->AddAgent(*Agent);
#endif
			Agent->SetActorTickEnabled(false);
			Agent->SetSteeringBehavior(pPrioritySteering.get());
			Agents.Add(Agent);
			OldPositions.Add({Location.X, Location.Y});
		}
	}
}

Flock::~Flock()
{
}

void Flock::Tick(float DeltaTime)
{
	FTargetData Target;
	Target.Position = pAgentToEvade->GetPosition();
	Target.Orientation = pAgentToEvade->GetRotation();
	Target.LinearVelocity = pAgentToEvade->GetLinearVelocity();
	Target.AngularVelocity = pAgentToEvade->GetAngularVelocity();
	
	pEvadeBehavior->SetTarget(Target);
	for (int i{0}; i < Agents.Num(); ++i)
	{
		Agents[i]->Tick(DeltaTime);
#ifdef GAMEAI_USE_SPACE_PARTITIONING
		pPartitionedSpace->UpdateAgentCell(*Agents[i], OldPositions[i]);
		OldPositions[i] = Agents[i]->GetPosition();
#endif
	}
}

void Flock::RenderDebug()
{
	RenderNeighborhood();
	
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	if (DebugRenderPartitions)
	{
		pPartitionedSpace->RenderCells();	
	}	
#endif
	pSeekBehavior->SetDebugRendering(DebugRenderSteering);
	pEvadeBehavior->SetDebugRendering(DebugRenderSteering);
	pWanderBehavior->SetDebugRendering(DebugRenderSteering);
	pCohesionBehavior->SetDebugRendering(DebugRenderSteering);
	pSeparationBehavior->SetDebugRendering(DebugRenderSteering);
	pVelMatchBehavior->SetDebugRendering(DebugRenderSteering);
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Checkbox("Debug Neighborhood: ", &DebugRenderNeighborhood);
		ImGui::Checkbox("Debug Steering: ", &DebugRenderSteering);
		ImGui::Checkbox("Debug Partitions: ", &DebugRenderPartitions);
		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();
		
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek: ",pEvadeAgentBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f
			, [&] (float InVal) {pEvadeAgentBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal;});
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander: ",pEvadeAgentBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f
			, [&] (float InVal) {pEvadeAgentBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal;});
		ImGui::Spacing();
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Cohesion: ",pFlockBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f
			, [&] (float InVal) {pFlockBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal;});
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Separation: ",pFlockBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f
			, [&] (float InVal) {pFlockBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal;});
		ImGuiHelpers::ImGuiSliderFloatWithSetter("VelocityMatch: ",pFlockBlendedSteering->GetWeightedBehaviorsRef()[2].Weight, 0.f, 1.f
			, [&] (float InVal) {pFlockBlendedSteering->GetWeightedBehaviorsRef()[2].Weight = InVal;});
		
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	if (DebugRenderNeighborhood)
	{
		RegisterNeighbors(Agents[0]);
		DrawDebugCircle(pWorld, FVector{Agents[0]->GetPosition(), 10.f}, NeighborhoodRadius, 10, FColor::Magenta
				, false, -1, 0, 0, FVector(0, 1, 0), FVector(1, 0, 0));
	
		for (int i{0}; i < NrOfNeighbors; ++i)
		{
			DrawDebugCircle(pWorld, FVector{Neighbors[i]->GetPosition(), 10.f}, 50.f, 10, FColor::Emerald
				, false, -1, 0, 0, FVector(0, 1, 0), FVector(1, 0, 0));
		}
	}
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	for (NrOfNeighbors = 0; auto const& Agent : Agents)
	{
		if (Agent == pAgent) continue;
		if ((pAgent->GetPosition() - Agent->GetPosition()).Length() < NeighborhoodRadius)
		{
			Neighbors[NrOfNeighbors] = Agent;
			++NrOfNeighbors;
		}
	}
}
#else
void Flock::RegisterNeighbors(ASteeringAgent* const Agent)
{
	pPartitionedSpace->RegisterNeighbors(*Agent, NeighborhoodRadius);

	NrOfNeighbors = pPartitionedSpace->GetNrOfNeighbors();
	const TArray<ASteeringAgent*>& partitionNeighbors = pPartitionedSpace->GetNeighbors();

	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		Neighbors[i] = partitionNeighbors[i];
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;
	
	for (int i = 0; i < NrOfNeighbors; ++i)
		avgPosition += Neighbors[i]->GetPosition();
	
	if (NrOfNeighbors == 0) return FVector2D::ZeroVector;
	avgPosition /= NrOfNeighbors;
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;
	
	for (int i = 0; i < NrOfNeighbors; ++i)
		avgVelocity += Neighbors[i]->GetLinearVelocity();
	
	if (NrOfNeighbors == 0) return FVector2D::ZeroVector;
	avgVelocity /= NrOfNeighbors;

	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	
	pSeekBehavior->SetTarget(Target);
}

