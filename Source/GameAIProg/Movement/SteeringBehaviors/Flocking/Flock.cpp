#include "Flock.h"
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
{
	Neighbors.SetNum(15);
	if (pAgentToEvade == nullptr)
	{
		pAgentToEvade = pWorld->SpawnActor<ASteeringAgent>(AgentClass);
		pAgentToEvade->SetSteeringBehavior(new Wander());
	}
	std::vector<BlendedSteering::WeightedBehavior> Behaviors{{new Cohesion(this), 0.4f}};
	Behaviors.push_back({new Separation(this), 0.5f});
	Behaviors.push_back({new Alignment(this), 0.33f});
	pBlendedSteering = std::make_unique<BlendedSteering>(Behaviors);
	std::vector<ISteeringBehavior*> SteeringBehaviors{new Evade(), pBlendedSteering.get()};
	pPrioritySteering = std::make_unique<PrioritySteering>(SteeringBehaviors);
	for (int i = 0; i < FlockSize; ++i)
	{
		if (ASteeringAgent* Agent = pWorld->SpawnActor<ASteeringAgent>(AgentClass))
		{
			Agent->SetSteeringBehavior(pPrioritySteering.get());
			Agents.Add(Agent);
		}
	}
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
 // TODO: update the flock
 // TODO: for every agent:
  // TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
  // TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
  // TODO: trim the agent to the world
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
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

  // TODO: implement ImGUI checkboxes for debug rendering here

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

  // TODO: implement ImGUI sliders for steering behavior weights here
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
 // TODO: Debugrender the neighbors for the first agent in the flock
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	for (NrOfNeighbors = 0; auto const& Agent : Agents)
	{
		if (Agent == pAgent) continue;
		if (NeighborhoodRadius < (pAgent->GetPosition() - Agent->GetPosition()).Length())
		{
			Neighbors[NrOfNeighbors] = Agent;
			++NrOfNeighbors;
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;

	for (int i = 0; i < NrOfNeighbors; ++i)
		avgPosition += Neighbors[i]->GetPosition();
	
	avgPosition /= NrOfNeighbors;
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

	for (int i = 0; i < NrOfNeighbors; ++i)
		avgVelocity += Neighbors[i]->GetLinearVelocity();
	
	avgVelocity /= NrOfNeighbors;

	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
 // TODO: Implement
}

