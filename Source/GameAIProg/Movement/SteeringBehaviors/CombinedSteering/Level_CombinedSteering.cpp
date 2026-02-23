
#include "Level_CombinedSteering.h"
#include "imgui.h"
#include <format>


// Sets default values
ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();
	AddAgent(false);
	AddAgent(true, {BehaviorTypes::Evade, BehaviorTypes::Wander});
}

void ALevel_CombinedSteering::BeginDestroy()
{
	Super::BeginDestroy();

}

// Called every frame
void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	FTargetData TargetData{SteeringAgents[0].Agent->GetPosition()};
	SteeringAgents[1].SelectedTarget = 0;
	
#pragma region UI
	//UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	
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
		ImGui::Spacing();
	
		ImGui::Text("Flocking");
		ImGui::Spacing();
		ImGui::Spacing();
	
		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
   // TODO: Handle the debug rendering of your agents here :)
		}
		ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
		if (TrimWorld->bShouldTrimWorld)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
				TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
				[this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
		}
		
		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Spacing();
		
		if (ImGui::Button("Add Agent"))
			AddAgent(false);
		
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();
	
		for (int i = 0; auto const& chosenBehaviour: SteeringAgents[0].SelectedBehaviors)
		{
			std::string ChosenBehaviorName = "";
			switch (static_cast<BehaviorTypes>(chosenBehaviour))
			{
			case BehaviorTypes::Seek:
				ChosenBehaviorName = "Seek";
				break;
			case BehaviorTypes::Arrive:
				ChosenBehaviorName = "Arrive";
				break;
			case BehaviorTypes::Wander:
				ChosenBehaviorName = "Wander";
				break;
			case BehaviorTypes::Face:
				ChosenBehaviorName = "Face";
				break;
			case BehaviorTypes::Evade:
            	ChosenBehaviorName = "Evade";
				break;
			case BehaviorTypes::Flee:
				ChosenBehaviorName = "Flee";
				break;
			case BehaviorTypes::Pursuit:
				ChosenBehaviorName = "Pursuit";
				break;
			}
			ImGuiHelpers::ImGuiSliderFloatWithSetter(ChosenBehaviorName.c_str(),
			static_cast<BlendedSteering*>(SteeringAgents[0].Behavior.get())->GetWeightedBehaviorsRef()[i].Weight, 0.f, 1.f,
			[this, i](float InVal) { static_cast<BlendedSteering*>(SteeringAgents[0].Behavior.get())->GetWeightedBehaviorsRef()[i].Weight = InVal; }, "%.2f");
			i++;
		}
			
		 /*ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
		 	static_cast<BlendedSteering*>(SteeringAgents[0].Behavior.get())->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
		 	[this](float InVal) { static_cast<BlendedSteering*>(SteeringAgents[0].Behavior.get())->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");
		
		 ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
		 static_cast<BlendedSteering*>(SteeringAgents[0].Behavior.get())->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f,
		 [this](float InVal) { static_cast<BlendedSteering*>(SteeringAgents[0].Behavior.get())->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");*/
	
		//End
		ImGui::End();
	}
#pragma endregion
	for (ImGui_Agent& a : SteeringAgents)
	{
		if (a.Agent)
		{
			UpdateTarget(a);
		}
	}
 // TODO: implement Make sure to also evade the wanderer
}


bool ALevel_CombinedSteering::AddAgent(bool HasPrioritySteering, const std::vector<BehaviorTypes>& Behaviors, bool AutoOrient)
{
	ImGui_Agent ImGuiAgent = {};
	ImGuiAgent.Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{0,0,90}, FRotator::ZeroRotator);
	
	if (IsValid(ImGuiAgent.Agent))
	{
		for (auto const& Behavior : Behaviors)
			ImGuiAgent.SelectedBehaviors.push_back(static_cast<int>(Behavior));
		ImGuiAgent.SelectedTarget = -1; // Mouse
		
		SetAgentBehavior(ImGuiAgent, HasPrioritySteering);

		SteeringAgents.push_back(std::move(ImGuiAgent));
		
		RefreshTargetLabels();

		return true;
	}

	return false;
}

void ALevel_CombinedSteering::SetAgentBehavior(ImGui_Agent& Agent, bool HasPrioritySteering)
{	
	Agent.Behavior.reset();
	
	float BehaviorWeight = 1.f / Agent.SelectedBehaviors.size();
	std::vector<BlendedSteering::WeightedBehavior> WeightedBehaviors{};
	std::vector<ISteeringBehavior*> Behaviors{};
	for (auto const& BehaviorType: Agent.SelectedBehaviors)
	{
		switch (static_cast<BehaviorTypes>(BehaviorType))
		{
		case BehaviorTypes::Seek:
			WeightedBehaviors.push_back({new Seek(), BehaviorWeight});
			Behaviors.push_back(new Seek{});
			break;
		case BehaviorTypes::Flee:
			WeightedBehaviors.push_back({new Flee(), BehaviorWeight});
			Behaviors.push_back(new Flee{});
			break;
		case BehaviorTypes::Arrive:
			WeightedBehaviors.push_back({new Arrive(), BehaviorWeight});
			Behaviors.push_back(new Arrive{});
			break;
		case BehaviorTypes::Face:
			WeightedBehaviors.push_back({new Face(), BehaviorWeight});
			Behaviors.push_back(new Face{});
			break;
		case BehaviorTypes::Evade:
			WeightedBehaviors.push_back({new Evade(), BehaviorWeight});
			Behaviors.push_back(new Evade{});
			break;
		case BehaviorTypes::Pursuit:
			WeightedBehaviors.push_back({new Pursuit(), BehaviorWeight});
			Behaviors.push_back(new Pursuit{});
			break;
		case BehaviorTypes::Wander:
			WeightedBehaviors.push_back({new Wander(), BehaviorWeight});
			Behaviors.push_back(new Wander{});
			break;
		default:
			assert(false); // Incorrect Agent Behavior gotten during SetAgentBehavior()	
		}
	}
	if (HasPrioritySteering) Agent.Behavior = std::make_unique<PrioritySteering>(Behaviors);
	else Agent.Behavior = std::make_unique<BlendedSteering>(WeightedBehaviors);
	
	UpdateTarget(Agent);
	
	Agent.Agent->SetSteeringBehavior(Agent.Behavior.get());
}

void ALevel_CombinedSteering::RefreshTargetLabels()
{
	TargetLabels.clear();
	
	TargetLabels.push_back("Mouse");
	for (int i{0}; i < SteeringAgents.size(); ++i)
	{
		TargetLabels.push_back(std::format("Agent {}", i));
	}
}

void ALevel_CombinedSteering::UpdateTarget(ImGui_Agent& Agent)
{
	// Note: MouseTarget position is updated via Level BP every click
	
	bool const bUseMouseAsTarget = Agent.SelectedTarget < 0;
	if (!bUseMouseAsTarget)
	{
		ASteeringAgent* const TargetAgent = SteeringAgents[Agent.SelectedTarget].Agent;

		FTargetData Target;
		Target.Position = TargetAgent->GetPosition();
		Target.Orientation = TargetAgent->GetRotation();
		Target.LinearVelocity = TargetAgent->GetLinearVelocity();
		Target.AngularVelocity = TargetAgent->GetAngularVelocity();

		Agent.Behavior->SetTarget(Target);
	}
	else
	{
		Agent.Behavior->SetTarget(MouseTarget);
	}
}

void ALevel_CombinedSteering::RefreshAgentTargets(unsigned int IndexRemoved)
{
	for (UINT i = 0; i < SteeringAgents.size(); ++i)
	{
		if (i >= IndexRemoved)
		{
			auto& Agent = SteeringAgents[i];
			if (Agent.SelectedTarget == IndexRemoved || i  == Agent.SelectedTarget)
			{
				--Agent.SelectedTarget;
			}
		}
	}
}
