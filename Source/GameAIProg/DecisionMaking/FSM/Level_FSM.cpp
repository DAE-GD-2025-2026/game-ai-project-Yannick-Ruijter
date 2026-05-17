// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_FSM.h"

#include "FSMComponent.h"
#include "UBlackBoardKeyType_PathFollow.h"
#include "UBlackBoardKeyType_Seek.h"
#include "UBlackBoardKeyType_Wander.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "BehaviorTree/BlackboardData.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Float.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Object.h"
#include "DecisionMaking/GameAIController.h"
#include "Perception/AIPerceptionComponent.h"
#include "Perception/AISenseConfig_Sight.h"


// Sets default values
ALevel_FSM::ALevel_FSM()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_FSM::BeginPlay()
{
	Super::BeginPlay();
	
	ThiefAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass,
	FVector{700,0,90}, FRotator::ZeroRotator);
	ThiefAgent->SetDebugRenderingEnabled(true);
	ThiefSeek = std::make_unique<Seek>();
	ThiefAgent->SetSteeringBehavior(ThiefSeek.get());
	ThiefSeek->SetTarget(MouseTarget);
	
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, 
	FVector{0,0,90}, FRotator::ZeroRotator);
	Agent->SetDebugRenderingEnabled(false);
	
	if (AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController()))
	{
		SetupPerception(AIController);
		SetupAgents(AIController);
	}
	
}

void ALevel_FSM::AddFloatToBlackBoard(AGameAIController* AIController, FName const& name, float value)
{
	FBlackboardEntry Entry;
	Entry.EntryName = name;
	Entry.KeyType = NewObject<UBlackboardKeyType_Float>();
	AIController->FSMBlackboardAsset->Keys.Add(Entry);
	AIController->GetBlackboardComponent()->SetValueAsFloat(name, value);
}

void ALevel_FSM::SetupPerception(AGameAIController* AIController)
{
	if (auto perception = AIController->GetPerceptionComponent(); perception != nullptr)
	{
		UAISenseConfig_Sight* SightConfig = CreateDefaultSubobject<UAISenseConfig_Sight>("SightConfig");
		SightConfig->SightRadius = 500.f;
		SightConfig->LoseSightRadius = 600.f;
		SightConfig->PeripheralVisionAngleDegrees = 60.f;
		SightConfig->SetMaxAge(5.f);
		SightConfig->DetectionByAffiliation.bDetectEnemies = true;
		SightConfig->DetectionByAffiliation.bDetectNeutrals = true;
		SightConfig->DetectionByAffiliation.bDetectFriendlies = true;

		perception->ConfigureSense(*SightConfig);
		perception->SetDominantSense(SightConfig->GetSenseImplementation());
	}
}

void ALevel_FSM::SetupAgents(AGameAIController* AIController)
{
	if (UFSMComponent* FSM = Cast<UFSMComponent>(AIController->GetBrainComponent()))
	{
		FSMPathFollow = std::make_unique<PathFollow>(true);
		std::vector path{
			FVector2D{275, 600},
			FVector2D{870, 600},
			FVector2D{870, 1200},
			FVector2D{-600, 1200},
			FVector2D{-600, -430},
			FVector2D{275, -430},
		};
		FSMPathFollow->SetPath(path);
		FSMSeek = std::make_unique<Seek>();
		FSMWander = std::make_unique<Wander>();
		
		auto RegisterKey = [&](FName name, UBlackboardKeyType* keyType)
		{
			FBlackboardEntry Entry;
			Entry.EntryName = name;
			Entry.KeyType = keyType;
			AIController->FSMBlackboardAsset->Keys.Add(Entry);
		};

		RegisterKey("ThiefAgent",NewObject<UBlackboardKeyType_Object>());
		RegisterKey("Agent",NewObject<UBlackboardKeyType_Object>());
		RegisterKey("Seek",NewObject<UBlackBoardKeyType_Seek>());
		RegisterKey("Wander",NewObject<UBlackBoardKeyType_Wander>());
		RegisterKey("PathFollow",NewObject<UBlackBoardKeyType_PathFollow>());

		AIController->GetBlackboardComponent()->InitializeBlackboard(*AIController->FSMBlackboardAsset);

		AIController->GetBlackboardComponent()->SetValueAsObject("ThiefAgent", ThiefAgent);
		AIController->GetBlackboardComponent()->SetValueAsObject("Agent", Agent);
		AIController->GetBlackboardComponent()->SetValue<UBlackBoardKeyType_Seek>("Seek", FSMSeek.get());
		AIController->GetBlackboardComponent()->SetValue<UBlackBoardKeyType_Wander>("Wander", FSMWander.get());
		AIController->GetBlackboardComponent()->SetValue<UBlackBoardKeyType_PathFollow>("PathFollow", FSMPathFollow.get());
		Agent->SetSteeringBehavior(FSMPathFollow.get());
		
		auto patrolState = std::make_unique<GameAI::FSM::State>("Patrol", 
			[&]()
			{
				auto agent = static_cast<ASteeringAgent*>(AIController->GetBlackboardComponent()->GetValueAsObject("Agent"));
				auto steeringBehavior = AIController->GetBlackboardComponent()->GetValue<UBlackBoardKeyType_PathFollow>("PathFollow");
				agent->SetSteeringBehavior(steeringBehavior);
			}, [&](float ){});
		auto patrolStatePtr = patrolState.get();
		FSM->AddState(std::move(patrolState));     
		
		auto searchState = std::make_unique<GameAI::FSM::State>("Search", 
			[&]()
			{
				auto agent = static_cast<ASteeringAgent*>(AIController->GetBlackboardComponent()->GetValueAsObject("Agent"));
				auto steeringBehavior = AIController->GetBlackboardComponent()->GetValue<UBlackBoardKeyType_Wander>("Wander");
				agent->SetSteeringBehavior(steeringBehavior);
			}, [&](float deltaTime){});
		auto searchStatePtr = searchState.get();
		FSM->AddState(std::move(searchState));

		auto chaseState = std::make_unique<GameAI::FSM::State>("Chase", 
			[&]()
			{
				auto agent = static_cast<ASteeringAgent*>(AIController->GetBlackboardComponent()->GetValueAsObject("Agent"));
				auto steeringBehavior = AIController->GetBlackboardComponent()->GetValue<UBlackBoardKeyType_Seek>("Seek");
				agent->SetSteeringBehavior(steeringBehavior);
			}, [&](float){});
		auto chaseStatePtr = chaseState.get();
		FSM->AddState(std::move(chaseState)); 
		
		FSM->AddTransition(patrolStatePtr, chaseStatePtr, [AIController]()
		{
			auto thief = static_cast<ASteeringAgent*>(
			   AIController->GetBlackboardComponent()->GetValueAsObject("ThiefAgent"));

			if (!thief) return false;

		   UAIPerceptionComponent* Perception = AIController->GetPerceptionComponent();
		   if (!Perception) return false;

		   FActorPerceptionBlueprintInfo Info;
		   Perception->GetActorsPerception(thief, Info);

		   for (const FAIStimulus& Stimulus : Info.LastSensedStimuli)
		   {
			   if (Stimulus.Type == UAISense::GetSenseID<UAISense_Sight>() 
				   && Stimulus.WasSuccessfullySensed())
			   {
				   return true;
			   }
		   }
		   return false;
		});
		FSM->AddTransition(chaseStatePtr, searchStatePtr, [AIController]()
		{
			
			auto thief = static_cast<ASteeringAgent*>(
			   AIController->GetBlackboardComponent()->GetValueAsObject("ThiefAgent"));

			if (!thief) return false;

		   UAIPerceptionComponent* Perception = AIController->GetPerceptionComponent();
		   if (!Perception) return false;

		   FActorPerceptionBlueprintInfo Info;
		   Perception->GetActorsPerception(thief, Info);

		   for (const FAIStimulus& Stimulus : Info.LastSensedStimuli)
		   {
			   if (Stimulus.Type == UAISense::GetSenseID<UAISense_Sight>() 
				   && !Stimulus.WasSuccessfullySensed())
			   {
				   return true;
			   }
		   }
		   return false;
		});
		FSM->AddTransition(searchStatePtr, chaseStatePtr, [AIController]()
		{
			auto thief = static_cast<ASteeringAgent*>(
			   AIController->GetBlackboardComponent()->GetValueAsObject("ThiefAgent"));

			if (!thief) return false;

		   UAIPerceptionComponent* Perception = AIController->GetPerceptionComponent();
		   if (!Perception) return false;

		   FActorPerceptionBlueprintInfo Info;
		   Perception->GetActorsPerception(thief, Info);

		   for (const FAIStimulus& Stimulus : Info.LastSensedStimuli)
		   {
			   if (Stimulus.Type == UAISense::GetSenseID<UAISense_Sight>() 
				   && Stimulus.WasSuccessfullySensed())
			   {
				   return true;
			   }
		   }
		   return false;
		});
		FSM->AddTransition(searchStatePtr, patrolStatePtr, [AIController]()
		{
			auto thief = static_cast<ASteeringAgent*>(
				AIController->GetBlackboardComponent()->GetValueAsObject("ThiefAgent"));

			if (!thief) return false;

			UAIPerceptionComponent* Perception = AIController->GetPerceptionComponent();
			if (!Perception) return false;

			FActorPerceptionBlueprintInfo Info;
			Perception->GetActorsPerception(thief, Info);

			return Info.LastSensedStimuli.IsEmpty();
		});
		
		AIController->RunFiniteStateMachine();
	}
}

template <typename DataType, typename RawData>
void ALevel_FSM::AddToBlackBoard(AGameAIController* AIController, FName const& name, RawData* object)
{
	FBlackboardEntry Entry;
	Entry.EntryName = name;
	Entry.KeyType = NewObject<DataType>();
	AIController->FSMBlackboardAsset->Keys.Add(Entry);
	AIController->GetBlackboardComponent()->SetValue<DataType>(name, object); 
}

// Called every frame
void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	ThiefSeek->SetTarget(MouseTarget);
}

