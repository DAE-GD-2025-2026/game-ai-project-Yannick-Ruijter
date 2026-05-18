// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_FSM.h"

#include "FSMComponent.h"
#include "UBlackBoardKeyType_PathFollow.h"
#include "UBlackBoardKeyType_Seek.h"
#include "UBlackBoardKeyType_Wander.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "BehaviorTree/BlackboardData.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Bool.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Float.h"
#include "BehaviorTree/Blackboard/BlackboardKeyType_Object.h"
#include "DecisionMaking/GameAIController.h"
#include "Perception/AIPerceptionComponent.h"
#include "Perception/AIPerceptionStimuliSourceComponent.h"
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

	UAIPerceptionStimuliSourceComponent* StimuliSource = Cast<UAIPerceptionStimuliSourceComponent>(
	ThiefAgent->AddComponentByClass(UAIPerceptionStimuliSourceComponent::StaticClass(), false, FTransform::Identity, false));
	if (StimuliSource)
	{
		StimuliSource->RegisterForSense(UAISense_Sight::StaticClass());
		StimuliSource->RegisterWithPerceptionSystem();
	}

	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass,
		FVector{0,0,90}, FRotator::ZeroRotator);
	Agent->SetDebugRenderingEnabled(false);

	if (AGameAIController* AIController = Cast<AGameAIController>(Agent->GetController()))
	{
		SetupPerception(AIController);
		SetupAgents(AIController);
	}
}

void ALevel_FSM::SetupPerception(AGameAIController* AIController)
{
	UAIPerceptionComponent* Perception = Cast<UAIPerceptionComponent>(
		AIController->AddComponentByClass(UAIPerceptionComponent::StaticClass(), 
		false, FTransform::Identity, false));
	AIController->SetPerceptionComponent(*Perception);
	
	UAISenseConfig_Sight* SightConfig = NewObject<UAISenseConfig_Sight>(Perception, "SightConfig");
	SightConfig->SightRadius = 500.f;
	SightConfig->LoseSightRadius = 600.f;
	SightConfig->PeripheralVisionAngleDegrees = 60.f;
	SightConfig->SetMaxAge(5.f);
	SightConfig->DetectionByAffiliation.bDetectEnemies = true;
	SightConfig->DetectionByAffiliation.bDetectNeutrals = true;
	SightConfig->DetectionByAffiliation.bDetectFriendlies = true;
	SightConfig->AutoSuccessRangeFromLastSeenLocation = 100.f;
	
	Perception->ConfigureSense(*SightConfig);
	Perception->SetDominantSense(SightConfig->GetSenseImplementation());
	Perception->RequestStimuliListenerUpdate();
	
	Perception->OnTargetPerceptionUpdated.AddDynamic(this, &ALevel_FSM::OnTargetPerceptionUpdated);
}

void ALevel_FSM::OnTargetPerceptionUpdated(AActor* Actor, FAIStimulus Stimulus)
{
	if (Stimulus.Type != UAISense::GetSenseID<UAISense_Sight>()) return;

	// Ignore "lost" signals if we never successfully sensed to begin with
	bool bWasSensed = Stimulus.WasSuccessfullySensed();
	bool bCurrentlySpotted = Cast<AGameAIController>(Agent->GetController())
		->GetBlackboardComponent()->GetValueAsBool("ThiefSpotted");

	if (!bWasSensed && !bCurrentlySpotted) return; // nothing changed, ignore

	Cast<AGameAIController>(Agent->GetController())
		->GetBlackboardComponent()->SetValueAsBool("ThiefSpotted", bWasSensed);
	
	DrawDebugBox(GetWorld(), Stimulus.StimulusLocation, FVector{100.f, 100.f, 100.f}, FColor::Cyan);
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
		InitBlackboard(AIController);
		Agent->SetSteeringBehavior(FSMPathFollow.get());
		
		auto patrolState = std::make_unique<GameAI::FSM::State>("Patrol", 
			[AIController]()
			{
				auto agent = static_cast<ASteeringAgent*>(AIController->GetBlackboardComponent()->GetValueAsObject("Agent"));
				auto steeringBehavior = AIController->GetBlackboardComponent()->GetValue<UBlackBoardKeyType_PathFollow>("PathFollow");
				agent->SetSteeringBehavior(steeringBehavior);
			}, [&](float ){});
		auto patrolStatePtr = patrolState.get();
		FSM->AddState(std::move(patrolState));     
		
		auto searchState = std::make_unique<GameAI::FSM::State>("Search", 
			[AIController]()
			{
				auto agent = static_cast<ASteeringAgent*>(AIController->GetBlackboardComponent()->GetValueAsObject("Agent"));
				auto steeringBehavior = AIController->GetBlackboardComponent()->GetValue<UBlackBoardKeyType_Wander>("Wander");
				agent->SetSteeringBehavior(steeringBehavior);
			}, [&](float deltaTime){});
		auto searchStatePtr = searchState.get();
		FSM->AddState(std::move(searchState));

		auto chaseState = std::make_unique<GameAI::FSM::State>("Chase", 
			[AIController]()
			{
				auto agent = static_cast<ASteeringAgent*>(AIController->GetBlackboardComponent()->GetValueAsObject("Agent"));
				auto steeringBehavior = AIController->GetBlackboardComponent()->GetValue<UBlackBoardKeyType_Seek>("Seek");
				auto thief = static_cast<ASteeringAgent*>(AIController->GetBlackboardComponent()->GetValueAsObject("ThiefAgent"));
				FTargetData targetData{thief->GetPosition()};
				steeringBehavior->SetTarget(targetData);
				agent->SetSteeringBehavior(steeringBehavior);
			}, [AIController](float)
			{
				auto steeringBehavior = AIController->GetBlackboardComponent()->GetValue<UBlackBoardKeyType_Seek>("Seek");
				auto thief = static_cast<ASteeringAgent*>(AIController->GetBlackboardComponent()->GetValueAsObject("ThiefAgent"));
				FTargetData targetData{thief->GetPosition()};
				steeringBehavior->SetTarget(targetData);
			});
		auto chaseStatePtr = chaseState.get();
		FSM->AddState(std::move(chaseState)); 
		
		FSM->AddTransition(patrolStatePtr, chaseStatePtr, [AIController = Cast<AGameAIController>(Agent->GetController())]()
		{
			return AIController->GetBlackboardComponent()->GetValueAsBool("ThiefSpotted");
		});
		FSM->AddTransition(chaseStatePtr, searchStatePtr, [AIController = Cast<AGameAIController>(Agent->GetController())]()
		{
			return !AIController->GetBlackboardComponent()->GetValueAsBool("ThiefSpotted");
		});
		FSM->AddTransition(searchStatePtr, chaseStatePtr, [AIController = Cast<AGameAIController>(Agent->GetController())]()
		{
			return AIController->GetBlackboardComponent()->GetValueAsBool("ThiefSpotted");
		});
		FSM->AddTransition(searchStatePtr, patrolStatePtr, [&](){return TransitionToPatrol();});
		
		AIController->RunFiniteStateMachine();
		FSM->StartLogic();
	}
}

bool ALevel_FSM::TransitionToPatrol()
{
	auto AIController = Cast<AGameAIController>(Agent->GetController());
	
	auto thief = static_cast<ASteeringAgent*>(
		AIController->GetBlackboardComponent()->GetValueAsObject("ThiefAgent"));

	if (!thief) return false;

	UAIPerceptionComponent* Perception = AIController->GetPerceptionComponent();
	if (!Perception) return false;

	FActorPerceptionBlueprintInfo Info;
	Perception->GetActorsPerception(thief, Info);

	return Info.LastSensedStimuli.IsEmpty() || Info.LastSensedStimuli[0].IsExpired();
}

void ALevel_FSM::InitBlackboard(AGameAIController* AIController)
{
	auto RegisterKey = [&](FName name, UBlackboardKeyType* keyType)
	{
		FBlackboardEntry Entry;
		Entry.EntryName = name;
		Entry.KeyType = keyType;
		AIController->FSMBlackboardAsset->Keys.Add(Entry);
	};

	RegisterKey("ThiefAgent", NewObject<UBlackboardKeyType_Object>());
	RegisterKey("Agent", NewObject<UBlackboardKeyType_Object>());
	RegisterKey("Seek", NewObject<UBlackBoardKeyType_Seek>());
	RegisterKey("Wander", NewObject<UBlackBoardKeyType_Wander>());
	RegisterKey("PathFollow", NewObject<UBlackBoardKeyType_PathFollow>());
	RegisterKey("ThiefSpotted", NewObject<UBlackboardKeyType_Bool>());

	AIController->GetBlackboardComponent()->InitializeBlackboard(*AIController->FSMBlackboardAsset);

	AIController->GetBlackboardComponent()->SetValueAsObject("ThiefAgent", ThiefAgent);
	AIController->GetBlackboardComponent()->SetValueAsObject("Agent", Agent);
	AIController->GetBlackboardComponent()->SetValue<UBlackBoardKeyType_Seek>("Seek", FSMSeek.get());
	AIController->GetBlackboardComponent()->SetValue<UBlackBoardKeyType_Wander>("Wander", FSMWander.get());
	AIController->GetBlackboardComponent()->SetValue<UBlackBoardKeyType_PathFollow>("PathFollow", FSMPathFollow.get());
	AIController->GetBlackboardComponent()->SetValueAsBool("ThiefSpotted", false);
}

// Called every frame
void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	ThiefSeek->SetTarget(MouseTarget);
}

