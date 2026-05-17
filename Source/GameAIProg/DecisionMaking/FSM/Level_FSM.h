// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <memory>

#include "CoreMinimal.h"
#include "DecisionMaking/GameAIController.h"
#include "Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"
#include "Shared/Level_Base.h"
#include "Level_FSM.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_FSM : public ALevel_Base
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ALevel_FSM();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	UPROPERTY()
	ASteeringAgent* Agent{nullptr};
	UPROPERTY()
	ASteeringAgent* ThiefAgent{nullptr};
	
	std::unique_ptr<PathFollow> FSMPathFollow{nullptr};
	std::unique_ptr<Wander> FSMWander{nullptr};
	std::unique_ptr<Seek> FSMSeek{nullptr};
	std::unique_ptr<Seek> ThiefSeek{nullptr};
	
	template <typename DataType, typename RawData>
	static void AddToBlackBoard(AGameAIController* AIController, FName const& name, RawData* object);
	static void AddFloatToBlackBoard(AGameAIController* AIController, FName const& name, float value);
	
	void SetupPerception(AGameAIController* AIController);
	void SetupAgents(AGameAIController* AIController);
};
