// Fill out your copyright notice in the Description page of Project Settings.


#include "FSMComponent.h"


// Sets default values for this component's properties
UFSMComponent::UFSMComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// TODO Setup FSM
}


void UFSMComponent::AddState(std::unique_ptr<GameAI::FSM::State>&& NewState)
{
	FSMInstance->AddState(std::move(NewState));
}

void UFSMComponent::AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To, std::function<bool()> EvalFunc)
{
	FSMInstance->AddTransition(From, To, EvalFunc);
}

// Called when the game starts
void UFSMComponent::BeginPlay()
{
	Super::BeginPlay();
}


// Called every frame
void UFSMComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!bIsRunning) return;
	auto currentState = FSMInstance->GetCurrentState();
	//currentState->Tick(DeltaTime);
	for (auto const& transition: FSMInstance->GetTransitions())
	{
		if (transition->m_FromState == currentState)
		{
			if (transition->m_EvalFunc()) FSMInstance->SetCurrentState(transition->m_ToState);
			break;
		}
	}
}

void UFSMComponent::StartLogic()
{
	Super::StartLogic();
	bIsRunning = true;
}

void UFSMComponent::StopLogic(const FString& Reason)
{
	//somewhere do something with the reason
	bIsRunning = false;
	LastStopReason = Reason;
}

bool UFSMComponent::IsRunning() const
{
	return bIsRunning;
}

