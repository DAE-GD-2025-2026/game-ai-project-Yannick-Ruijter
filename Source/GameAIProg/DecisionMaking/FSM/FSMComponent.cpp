// Fill out your copyright notice in the Description page of Project Settings.


#include "FSMComponent.h"


void GameAI::FSM::FSM::SetCurrentState(State const* state)
{
	for (int i = 0; i < m_States.size(); ++i)
	{
		if (m_States[i].get() == state)
		{
			m_CurrentStateIndex = i;
			m_States[i]->m_OnEnter(AIContr);
			return;
		}
	}
}

void UFSMComponent::AddState(std::unique_ptr<GameAI::FSM::State>&& NewState)
{
	FSMInstance->AddState(std::move(NewState));
}

void UFSMComponent::AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To, std::function<bool(AGameAIController*)> EvalFunc)
{
	FSMInstance->AddTransition(From, To, EvalFunc);
}

// Called when the game starts
void UFSMComponent::BeginPlay()
{
	Super::BeginPlay();
}


UFSMComponent::UFSMComponent(AGameAIController* AIController)
	:AIController{AIController}
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// TODO Setup FSM
	
}

// Called every frame
void UFSMComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!bIsRunning) return;
	auto currentState = FSMInstance->GetCurrentState();
	currentState->Tick(DeltaTime);
	for (auto const& transition: FSMInstance->GetTransitions())
	{
		if (transition->m_FromState == currentState)
		{
			if (transition->m_EvalFunc()) 
				FSMInstance->SetCurrentState(transition->m_ToState);
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

