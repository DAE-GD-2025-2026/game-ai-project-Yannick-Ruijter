// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "CoreMinimal.h"
#include "BrainComponent.h"
#include "Curves/BezierUtilities.h"
#include "DecisionMaking/GameAIController.h"
#include "FSMComponent.generated.h"

namespace GameAI::FSM
{
	class State
	{
	public:
		State(std::string const& name, std::function<void()>const & onEnter, std::function<void(float)>const & tick) 
		: m_StateName{name}, m_OnEnter{onEnter}, m_Tick{tick} {}
		void Tick(float DeltaTime){m_Tick(DeltaTime);};
	
		std::string m_StateName;
		std::function<void()> m_OnEnter;
		std::function<void(float)> m_Tick;
	};
	class Transition
	{
	public:
		Transition(State* From, State* To, std::function<bool()>const& EvalFunc)
			: m_FromState{From}, m_ToState{To}, m_EvalFunc{EvalFunc}
		{}
		
		State* m_FromState;
		State* m_ToState;
		std::function<bool()> m_EvalFunc;
	};
	class FSM
	{
	public:
		FSM() = default;
		void AddState(std::unique_ptr<State>&& NewState)
		{
			m_States.emplace_back(std::move(NewState));
			if (m_CurrentStateIndex == -1) m_CurrentStateIndex = 0;
		};
		void AddTransition(State* FromState,State* ToState, std::function<bool()>const& EvalFunc)
		{
			m_Transitions.emplace_back(std::make_unique<Transition>(FromState, ToState, EvalFunc));
		};
		std::vector<std::unique_ptr<Transition>> const& GetTransitions() const{return m_Transitions;};
		State* GetCurrentState() const
		{
			if (m_CurrentStateIndex < 0) return nullptr;
			return m_States[m_CurrentStateIndex].get();
		};
		void SetCurrentState(State const* state);
	private:
		std::vector<std::unique_ptr<State>> m_States{};
		std::vector<std::unique_ptr<Transition>> m_Transitions{};
		int m_CurrentStateIndex{-1};
	}; // contains FSM logic
}

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class GAMEAIPROG_API UFSMComponent : public UBrainComponent
{
	GENERATED_BODY()

public:
	// Sets default values for this component's properties
	UFSMComponent();

	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;
	
	virtual void StartLogic() override;
	virtual void StopLogic(const FString& Reason) override;
	
	virtual bool IsRunning() const override; 
	
	void AddState(std::unique_ptr<GameAI::FSM::State>&& NewState);
	void AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To, std::function<bool()> EvalFunc);
		
protected:
	// Called when the game starts
	virtual void BeginPlay() override;

private:
	std::unique_ptr<GameAI::FSM::FSM> FSMInstance{std::make_unique<GameAI::FSM::FSM>()};
	bool bIsRunning{false};
	FString LastStopReason{};
};
