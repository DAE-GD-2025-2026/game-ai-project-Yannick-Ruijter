#pragma once

#include "BehaviorTree/Blackboard/BlackboardKeyType.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "UBlackBoardKeyType_Wander.generated.h"

UCLASS(EditInlineNew, meta=(DisplayName="Wander"))
class UBlackBoardKeyType_Wander : public UBlackboardKeyType
{
	GENERATED_BODY()

public:
	typedef Wander* FDataType;
	static const FDataType InvalidValue;

	UBlackBoardKeyType_Wander(const FObjectInitializer& ObjectInitializer);

	static Wander* GetValue(const UBlackBoardKeyType_Wander* KeyOb, const uint8* RawData);
	static bool  SetValue(UBlackBoardKeyType_Wander* KeyOb, uint8* RawData, Wander* Value);

	virtual bool IsAllowedByFilter(UBlackboardKeyType* AllowedType) const override;
};