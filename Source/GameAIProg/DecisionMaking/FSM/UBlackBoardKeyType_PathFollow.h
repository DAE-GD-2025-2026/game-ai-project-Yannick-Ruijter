#pragma once

#include "BehaviorTree/Blackboard/BlackboardKeyType.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"
#include "UBlackBoardKeyType_PathFollow.generated.h"

UCLASS(EditInlineNew, meta=(DisplayName="PathFollow"))
class UBlackBoardKeyType_PathFollow : public UBlackboardKeyType
{
	GENERATED_BODY()

public:
	typedef PathFollow* FDataType;
	static const FDataType InvalidValue;

	UBlackBoardKeyType_PathFollow(const FObjectInitializer& ObjectInitializer);

	static PathFollow* GetValue(const UBlackBoardKeyType_PathFollow* KeyOb, const uint8* RawData);
	static bool  SetValue(UBlackBoardKeyType_PathFollow* KeyOb, uint8* RawData, PathFollow* Value);

	virtual bool IsAllowedByFilter(UBlackboardKeyType* AllowedType) const override;
};