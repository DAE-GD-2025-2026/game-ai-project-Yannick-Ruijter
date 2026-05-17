#pragma once

#include "BehaviorTree/Blackboard/BlackboardKeyType.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "UBlackBoardKeyType_Seek.generated.h"  // ← must be last include

UCLASS(EditInlineNew, meta=(DisplayName="Seek"))
class UBlackBoardKeyType_Seek : public UBlackboardKeyType
{
	GENERATED_BODY()  // ← required for UObject reflection

public:
	typedef Seek* FDataType;
	static const FDataType InvalidValue;

	UBlackBoardKeyType_Seek(const FObjectInitializer& ObjectInitializer);

	static Seek* GetValue(const UBlackBoardKeyType_Seek* KeyOb, const uint8* RawData);
	static bool  SetValue(UBlackBoardKeyType_Seek* KeyOb, uint8* RawData, Seek* Value);

	virtual bool IsAllowedByFilter(UBlackboardKeyType* AllowedType) const override;
};