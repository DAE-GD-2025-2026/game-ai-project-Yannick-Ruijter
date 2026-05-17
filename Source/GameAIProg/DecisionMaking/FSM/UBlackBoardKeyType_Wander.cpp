#include "UBlackBoardKeyType_Wander.h"

const UBlackBoardKeyType_Wander::FDataType UBlackBoardKeyType_Wander::InvalidValue = nullptr;
UBlackBoardKeyType_Wander::UBlackBoardKeyType_Wander(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	ValueSize = sizeof(Wander*);  // tells the blackboard how many bytes to allocate
}

Wander* UBlackBoardKeyType_Wander::GetValue(const UBlackBoardKeyType_Wander* KeyOb, const uint8* RawData)
{
	return *reinterpret_cast<Wander* const*>(RawData);
}

bool UBlackBoardKeyType_Wander::SetValue(UBlackBoardKeyType_Wander* KeyOb, uint8* RawData, Wander* Value)
{
	*reinterpret_cast<Seek**>(RawData) = Value;
	return true;
}

bool UBlackBoardKeyType_Wander::IsAllowedByFilter(UBlackboardKeyType* AllowedType) const
{
	return AllowedType->IsA<UBlackBoardKeyType_Wander>();
}