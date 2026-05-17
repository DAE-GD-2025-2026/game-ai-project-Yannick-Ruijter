#include "UBlackBoardKeyType_PathFollow.h"

const UBlackBoardKeyType_PathFollow::FDataType UBlackBoardKeyType_PathFollow::InvalidValue = nullptr;
UBlackBoardKeyType_PathFollow::UBlackBoardKeyType_PathFollow(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	ValueSize = sizeof(Seek*);  // tells the blackboard how many bytes to allocate
}

PathFollow* UBlackBoardKeyType_PathFollow::GetValue(const UBlackBoardKeyType_PathFollow* KeyOb, const uint8* RawData)
{
	return *reinterpret_cast<PathFollow* const*>(RawData);
}

bool UBlackBoardKeyType_PathFollow::SetValue(UBlackBoardKeyType_PathFollow* KeyOb, uint8* RawData, PathFollow* Value)
{
	*reinterpret_cast<PathFollow**>(RawData) = Value;
	return true;
}

bool UBlackBoardKeyType_PathFollow::IsAllowedByFilter(UBlackboardKeyType* AllowedType) const
{
	return AllowedType->IsA<UBlackBoardKeyType_PathFollow>();
}