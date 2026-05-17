#include "UBlackBoardKeyType_Seek.h"

const UBlackBoardKeyType_Seek::FDataType UBlackBoardKeyType_Seek::InvalidValue = nullptr;
UBlackBoardKeyType_Seek::UBlackBoardKeyType_Seek(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	ValueSize = sizeof(Seek*);  // tells the blackboard how many bytes to allocate
}

Seek* UBlackBoardKeyType_Seek::GetValue(const UBlackBoardKeyType_Seek* KeyOb, const uint8* RawData)
{
	return *reinterpret_cast<Seek* const*>(RawData);
}

bool UBlackBoardKeyType_Seek::SetValue(UBlackBoardKeyType_Seek* KeyOb, uint8* RawData, Seek* Value)
{
	*reinterpret_cast<Seek**>(RawData) = Value;
	return true;
}

bool UBlackBoardKeyType_Seek::IsAllowedByFilter(UBlackboardKeyType* AllowedType) const
{
	return AllowedType->IsA<UBlackBoardKeyType_Seek>();
}