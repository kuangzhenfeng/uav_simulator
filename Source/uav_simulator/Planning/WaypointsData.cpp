// Copyright Epic Games, Inc. All Rights Reserved.

#include "WaypointsData.h"

UWaypointsData::UWaypointsData()
{
}

void UWaypointsData::AddWaypoint(const FVector& Waypoint)
{
	Waypoints.Add(Waypoint);
}

void UWaypointsData::SetWaypoints(const TArray<FVector>& InWaypoints)
{
	Waypoints = InWaypoints;
}

const TArray<FVector>& UWaypointsData::GetWaypoints() const
{
	return Waypoints;
}

int32 UWaypointsData::GetNumWaypoints() const
{
	return Waypoints.Num();
}

FVector UWaypointsData::GetWaypoint(int32 Index) const
{
	if (Waypoints.IsValidIndex(Index))
	{
		return Waypoints[Index];
	}
	return FVector::ZeroVector;
}

bool UWaypointsData::IsValid() const
{
	return Waypoints.Num() >= 2;
}

void UWaypointsData::Clear()
{
	Waypoints.Empty();
}
