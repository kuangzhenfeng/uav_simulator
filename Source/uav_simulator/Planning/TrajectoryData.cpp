// Copyright Epic Games, Inc. All Rights Reserved.

#include "TrajectoryData.h"

UTrajectoryData::UTrajectoryData()
{
}

void UTrajectoryData::SetTrajectory(const FTrajectory& InTrajectory)
{
	Trajectory = InTrajectory;
}

const FTrajectory& UTrajectoryData::GetTrajectory() const
{
	return Trajectory;
}

bool UTrajectoryData::IsValid() const
{
	return Trajectory.bIsValid;
}

void UTrajectoryData::Clear()
{
	Trajectory.Clear();
}
