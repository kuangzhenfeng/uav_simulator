@echo off
REM === Shared environment config ===
REM Modify UE_ROOT to match your UE installation
set UE_ROOT=D:\mySoftware\Epic Games\UE_5.7
for %%I in ("%~dp0..") do set PROJECT_ROOT=%%~fI
