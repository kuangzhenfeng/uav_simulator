@echo off
call "%~dp0env.bat"
"%UE_ROOT%\Engine\Binaries\DotNET\UnrealBuildTool\UnrealBuildTool.exe" ^
  -Mode=GenerateClangDatabase ^
  uav_simulatorEditor Win64 Development ^
  -Project="%PROJECT_ROOT%\uav_simulator.uproject" ^
  -OutputDir="%PROJECT_ROOT%"
