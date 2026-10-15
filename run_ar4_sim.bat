@echo off
REM ===================================================================
REM  Run AR4 system (Gazebo Harmonic simulation + MoveIt2), automatically
REM  Opens 2 WSL windows: 1) Gazebo sim   2) MoveIt + RViz (opens blank --
REM  see note below, this is intentional and fixes the mouse-click freeze)
REM ===================================================================

set WSL_DISTRO=Ubuntu-24.04
set WS_PATH=/mnt/c/Users/User/Desktop/ar4/ar4_ws

echo [1/2] Starting Gazebo simulation ...
start "AR4 - Gazebo Sim" wsl.exe -d %WSL_DISTRO% -- bash -ic "cd %WS_PATH% && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch ar_gz_bringup ar_gz.launch.py; exec bash"

echo Waiting 15 seconds for Gazebo to load and spawn the arm ...
timeout /t 15 /nobreak >nul

echo [2/2] Starting MoveIt + RViz (opens blank on purpose) ...
start "AR4 - MoveIt + RViz" wsl.exe -d %WSL_DISTRO% -- bash -ic "cd %WS_PATH% && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch ar_gz_bringup ar_moveit_jazzy.launch.py use_sim_time:=true; exec bash"

echo.
echo Done. RViz opens BLANK on purpose (fixes a WSLg mouse-click freeze bug).
echo In the RViz window: File menu - Open Config - choose (this is a path
echo inside the RViz file dialog, which browses the Linux filesystem):
echo   %WS_PATH%/install/ar_gz_bringup/share/ar_gz_bringup/config/ar_moveit.rviz
echo Then Plan / Execute works as normal.
echo To stop: close both windows (or Ctrl+C in each window first).
pause
