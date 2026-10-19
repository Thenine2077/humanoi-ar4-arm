@echo off
REM ===================================================================
REM  Two AR4 arms, SIMULATED (mock hardware, no boards) on the PC - teach poses
REM  and try dual-arm programs while the Pi is not available.
REM  Opens 3 WSL windows: 1) mock arms (ar_dual)  2) MoveIt + RViz  3) program player
REM  Poses/programs go to ar4_ws\src\ar_program\config\programs.yaml on the PC
REM  (a different file from the one on the Pi - copy it over when the Pi is back).
REM ===================================================================

set WSL_DISTRO=Ubuntu-24.04
set WS_PATH=/mnt/c/Users/User/Desktop/ar4/ar4_ws

echo [1/3] Mock arms (left + right) ...
start "AR4 dual - arms (mock)" wsl.exe -d %WSL_DISTRO% -- bash -ic "cd %WS_PATH% && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch ar_dual dual_hardware.launch.py left_hw:=mock right_hw:=mock; exec bash"
timeout /t 8 /nobreak >nul

echo [2/3] MoveIt + RViz (opens BLANK on purpose - fixes the WSLg mouse freeze) ...
start "AR4 dual - MoveIt + RViz" wsl.exe -d %WSL_DISTRO% -- bash -ic "cd %WS_PATH% && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch ar_dual dual_moveit.launch.py rviz_config:=none; exec bash"
timeout /t 3 /nobreak >nul

echo [3/3] Program player ...
start "AR4 dual - player" wsl.exe -d %WSL_DISTRO% -- bash -ic "cd %WS_PATH% && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch ar_program arm_program.launch.py speed:=0.22; exec bash"

echo.
echo Done. In RViz: File - Open Config - choose
echo   %WS_PATH%/install/ar_dual/share/ar_dual/config/dual_moveit.rviz
echo Then Planning Group: left_arm / right_arm / both_arms - drag the ball or Joints tab - Plan and Execute
echo.
echo Teach / programs: open another WSL window, then
echo   cd %WS_PATH% ^&^& source install/setup.bash
echo   ros2 run ar_program teach act1 --arm left
echo   ros2 run ar_program teach --program pick2 act1/act1 act2/- -/act3
echo   ros2 run ar_program teach --show pick2
echo   ros2 topic pub --once /arm/run std_msgs/msg/String "{data: pick2}"
echo To stop: close all three windows (or Ctrl+C in each first).
pause
