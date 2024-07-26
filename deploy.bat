@scp -C build\libs\Doritos.jar robot:Doritos.jar
ssh -C robot "cd /usr/local/frc/bin; ./frcKillRobot.sh"
::ssh -C -J prototype robot "cd /usr/local/frc/bin; frcRunRobot.sh"
::ssh -C -J prototype robot  "./robotCommand"
pause