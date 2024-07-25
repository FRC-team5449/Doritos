start /min ssh -C -J prototype robot "cd /usr/local/frc/bin; frcKillRobot.sh"
@scp -C -J prototype build\libs\Doritos.jar robot:Doritos.jar
ssh -C -J prototype robot "cd /usr/local/frc/bin; frcRunRobot.sh"
::ssh -C -J prototype robot  "./robotCommand"
pause