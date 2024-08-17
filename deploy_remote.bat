@REM start /min ssh -C -J o7un79x2g2rvqj9cbtssrnruid9jsarckkxvrybabneoxcb3k4hisj6oco4jg31gw@ssh-j.com,prototype robot  "cd /usr/local/frc/bin; frcKillRobot.sh"
@echo . > DeployFlag
start /min scp -C build\libs\Doritos.jar DeployFlag prototype:~
@ssh -C prototype "./deploy.sh"
@REM ssh -C -J o7un79x2g2rvqj9cbtssrnruid9jsarckkxvrybabneoxcb3k4hisj6oco4jg31gw@ssh-j.com,prototype robot  "cd /usr/local/frc/bin; frcRunRobot.sh"
pause