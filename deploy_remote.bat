start /min ssh -C -J o7un79x2g2rvqj9cbtssrnruid9jsarckkxvrybabneoxcb3k4hisj6oco4jg31gw@ssh-j.com,prototype robot  "cd /usr/local/frc/bin; frcKillRobot.sh"
scp -C -J o7un79x2g2rvqj9cbtssrnruid9jsarckkxvrybabneoxcb3k4hisj6oco4jg31gw@ssh-j.com,prototype build\libs\Doritos.jar robot:Doritos.jar
ssh -C -J o7un79x2g2rvqj9cbtssrnruid9jsarckkxvrybabneoxcb3k4hisj6oco4jg31gw@ssh-j.com,prototype robot  "cd /usr/local/frc/bin; frcRunRobot.sh"
pause