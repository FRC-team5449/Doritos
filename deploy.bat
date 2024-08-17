@start /min ssh -C robot "/etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t 2> /dev/null;echo '/usr/local/frc/JRE/bin/java -XX:+UseSerialGC -Djava.lang.invoke.stringConcat=BC_SB -Djava.library.path=/usr/local/frc/third-party/lib -jar /home/lvuser/Doritos.jar ' > /home/lvuser/robotCommand;chmod +x /home/lvuser/robotCommand; chown lvuser /home/lvuser/robotCommand"
@scp -C build\libs\Doritos.jar robot:Doritos.jar
ssh -C robot "chmod +x /home/lvuser/Doritos.jar; chown lvuser /home/lvuser/Doritos.jar; sync; /usr/local/natinst/bin/nirtcfg --file=/etc/natinst/share/ni-rt.ini --get section=systemsettings,token=NoApp.enabled,value=unknown; /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r 2> /dev/null"
::ssh -C -J prototype robot "cd /usr/local/frc/bin; frcRunRobot.sh"
::ssh -C -J prototype robot  "./robotCommand"
pause