#-----------------------------------------------------------
# Run the STC application
# arguments
# -reset	removes data, logs and cfg files. Start with
#               factory settings.
# -ipcrm	removes IPC resources. It is like rebooting
#               the computer.
# dev		run from development 
#-----------------------------------------------------------
#!/bin/bash


#-----------------------------------------------------------
# Check system resources                                   #
#-----------------------------------------------------------
function CheckResource()
{
missdev=0
# check tty for Dual Antenna GPS
TTYdevice=`dmesg | grep -i "legacy console" | awk '{ print $6 } '`
echo "The available UART is /dev/ $TTYdevice"

# check CAN adapter
CANdevice=`ls /sys/class/net | grep can0`
if [[ "$CANdevice" != "can0" ]]; then
    echo "CAN adapter not found!"
    missdev=$((missdev+1))
fi
return "$missdev"
}


#-----------------------------------------------------------
# Reset                                                    #
#-----------------------------------------------------------
function STCReset()
{
echo "Clean STC environment"
if test -d ./config; then
	rm -r ./config 
	rmdir ./config
fi
if test -d ./log; then
	rm -r ./log 
	rmdir ./log
fi
if test -d ./data; then
	rm -r ./data 
	rmdir ./data
fi
}

#-----------------------------------------------------------
# Reset                                                    #
#-----------------------------------------------------------
function CleanIPC()
{
echo "Clean STC ipcs"
ME=`whoami`

IPCS_S=`ipcs -s | egrep "0x[0-9a-f]+ [0-9]+" | grep $ME | cut -f2 -d" "`
IPCS_M=`ipcs -m | egrep "0x[0-9a-f]+ [0-9]+" | grep $ME | cut -f2 -d" "`
IPCS_Q=`ipcs -q | egrep "0x[0-9a-f]+ [0-9]+" | grep $ME | cut -f2 -d" "`

for id in $IPCS_M; do
  ipcrm -m $id;
done

for id in $IPCS_S; do
  ipcrm -s $id;
done

for id in $IPCS_Q; do
  ipcrm -q $id;
done

}


############################################################
# STC applications                                         #
############################################################
echo "Run STC applications"
#echo "$0:"

#Cancellare la riga. Questa operazione è in carico a stop.sh
ps -ef | grep "STC" | awk '{print $2}' | grep -v 'grep' | xargs kill

if [[ "$1" == "-reset" ]]; then
  STCReset
fi

if [[ "$1" == "-ipcrm" ]]; then
  CleanIPC
fi

CheckResource
res=$?
if [[ "$res" -gt 0 ]]; then
  echo "$0: Missing $res devices. Unpredictable behaviour could occur"
fi

#==========================================
clear
if (( EUID == 0 )); then
   echo "You should not run as root." 1>&2
   echo "login as a STC operators and restart." 1>&2
   exit 100
fi

echo "--------------------------------------------------------------------------"
echo "STC Application"
if [ "$1" == "dev" ]; then
    echo " (intel/debug) - Development target platform"
    #~/STCApps/STCMonitor/Debug/STCMonitor &
    ~/STCApps/STCLogger/Debug/STCLogger &
    ~/STCApps/STCGPSData/Debug/STCGPSData &
    ~/STCApps/STCControl/Debug/STCControl &
else
    echo " (ARM/release) - production platform"
    #~/STCMonitor &
    ~/STCLogger &
    ~/STCGPSData &
    ~/STCControl &
fi
echo "                                                          ALMA sistemi srl"
echo "                                                          Roma  - (R) 2025"
echo "--------------------------------------------------------------------------"

ps -a
echo "--------------------------------------------------------------------------"

