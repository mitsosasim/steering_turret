#-----------------------------------------------------------
# Uninstall the STC application
#
#-----------------------------------------------------------
#!/bin/bash

#-----------------------------------------------------------
DstFolder="/home/STCApps"
STCService="/etc/systemd/system/STCControl.service"
STCGroup="STCOpers"
fstfile="/etc/fstab"

#-----------------------------------------------------------
# Ask for confirmation
#-----------------------------------------------------------
clear
echo "The STC control application will be completely removed from this computer."
echo "users, groups and relevant data and settings have been removed too."
echo "This action is destructive and cannot be recovered !"
echo ""
read -p "Are you sure you want to uninstall STC Control? [Y/n]" -n 1 -r && echo""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
fi
echo ""
echo "STC control uninstall: start"
#-----------------------------------------------------------
# 1a. Check if application is active as service
#     stop the service
# 1.b Check if application is active as background
#     stop the apps
#-----------------------------------------------------------
RES="$(systemctl is-active STCControl.service)"
if [ "${RES}" = "active" ]; then
    systemctl stop STCControl.service
    echo "- Service stopped."
    systemctl disable STCControl
    echo "- Service disabled."
    rm $STCService
    echo "- Service removed."
else 
    echo ". Service not running."
fi

RES=`ps -ef | grep "STC" | grep -v "grep"`
if [ ! -z "${RES}" ]; then
    ps -ef | grep "STC" | grep -v "grep" | awk '{print $2}' | xargs kill
    echo "- STC application stopped"
else 
    echo ". Application not running."  
fi

#-----------------------------------------------------------
# Remove users (but not root) and home directories
# remove 'STCOpers' group
#-----------------------------------------------------------
if id "alma" >/dev/null 2>&1; then
    usermod -rG STCOpers alma
    usermod -rG uucp alma
    userdel alma
    if [ -d "/home/alma" ]; then
        rm -rf /home/alma
    fi
    echo "- user alma completely removed."
else 
    echo ". user alma not found."  
fi

if id "stcuser" >/dev/null 2>&1; then
    usermod -rG STCOpers stcuser
    usermod -rG uucp stcuser
    userdel stcuser
    if [ -d "/home/stcuser" ]; then
        rm -rf /home/stcuser
    fi
    echo "- user stcuser completely removed."
else 
    echo ". user stcuser not found."  
fi

if [ $(getent group admin) ]; then
     groupdel STCOpers
     echo "- group STCOpers removed."
else 
    echo ". group STCOpers not found."  
fi

#-----------------------------------------------------------
# Clean the STC tree
#-----------------------------------------------------------
if [ -d "$DstFolder" ]; then
    rm -R $DstFolder
    echo "- $DstFolder removed."
else 
    echo ". folder $DstFolder not found."  
fi

#-----------------------------------------------------------
# Do we keep the mount point for the USB memory card ?
#-----------------------------------------------------------
N=`grep -n "media/usb" $fstfile | cut -d: -f1`
if [[ $N -gt 0 ]]; then
    echo "- mount point for media removed from $fstfile"
    sed -i -e "$N d" $fstfile
else 
    echo ". media/usb not defined in $fstfile."  
fi

#-----------------------------------------------------------
echo "STC control uninstall complete!"
echo ""
echo "The STC control application has been removed from this computer."
echo "Please reboot."



