#-----------------------------------------------------------
# Install the STC application
# set SrcFolder to
# 	"/mnt/NAS/STCBoard/STCInstall"	install from NAS
# 	"/media/usb/STCInstall"		install from USB card
#-----------------------------------------------------------
#!/bin/bash
# //192.168.1.17/Shared/STCBoard /mnt/NAS cifs users,rw,auto,user=generic,password=Alma0909,gid=984,forcegid,file_mode=0775,dir_mode=0775  0  2

#-----------------------------------------------------------
umask 0003
#SrcFolder="/media/usb/STCInstall"
SrcFolder="/mnt/NAS/STCBoard/STCInstall"
DstFolder="/home/STCApps"
STCService="/etc/systemd/system/STCControl.service"
STCGroup="stcopers"
fstfile="/etc/fstab"

#-----------------------------------------------------------
# Ask for confirmation
#-----------------------------------------------------------
clear
read -p "Are you sure you want to install STC Control? [Y/n]" -n 1 -r && echo""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
fi

echo ""
echo "STC control install: start"
# ==========================================================
# Setup the Operational platform
# ==========================================================
#-----------------------------------------------------------
# Create a group for STC Operators
# Create users for STC control.
# - users will have their own /home/ but the operational 
#   /home/STCApps will be set as default home directory
#-----------------------------------------------------------
if [ $(getent group $STCGroup) ]; then
    echo ". group $STCGroup found"
else
    echo "- create the $STCGroup group"
    groupadd $STCGroup
fi

grpid=`cat /etc/group | grep ^$STCGroup | cut -d: -f3`
echo "- group $STCGroup has id = $grpid"

# alma/alma
if id "alma" >/dev/null 2>&1; then
    echo ". user alma found"
else
    pswd="alma"
    echo "- create the alma user..."
    useradd alma
    echo -e "$pswd\n$pswd" | passwd alma > /dev/null
    usermod -d $DstFolder alma
    usermod -G $STCGroup alma
    # /dev/tty access when ubuntu
    #usermod -a -G dialup alma
    # /dev/tty access when Manjaro
    usermod -a -G uucp alma
fi

# stcusers/stc000
if id "stcuser" >/dev/null 2>&1; then
    echo ". user stcuser found"
else
    pswd="stc000"
    echo "- create the stcuser user..."
    useradd stcuser
    echo -e "$pswd\n$pswd" | passwd stcuser > /dev/null
    usermod -d $DstFolder stcuser
    usermod -G $STCGroup stcuser
    # /dev/tty access when ubuntu
    #usermod -a -G dialup alma
    # /dev/tty access when Manjaro
    usermod -a -G uucp stcuser
fi

#-----------------------------------------------------------
# Create the STC Application directory tree into /home
# and allow permissions to group STCOpers
#-----------------------------------------------------------
if [ ! -d "$DstFolder" ]; then
    mkdir $DstFolder
    mkdir $DstFolder/log
    mkdir $DstFolder/data
    mkdir $DstFolder/config
    mkdir $DstFolder/tools
    echo "- $DstFolder tree has been created!"
else
    echo ". $DstFolder already exist"
    if [ ! -d "$DstFolder/log" ]; then
        mkdir $DstFolder/log
    fi
    if [ ! -d "$DstFolder/data" ]; then
        mkdir $DstFolder/data
    fi
    if [ ! -d "$DstFolder/config" ]; then
        mkdir $DstFolder/config
    fi
    if [ ! -d "$DstFolder/tools" ]; then
        mkdir $DstFolder/tools
    fi
fi
echo "hostnamectl" >> $DstFolder/.bashrc
cd $DstFolder
find $DstFolder/* -type d -exec chmod 774 {} \;

#-----------------------------------------------------------
# Setup a mount point for the USB memory card, allowing
#   STCOpers users to install updates
#-----------------------------------------------------------
if [ ! -f "$fstfile.SAVE" ]; then
    echo "- create a copy of $fstfile ($fstfile.SAVE)"
    cp $fstfile $fstfile.SAVE
fi
if [ ! -d "/media" ]; then
    mkdir /media
    mkdir /media/usb
    chown root:STCOpers /media/usb
    echo "- mount point for the USB memory card has been created!"
else
    if [ ! -d "/media/usb" ]; then
        mkdir /media/usb
	chown root:STCOpers /media/usb
        echo "- mount point for the USB memory card has been updated!"
    fi
fi

#-----------------------------------------------------------
mntcmd="/dev/sda1 /media/usb vfat users,umask=003,utf8,noauto,uid=$grpid, 0 0"
echo "- mount line for $fstfile = $mntcmd"

N=`grep -n "media/usb" $fstfile | cut -d: -f1`
if [[ $N -gt 0 ]]; then
     sed  "$N c\\$mntcmd" $fstfile > $fstfile.2
    echo "- updated mount point on /etc/fstab for /media/usb"
else
    echo "- mount point for media has been set"
    echo $mntcmd > $fstfile.2
fi
mv $fstfile.2 $fstfile

# =========================================================
# Copy the Application and data from the source
# =========================================================
echo "- upload files from $SrcFolder"
cp -r $SrcFolder/. $DstFolder
find $DstFolder/* -type f -exec chmod 774 {} \;
chown -R root:$STCGroup $DstFolder

#-----------------------------------------------------------
# Setup the startup as a service
#-----------------------------------------------------------
echo "- STC startup as a service.."
temp="$DstFolder/config/STCControl.service.ORIG"
echo "[Unit]" > $temp
echo "Description=STCControl (ALMA srl - Roma)" >> $temp
echo "After=network.target" >> $temp
echo "" >> $temp
echo "[Service]" >> $temp
echo "Type=simple" >> $temp
echo "ExecStart=/home/STCApps/start.sh prod" >> $temp
echo "WorkingDirectory=$DstFolder" >> $temp
echo "" >> $temp
echo "[Install]" >> $temp
echo "WantedBy=multi-user.target" >> $temp
cp $temp $STCService
chmod 744 $STCService
systemctl enable $STCService

#-----------------------------------------------------------
# Installation complete!
#-----------------------------------------------------------
echo "STC control install: complete!"
echo ""
echo "Now:"
echo "- Edit the /home/STCApps/config/STCDefault.cfg to set your specific parameters, if any;"
echo "- REMEMBER to change user passwords according the Company policy rules;"
echo "- reboot to start STC"
echo "- check STCControl execution, by:"
echo "  systemctl status $STCService" 


