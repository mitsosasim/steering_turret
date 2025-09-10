#-----------------------------------------------------------
#-----------------------------------------------------------
#!/bin/bash

#-----------------------------------------------------------
umask 0003
fstfile="/etc/fstab"
NASIP="192.168.1.17"
NASShare="Shared/STCBoard"
STCGroup="STCOpers"

#-----------------------------------------------------------
# check if the group STCOpers exist, otherwise create it 
#-----------------------------------------------------------
if [ $(getent group $STCGroup) ]; then
    echo ". group $STCGroup found"
else
    echo "- create the $STCGroup group"
    groupadd $STCGroup
fi

grpid=`cat /etc/group | grep ^$STCGroup | cut -d: -f3`
echo "- group $STCGroup has id = $grpid"

#-----------------------------------------------------------
# Setup a mount point for the NAS, allowing STCOpers to rw
#-----------------------------------------------------------
if [ ! -f "$fstfile.SAVE" ]; then
    echo "- create a copy of $fstfile ($fstfile.SAVE)"
    cp $fstfile $fstfile.SAVE
fi
if [ ! -d "/mnt" ]; then
    mkdir /mnt
    chown root:$STCGroup /mnt
fi
if [ ! -d "/mnt/NAS" ]; then
     mkdir /mnt/NAS
     chown root:$STCGroup /mnt/NAS
fi

#-----------------------------------------------------------
# create the cfg line for the fstab
#-----------------------------------------------------------
mntcmd="//$NASIP$NASShare /mnt/NAS cifs users,rw,auto,user=generic,password=Alma0909,gid=$grpid,forcegid,file_mode=0775,dir_mode=0775  0  2"

#-----------------------------------------------------------
# check fstab. If cfg line exist, update it, otherwise add
#-----------------------------------------------------------
N=`grep -n "$NASIP" $fstfile | cut -d: -f1`
if [[ $N -gt 0 ]]; then
     sed  "$N c\\$mntcmd" $fstfile > $fstfile.2
    echo "- updated mount point on /etc/fstab for /mnt/NAS"
else
    echo "- mount point for media has been set"
    echo $mntcmd > $fstfile.2
fi
mv $fstfile.2 $fstfile


