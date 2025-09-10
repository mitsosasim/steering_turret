#-----------------------------------------------------------
# Update the STC application
#
#-----------------------------------------------------------
#!/bin/bash

#-----------------------------------------------------------
DestFolder="/home/STCApps"
SourceFolder="/media/usb/STCUpdate"

#-----------------------------------------------------------
# Mount the pencard                                        #
#-----------------------------------------------------------
if [ ! mountpoint -q /media/usb ]; then
  mount /dev/sda1
fi

#-----------------------------------------------------------
# Check an STC update is included into the pencard            #
#-----------------------------------------------------------
if [ ! -d "$SourceFolder" ]; then
  echo "The USB device is not suitable for STC update. ABORTED!"
  exit
fi

#-----------------------------------------------------------
# Stop current application
#-----------------------------------------------------------
if [ systemctl is-active --quiet STCControl ]; then
   systemctl stop STCControl.service
else
   ps -ef | grep "STC" | awk '{print $2}' | grep -v 'grep' | xargs kill
fi

#-----------------------------------------------------------
# Make a new folder into the ./update
# copy current version into ./oldver
# copy the new version into ./newver
# copy the new version into /home/STCApps 
#-----------------------------------------------------------
StoreFolder="$DestFolder/updates"
[ ! -d "$StoreFolder" ] && mkdir -p "$StoreFolder"
UpdDate="$(date +"%Y%m%d-%H%M")"
mkdir "$StoreFolder/$UpdDate"
mkdir "$StoreFolder/$UpdDate/oldver"
mkdir "$StoreFolder/$UpdDate/newver"
echo  "$StoreFolder/$UpdDate backup folder was created!"

#-----------------------------------------------------------
# Loop through files in the source directory
#-----------------------------------------------------------
echo "upload files:"
for file in "$SourceFolder"/*; do
  if [ -f "$file" ]; then
    echo $file
    updfile=$(basename -- "$file")
    cp "$DestFolder/$updfile" "$StoreFolder/$UpdDate/oldver"
    cp "$file" "$StoreFolder/$UpdDate/newver"
    cp "$StoreFolder/$UpdDate/newver/$updfile" "$DestFolder/$updfile"
  fi
done

