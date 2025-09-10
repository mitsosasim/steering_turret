#-----------------------------------------------------------
# Upload the STC application to the pencard
#
#-----------------------------------------------------------
#!/bin/bash


#-----------------------------------------------------------
# Mount the pencard                                        #
#-----------------------------------------------------------
SourceFolder="/home/alma/STCApps"

#SourceFolder="/media/usb/STCUpdate"
DestFolder="/media/alma/C40B-A71D/STCUpdate"

#if mountpoint -q /media/usb 
#then
#  mount /dev/sda1 /media/usb
#fi

#-----------------------------------------------------------
# Check an STC update is included into the pencard            #
#-----------------------------------------------------------
if [ ! -d "$DestFolder" ]; then
  echo "The USB device is not suitable for STC upload"
  exit
fi

#-----------------------------------------------------------
# Loop through files in the source directory
#-----------------------------------------------------------
echo "upload files:"
cp "$SourceFolder/STCControl/ARM_A53/STCControl" $DestFolder
cp "$SourceFolder/STCLogger/ARM_A53/STCLogger" $DestFolder
cp "$SourceFolder/STCGPSData/ARM_A53/STCGPSData" $DestFolder
#cp "$SourceFolder/STCMonitor/ARM_A53/STCGPSData" $DestFolder
cp  $SourceFolder/*.sh $DestFolder
