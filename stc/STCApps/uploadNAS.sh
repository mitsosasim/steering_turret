#-----------------------------------------------------------
# Upload the STC application to the pencard
#
#-----------------------------------------------------------
#!/bin/bash


#-----------------------------------------------------------
# Mount the pencard                                        #
#-----------------------------------------------------------
SourceFolder="/home/alma/STCApps"
DestFolder="/mnt/NAS/STCBoard/STCInstall"

#-----------------------------------------------------------
# Loop through files in the source directory
#-----------------------------------------------------------
echo "upload files:"
cp "$SourceFolder/STCControl/ARM_A53/STCControl" $DestFolder
cp "$SourceFolder/STCLogger/ARM_A53/STCLogger" $DestFolder
cp "$SourceFolder/STCGPSData/ARM_A53/STCGPSData" $DestFolder
cp "$SourceFolder/install.sh" $DestFolder/tools
cp "$SourceFolder/uninstall.sh" $DestFolder/tools
cp "$SourceFolder/NAStofstab.sh" $DestFolder/tools
cp "$SourceFolder/update.sh" $DestFolder/tools
cp "$SourceFolder/start.sh" $DestFolder
cp "$SourceFolder/stop.sh" $DestFolder
#cp "$SourceFolder/data/" $DestFolder/data
#cp "$SourceFolder/config/aaa.cfg" $DestFolder/config
cp "$SourceFolder/STCControl.service" $DestFolder/config

