#-----------------------------------------------------------
# Create the STC application environment.
# 
# 1. Create the directory tree
# 2. Create a group for STC Operators
# 3. Create 2 users for STC control operations
#-----------------------------------------------------------
#!/bin/bash

#-----------------------------------------------------------
# Application root folder is:
# /home/STCApps
# 
# a dedicated group for the use of ST Control Application
# stcopers
#
# 2 users having all necessary OS privileges
# stc1/stc1
# stc2/stc2

# - users will have their own /home/ but the operational 
#   /home/STCApps will be set as default home directory

#-----------------------------------------------------------
umask 0003
DstFolder="/home/STCApps"
STCGroup="stcoper"

#-----------------------------------------------------------
# Ask for confirmation
#-----------------------------------------------------------
clear
read -p "Are you sure you want to create STC Control tree? [Y/n]" -n 1 -r && echo""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
fi

#-----------------------------------------------------------
# Operator Group
#-----------------------------------------------------------
if [ $(getent group $STCGroup) ]; then
    echo ". group $STCGroup found"
else
    echo "- create the $STCGroup group"
    groupadd $STCGroup
fi

#-----------------------------------------------------------
# Create the STC Application directory tree into /home
# and allow permissions to group STCOpers
#-----------------------------------------------------------
if [ ! -d "$DstFolder" ]; then
    echo "$DstFolder does not exist: create new!"
    mkdir $DstFolder
    mkdir $DstFolder/log
    mkdir $DstFolder/data
    mkdir $DstFolder/config
    mkdir $DstFolder/tools
    echo "$DstFolder tree has been created!"
else
    echo ". $DstFolder already exist"
    if [ ! -d "$DstFolder/log" ]; then
        echo "$DstFolder/log does not exist: create new!"
        mkdir $DstFolder/log
    fi
    if [ ! -d "$DstFolder/data" ]; then
        echo "$DstFolder/data does not exist: create new!"
        mkdir $DstFolder/data
    fi
    if [ ! -d "$DstFolder/config" ]; then
        echo "$DstFolder/config does not exist: create new!"
        mkdir $DstFolder/config
    fi
    if [ ! -d "$DstFolder/tools" ]; then
        echo "$DstFolder/tools does not exist: create new!"
        mkdir $DstFolder/tools
    fi
fi

#-----------------------------------------------------------
# Set permissions
#-----------------------------------------------------------
chown root:$STCGroup $DstFolder
chmod 774 $DstFolder
