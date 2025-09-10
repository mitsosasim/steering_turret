#-----------------------------------------------------------
#-----------------------------------------------------------
#!/bin/bash

#-----------------------------------------------------------
# check if the group STCOpers exist, otherwise create it 
#-----------------------------------------------------------
mytty=`dmesg | grep -i "legacy console" | awk '{ print $1 }'
echo "Current RS TTY is on $mytty"


