#!/bin/bash
#clear

############################################################
# STC applications                                         #
############################################################
echo "--------------------------------------------------------------------------"
echo "Stop STC applications"
echo ""
echo "Check log files in:"
echo "  $PWD/logs"
echo "--------------------------------------------------------------------------"
#echo "$0:"

ps -ef | grep "STC" | grep -v "grep" | awk '{print $2}' | xargs kill
ps -a

