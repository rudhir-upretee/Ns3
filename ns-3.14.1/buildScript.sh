#!/bin/bash

##
## Created : <28 Apr 2012> Rudhir

###############################################################################
# Ns3 Environment variables to control logging
#
# Log levels : level_error, level_warn, level_debug, level_info, 
#              level_function, level_logic, level_all
#
# Prefix     : prefix_func, prefix_time, prefix_node
###############################################################################
MOD1="VanetScript=level_debug|prefix_func"
MOD2="VanetMonitorApp=level_debug|prefix_func|prefix_time"
MOD3="SumoMobilityHelper=level_debug|prefix_func|prefix_time"
ENV_LOG="NS_LOG=$MOD1:$MOD2"
export $ENV_LOG

###############################################################################
# Catch ctrl-c interrupt
###############################################################################
ctrl_c ()
{
	echo
	echo "*****************************************"
	echo "EXIT : User pressed ctrl-c"
	echo "*****************************************"
	echo
	exit $?
}
trap ctrl_c SIGINT

###############################################################################
# Command line parameters to change tx power. 
# The argument is in dBm. For the default LogDistancePropagationLossModel 
# 24 dBm roughly gives a range of 1000 meters.
###############################################################################
TXPOWEND="--ns3::YansWifiPhy::TxPowerEnd=24"
TXPOWSTART="--ns3::YansWifiPhy::TxPowerStart=24"

###############################################################################
# Sample run commands :
#
#./waf --run "scratch/vanetAnalysis --PrintHelp
#./waf --run "scratch/vanetAnalysis --PrintAttributes=ns3::OnOffApplication" 
#./waf --run "main-random-walk" --vis
###############################################################################

./waf --run "scratch/vanetApp $TXPOWEND $TXPOWSTART --nWifi=200 --traciPort=50000 --traciHost=localhost --simulatorStartTime=54000 --simulatorStopTime=54350" 
