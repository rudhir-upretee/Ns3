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
ENV_LOG="NS_LOG=$MOD1:$MOD2:$MOD3"
export $ENV_LOG

## Catch ctrl-c interrupt
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
# Sample run commands :
#
#./waf --run "scratch/vanetAnalysis --PrintHelp
#./waf --run "scratch/vanetAnalysis --PrintAttributes=ns3::OnOffApplication" 
#./waf --run "scratch/vanetApp --nWifi=10 --simulatorStopTime=10" --vis
#./waf --run "main-random-walk" --vis
###############################################################################
./waf --run "scratch/vanetApp --nWifi=200 --traciPort=50000 --traciHost=localhost --simulatorStartTime=54000 --simulatorStopTime=54050" --vis 
#./waf --run "scratch/vanetApp --nWifi=200 --traciPort=50000 --traciHost=localhost --simulatorStopTime=50"  --vis
#./waf --run "scratch/vanetApp --nWifi=10 --simulatorStopTime=300"  
