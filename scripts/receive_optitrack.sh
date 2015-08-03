#! /usr/bin/env sh

# check if processes are killed otherwise force kill
control_c()
{
    # wait a bit before dead-check
    sleep 1s

    if ps -p $OPTITRACK_ROS_PID > /dev/null
    then
        echo "optitrack-ros is still running, now force killed"
        kill -9 $OPTITRACK_ROS_PID
    else
        echo "stopped optitrack-ros"
    fi

    if ps -p $GENOMIX_PID > /dev/null
    then
        echo "genomix is still running, now force killed"
        kill -9 $GENOMIX_PID
    else
        echo "stopped genomix"
    fi

    if ps -p $TCL_PID > /dev/null
    then
        echo "genomix is still running, now force killed"
        kill -9 $TCL_PID
    else
        echo "stopped eltclsh"
    fi

  exit $?
}

# trap keyboard interrupt (control-c), to send proper kill to signal to subprocesses
trap control_c INT

# clean up old log
if [ -f .log_optitrack-ros ]
then
    rm .log_optitrack-ros
fi
if [ -f .log_genomixd ]
then
    rm .log_genomixd
fi
if [ -f .log_tcl ]
then
    rm .log_tcl
fi

# run optitrack
OPTITRACK_ROS="optitrack-ros"
$OPTITRACK_ROS >> .log_optitrack-ros &
OPTITRACK_ROS_PID=$!
sleep 1s
echo "started optitrack-ros with PID=$OPTITRACK_ROS_PID"

# run genomix
GENOMIX="genomixd -v"
$GENOMIX >> .log_genomixd &
GENOMIX_PID=$!
sleep 1s
echo "started genomix with PID=$GENOMIX_PID"

sed -i "s/[0-9]\{1,3\}.[0-9]\{1,3\}.[0-9]\{1,3\}.[0-9]\{1,3\}/$2/g" $1
TCL="eltclsh $1"
$TCL >> .log_tcl &
TCL_PID=$!
sleep 1s
echo "started eltclsh with PID=$TCL_PID"

# wait for keyboard interrupt
wait
