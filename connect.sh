
yarp connect /icub/camcalib/left/out /DisparityModule/left:i; yarp connect /icub/camcalib/right/out /DisparityModule/right:i

if [ $# -eq 1 ]
then     
    yarp connect /DisparityModule/disp:o /$1   
fi
