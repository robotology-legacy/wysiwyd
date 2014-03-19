#!/bin/bash

SOURCE=$PWD

EXPECTED_ARGS=1
 
#if [ $# -ne $EXPECTED_ARGS ];
#then
#   echo "Usage: `basename $0` {main/contrib}"
#else
    base=$PWD
    cd $WYSIWYD_ROOT/main
    rm tmp-doc -rf
    doxygen ./conf/Doxyfile.txt
    cd $base
#fi




