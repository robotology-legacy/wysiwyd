#!/bin/bash

SOURCE=$PWD

EXPECTED_ARGS=1
DESTINATIONXML=generated-from-xml
 
#if [ $# -ne $EXPECTED_ARGS ];
#then
#   echo "Usage: `basename $0` {main/contrib}"
#else
    base=$PWD
    cd $WYSIWYD_ROOT/main
	
	# clean-up
    rm tmp-doc -rf
	rm $DESTINATIONXML -rf
   
    # generate doxy from xml
    mkdir $DESTINATIONXML
    list=`find . -iname *.xml | xargs`
    for i in $list
    do
       path=`dirname $i`
       filename=`basename $i`
       doxyfile=${filename%%.*}
       xsltproc --output $DESTINATIONXML/$doxyfile.dox $YARP_ROOT/scripts/yarp-module.xsl $i
    done 

    doxygen ./conf/Doxyfile.txt
    cd $base
#fi




