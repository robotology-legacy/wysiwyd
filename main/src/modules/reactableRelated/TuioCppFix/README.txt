This directory contains fixed file for compiling the TUIO_CPP library on linux (the original source code is in http://www.tuio.org/?software).
Those fixes were fine for TUI_CPP version 1.4 (Done in VVV2014)

1) Download TUI_CPP from http://www.tuio.org/?software
2) Extract somewhere, setup the TUIO_DIR environment variable to point there
3) Replace the original files by the one provided here
4) cd $TUIO_DIR
5) make
6) In the ccmake of WYSIWYD the TUIO library should be available. Turn on the cmake variable WYSIWYD_USE_TUIO. reactable2opc will be enabled.


/////////////////
Details of the fix from Anne Laure Mealier

TUIO Installation

Add at the end of the OSC_SOURCES of the Makefile: ./oscpack/ip/IpEndpointName.cpp 
OSC_SOURCES = ./oscpack/osc/OscTypes.cpp ./oscpack/osc/OscOutboundPacketStream.cpp ./oscpack/osc/OscReceivedElements.cpp ./oscpack/osc/OscPrintReceivedElements.cpp ./oscpack/ip/posix/NetworkingUtils.cpp ./oscpack/ip/posix/UdpSocket.cpp ./oscpack/ip/IpEndpointName.cpp

Possible Errors : 

C++ error : Sleep was not declared in this scope
Sleep is a Windows function. For Unix, look into using nanosleep (POSIX) or usleep (BSD; deprecated). You have to write 

#include <unistd.h> in TuioServer.cpp 
(cd /mnt/data/Data/BuildLinux/Robot/tools/TUIO_CPP/TUIO)

 «pthread_create@@GLIBC_2.2.5» error

To build reactable2OPC, if you have «pthread_create@@GLIBC_2.2.5» error, you need to add -lpthread in the CMakeLists.txt

 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNINGS_STRING} -lpthread")