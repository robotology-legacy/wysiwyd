This directory contains fixed file for compiling the TUIO_CPP library on linux (the original source code is in http://www.tuio.org/?software).
Those fixes were fine for TUI_CPP version 1.4 (Done in VVV2014)

1) Download TUI_CPP from http://www.tuio.org/?software
2) Extract somewhere, setup the TUIO_DIR environment variable to point there
3) Replace the original files by the one provided here
4) cd $TUIO_DIR
5) make
6) In the ccmake of WYSIWYD the TUIO library should be available. Turn on the cmake variable WYSIWYD_USE_TUIO. reactable2opc will be enabled.