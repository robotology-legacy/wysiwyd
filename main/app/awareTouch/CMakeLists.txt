# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Hector Barron
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname awareTouch)
icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.*)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)
icub_app_install(${appname} 
                 FILES ${scripts}
                 DESTINATION scripts/)
icub_app_install(${appname} 
                 FILES ${conf}
                 DESTINATION conf/)
