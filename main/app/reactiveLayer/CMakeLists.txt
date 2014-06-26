# Copyright: (C) 2011 EFAA Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname reactiveLayer)
icub_app(${appname})

file(GLOB conf    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.*)


icub_app_install(${appname} 
                 FILES ${conf}
                 DESTINATION conf/)

