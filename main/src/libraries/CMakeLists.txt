# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(wysiwydIDLClients)
add_subdirectory(wrdac)
if(WYSIWYD_USE_GTK2)
    add_subdirectory(cvz)
endif()
