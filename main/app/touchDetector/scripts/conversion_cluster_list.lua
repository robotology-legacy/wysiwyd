--
-- Copyright (C) 2015 iCub Facility
-- Authors: Bertrand HIGY
-- CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
--
 
-- loading lua-yarp binding library
require("yarp")

PortMonitor.update = function(thing)
	if thing:asBottle() == nil then
        print("event_monitor.lua: got wrong data type (expected type Bottle)")
        return thing
    end
    
	bt = thing:asBottle()
	output = yarp.Bottle()
	for i = 0, bt:size() - 1 do
	   	if (bt:get(i):asInt() > 3) then
	   		output:addInt(i);
	   	end
	end
	
	th = yarp.Things()
    th:setPortWriter(output)
	return th
end
