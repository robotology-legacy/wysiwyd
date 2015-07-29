--
-- Copyright (C) 2015 iCub Facility
-- Authors: Bertrand HIGY
-- CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
--
 
-- loading lua-yarp binding library
require("yarp")

PortMonitor.accept = function(thing)
	if thing:asBottle() == nil then
        print("event_monitor.lua: got wrong data type (expected type Bottle)")
        return thing
    end
    
	bt = thing:asBottle()
	PortMonitor.output = yarp.Bottle()
	for i = 0, bt:size() - 1 do
	   	if (bt:get(i):asInt() > 3) then
	   		PortMonitor.output:addInt(i);
	   	end
	end
	
	return PortMonitor.output:size() > 0;
end

PortMonitor.update = function(thing)
	th = yarp.Things()
    th:setPortWriter(PortMonitor.output)
	return th
end
