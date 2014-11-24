#ifndef _KNOWNINTERACTION_H_
#define _KNOWNINTERACTION_H_

#include <abmReasoningFunction.h>

class knownInteraction
{   
public:
    std::string                                          sSubject;
    std::vector< std::tuple<std::string, int, std::string, std::string> >            listInteraction;        // an interaction : argument (name of the "object", number of time, type of "interaction", role of the interaction "spatial..."


    void addInteraction(std::tuple<std::string, int, std::string, std::string> tInput)
    {
        bool bFound = false;
        for (std::vector<std::tuple<std::string, int, std::string, std::string>>::iterator itTuple = listInteraction.begin() ; itTuple != listInteraction.end() ; itTuple++)
        {
            if (std::get<0>(tInput) == std::get<0>(*itTuple) && std::get<2>(tInput) == std::get<2>(*itTuple) && std::get<3>(tInput) == std::get<3>(*itTuple) && !bFound) 
            {
                std::get<1>(*itTuple)++;
                bFound =true;
            }
        }
        if (!bFound)
        {
            listInteraction.push_back(tInput);
        }
    }

};

#endif