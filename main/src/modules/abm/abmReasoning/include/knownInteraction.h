#include <abmReasoningFunction.h>

class knownInteraction
{   
public:
    std::string                                          sSubject;
    std::vector< tuple<std::string, int, std::string, std::string> >            listInteraction;        // an interaction : argument (name of the "object", number of time, type of "interaction", role of the interaction "spatial..."


    void addInteraction(tuple<std::string, int, std::string, std::string> tInput)
    {
        bool bFound = false;
        for (std::vector<tuple<std::string, int, std::string, std::string>>::iterator itTuple = listInteraction.begin() ; itTuple != listInteraction.end() ; itTuple++)
        {
            if (get<0>(tInput) == get<0>(*itTuple) && get<2>(tInput) == get<2>(*itTuple) && get<3>(tInput) == get<3>(*itTuple) && !bFound) 
            {
                get<1>(*itTuple)++;
                bFound =true;
            }
        }
        if (!bFound)
        {
            listInteraction.push_back(tInput);
        }
    }

};

