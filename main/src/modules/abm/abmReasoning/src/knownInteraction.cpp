#include <knownInteraction.h>


void knownInteraction::addInteraction(tuple<string, int, string, string> tInput)
{
    bool bFound = false;
    for (vector<tuple<string, int, string, string>>::iterator itTuple = listInteraction.begin() ; itTuple != listInteraction.end() ; itTuple++)
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