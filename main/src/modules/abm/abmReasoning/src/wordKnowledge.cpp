#include <wordKnowledge.h>

Bottle wordKnowledge::addInstance(string sObjectIdOPC, string sWord, vector<string> vContext)
{
    Bottle bOutput;

	for (unsigned int i = 0; i < vContext.size() ; i++)
	{
		//matObject2Word.incr(sObjectIdOPC,vContext[i], sWord);
	}

    return bOutput;
}


void wordKnowledge::getObjectFromWord(string sWord, vector<string> vContext)
{
	if (matObject2Word.addLabelX(sWord) || matWord2Object.addLabelZ(sWord))
	{
		cout << "in wordKnowledge::getObjectFromWord:: sWord unknown" << endl;
		//TODO get this case
		return;
	}

	bool bAllContextExist = true;
	for (vector<string>::iterator itCon = vContext.begin() ; itCon != vContext.end() ; itCon++)
	{
		bAllContextExist = bAllContextExist && !(matObject2Word.addLabelY(*itCon));
		bAllContextExist = bAllContextExist && !(matWord2Object.addLabelY(*itCon));
	}


	// if the word is known:


	// prepare a list with the object known
	vector<pair<string, double>	>	vsdListObjectScore;
	for (vector<wordObject>::iterator itOb = listWordObjects.begin() ; itOb != listWordObjects.end() ; itOb++)
	{
		pair<string,double>	psdTemp;
		psdTemp.first = itOb->sLabel;
		psdTemp.second = 0;

		vsdListObjectScore.push_back(psdTemp);
	}

	int iterationWord;
	for (vector<pair<string, int>>::iterator itWor = matWord2Object.vLabelX.begin() ; itWor != matWord2Object.vLabelX.end() ; itWor++)
	{
		if (itWor->first == sWord)	iterationWord = itWor->second;
	}

	// for each possible object check the vraissemblance
	for (vector<pair<string,double> >::iterator  itCurrentObject = vsdListObjectScore.begin() ; itCurrentObject != vsdListObjectScore.end() ; itCurrentObject++)
	{



	}





}