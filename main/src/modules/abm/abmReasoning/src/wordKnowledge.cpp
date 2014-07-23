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

	for (vector<string>::iterator itCon = vContext.begin() ; itCon != vContext.end() ; itCon++)
	{
		(matObject2Word.addLabelY(*itCon));
		(matWord2Object.addLabelY(*itCon));
	}


	// if the word is known:
	int iterationWord=0;
	for (vector<pair<string, int>>::iterator itWor = matWord2Object.vLabelX.begin() ; itWor != matWord2Object.vLabelX.end() ; itWor++)
	{
		if (itWor->first == sWord)	iterationWord = itWor->second;
	}


	// prepare a list with the object known
	vector<pair<string, double>	>	vsdListObjectScore;
	for (vector<pair<string, int>>::iterator itCurrentObject = listWordObjects.begin() ; itCurrentObject != listWordObjects.end() ; itCurrentObject++)
	{
		pair<string,double>	psdTemp;
		psdTemp.first = itCurrentObject->first;
		psdTemp.second = 0;

		//int sumObject = itOb->iPresence
		// for each context:
		for (vector<pair<string, int>>::iterator itCurrentContext = listWordContexts.begin() ; itCurrentContext != listWordContexts.end() ; itCurrentContext++)
		{

			// for a given word

			scoreProp scorePropTemp;
			// use of the current object, with the current concext
			scorePropTemp.A = matWord2Object.get(sWord, itCurrentContext->first, itCurrentObject->first);

			// use of a different object with the current context
			scorePropTemp.B = itCurrentContext->second - scorePropTemp.A;

			// use of the current object in another context
			scorePropTemp.C = itCurrentObject->second - scorePropTemp.A;
			
			// use for the current word and a different context, of another word
			scorePropTemp.D = iterationWord - scorePropTemp.A - scorePropTemp.B - scorePropTemp.D;

			// context is present is our current situation
			bool contextPresent = false;
			for (vector<string>::iterator	itInputContext = vContext.begin() ; itInputContext != vContext.end() ; itInputContext++)
			{
				contextPresent = contextPresent || (*itInputContext == itCurrentContext->first);
			}

			// correlation between word object and context:
			psdTemp.second   +=  scorePropTemp.getScoreSum(contextPresent);
		}

		vsdListObjectScore.push_back(psdTemp);
	}


	// display results :
    double dScoreMax = -100000;
    string sResult;
    cout << endl << "Hesitate between Product : "  << endl;
    for (vector<pair<string, double > >::iterator it_SubSc = vsdListObjectScore.begin() ; it_SubSc != vsdListObjectScore.end() ; it_SubSc++)
    {
        double dScoreTemp = it_SubSc->second;
        cout << "\t" << it_SubSc->first << "\t score = " << dScoreTemp << endl;
        if (dScoreTemp > dScoreMax)
        {
            dScoreMax = dScoreTemp;
            sResult = it_SubSc->first;
        }
    }   
    
    if (dScoreMax <= 0)
    {
        cout << "Can't take a decision on which ag to select" << endl;
        sResult = "none";
    }
    cout << "Final choice : " << sResult << "\t score : " << dScoreMax << endl;

}

void wordKnowledge::getWordFromObject(string sObject, vector<string> vContext)
{
	if (matObject2Word.addLabelX(sObject) || matWord2Object.addLabelZ(sObject))
	{
		cout << "in wordKnowledge::getWordFromObject:: sObject unknown" << endl;
		//TODO get this case
		return;
	}

	for (vector<string>::iterator itCon = vContext.begin() ; itCon != vContext.end() ; itCon++)
	{
		(matObject2Word.addLabelY(*itCon));
		(matWord2Object.addLabelY(*itCon));
	}


	// if the object is known:
	int iterationObject=0;
	for (vector<pair<string, int>>::iterator itObj = matObject2Word.vLabelX.begin() ; itObj != matObject2Word.vLabelX.end() ; itObj++)
	{
		if (itObj->first == sObject)	iterationObject = itObj->second;
	}


	// prepare a list with the words known
	vector<pair<string, double>	>	vsdListWordScore;
	for (vector<pair<string, int>>::iterator itCurrentWord = listWordObjects.begin() ; itCurrentWord != listWordObjects.end() ; itCurrentWord++)
	{
		pair<string,double>	psdTemp;
		psdTemp.first = itCurrentWord->first;
		psdTemp.second = 0;

		//int sumObject = itOb->iPresence
		// for each context:
		for (vector<pair<string, int>>::iterator itCurrentContext = listWordContexts.begin() ; itCurrentContext != listWordContexts.end() ; itCurrentContext++)
		{

			// for a given word

			scoreProp scorePropTemp;
			// use of the current object, with the current concext
			scorePropTemp.A = matWord2Object.get(sObject, itCurrentContext->first, itCurrentWord->first);

			// use of a different object with the current context
			scorePropTemp.B = itCurrentContext->second - scorePropTemp.A;

			// use of the current object in another context
			scorePropTemp.C = itCurrentWord->second - scorePropTemp.A;
			
			// use for the current word and a different context, of another word
			scorePropTemp.D = iterationObject - scorePropTemp.A - scorePropTemp.B - scorePropTemp.D;

			// context is present is our current situation
			bool contextPresent = false;
			for (vector<string>::iterator	itInputContext = vContext.begin() ; itInputContext != vContext.end() ; itInputContext++)
			{
				contextPresent = contextPresent || (*itInputContext == itCurrentContext->first);
			}

			// correlation between word object and context:
			psdTemp.second   +=  scorePropTemp.getScoreSum(contextPresent);
		}

		vsdListWordScore.push_back(psdTemp);
	}


	// display results :
    double dScoreMax = -100000;
    string sResult;
    cout << endl << "Hesitate between Product : "  << endl;
    for (vector<pair<string, double > >::iterator it_SubSc = vsdListWordScore.begin() ; it_SubSc != vsdListWordScore.end() ; it_SubSc++)
    {
        double dScoreTemp = it_SubSc->second;
        cout << "\t" << it_SubSc->first << "\t score = " << dScoreTemp << endl;
        if (dScoreTemp > dScoreMax)
        {
            dScoreMax = dScoreTemp;
            sResult = it_SubSc->first;
        }
    }   
    
    if (dScoreMax <= 0)
    {
        cout << "Can't take a decision on which ag to select" << endl;
        sResult = "none";
    }
    cout << "Final choice : " << sResult << "\t score : " << dScoreMax << endl;

}




