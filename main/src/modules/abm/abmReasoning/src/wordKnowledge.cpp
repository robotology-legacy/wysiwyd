#include <wordKnowledge.h>

Bottle wordKnowledge::addInstance(string sObject, string sWord, vector<string> vContext)
{
    Bottle bOutput;

    matObject2Word.incr(sObject,vContext, sWord);
    matWord2Object.incr(sWord,vContext, sObject);

    return bOutput;
}


pair<string, double> wordKnowledge::getObjectFromWord(string sWord, vector<string> vContext)
{
    if (matObject2Word.addLabelZ(sWord, true) || matWord2Object.addLabelX(sWord, true))
    {
        cout << "in wordKnowledge::getObjectFromWord:: sWord unknown" << endl;
        //TODO get this case
        return pair<string, double>("none", -1000000);
    }

    for (vector<string>::iterator itCon = vContext.begin() ; itCon != vContext.end() ; itCon++)
    {
        (matObject2Word.addLabelY(*itCon, true));
        (matWord2Object.addLabelY(*itCon, true));
    }


    // if the word is known:
    int iterationWord=0;
    for (vector<pair<string, int>>::iterator itWor = matWord2Object.vLabelX.begin() ; itWor != matWord2Object.vLabelX.end() ; itWor++)
    {
        if (itWor->first == sWord)  iterationWord = itWor->second;
    }


    // prepare a list with the object known
    vector<pair<string, double> >   vsdListObjectScore;
    for (vector<pair<string, int>>::iterator itCurrentObject = matWord2Object.vLabelZ.begin() ; itCurrentObject != matWord2Object.vLabelZ.end() ; itCurrentObject++)
    {
        pair<string,double> psdTemp;
        psdTemp.first = itCurrentObject->first;
        psdTemp.second = 0;

        //int sumObject = itOb->iPresence
        // for each context:
        for (vector<pair<string, int>>::iterator itCurrentContext = matWord2Object.vLabelY.begin() ; itCurrentContext != matWord2Object.vLabelY.end() ; itCurrentContext++)
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
            scorePropTemp.D = matWord2Object.iSum - iterationWord - scorePropTemp.A - scorePropTemp.B - scorePropTemp.D;

            // context is present is our current situation
            bool contextPresent = false;
            for (vector<string>::iterator   itInputContext = vContext.begin() ; itInputContext != vContext.end() ; itInputContext++)
            {
                contextPresent = contextPresent || (*itInputContext == itCurrentContext->first);
            }

            // correlation between word object and context:
            psdTemp.second   +=  scorePropTemp.getScoreSum(contextPresent);
            scorePropTemp.getScoreSum(contextPresent);
            cout <<"";
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

    /*if (dScoreMax <= 0)
    {
    cout << "Can't take a decision on which ag to select" << endl;
    sResult = "none";
    }*/
    cout << "Word input is : " << sWord << endl;
    cout << "Context is : ";
    for (unsigned int i = 0 ; i < vContext.size() ; i++)
    {
        cout << "\t" << vContext[i];
    }
    cout << endl << "Final choice : " << sResult << "\t score : " << dScoreMax << endl;

    return pair<string, double>(sResult,dScoreMax);
}

pair<string, double> wordKnowledge::getWordFromObject(string sObject, vector<string> vContext)
{
    if (matObject2Word.addLabelX(sObject, true) || matWord2Object.addLabelZ(sObject, true))
    {
        cout << "in wordKnowledge::getWordFromObject:: sObject unknown" << endl;
        //TODO get this case
        return pair<string, double>("none", -1000000);
    }

    for (vector<string>::iterator itCon = vContext.begin() ; itCon != vContext.end() ; itCon++)
    {
        (matObject2Word.addLabelY(*itCon, true));
        (matWord2Object.addLabelY(*itCon, true));
    }


    // if the object is known:
    int iterationObject=0;
    for (vector<pair<string, int>>::iterator itObj = matObject2Word.vLabelX.begin() ; itObj != matObject2Word.vLabelX.end() ; itObj++)
    {
        if (itObj->first == sObject)    iterationObject = itObj->second;
    }


    // prepare a list with the words known
    vector<pair<string, double> >   vsdListWordScore;
    for (vector<pair<string, int>>::iterator itCurrentWord = matObject2Word.vLabelZ.begin() ; itCurrentWord != matObject2Word.vLabelZ.end() ; itCurrentWord++)
    {
        pair<string,double> psdTemp;
        psdTemp.first = itCurrentWord->first;
        psdTemp.second = 0;

        //int sumObject = itOb->iPresence
        // for each context:
        for (vector<pair<string, int>>::iterator itCurrentContext = matObject2Word.vLabelY.begin() ; itCurrentContext != matObject2Word.vLabelY.end() ; itCurrentContext++)
        {

            // for a given word

            scoreProp scorePropTemp;
            // use of the current object, with the current concext
            scorePropTemp.A = matWord2Object.get(itCurrentWord->first, itCurrentContext->first, sObject);

            // use of a different object with the current context
            scorePropTemp.B = itCurrentContext->second - scorePropTemp.A;

            // use of the current object in another context
            scorePropTemp.C = itCurrentWord->second - scorePropTemp.A;

            // use for the current word and a different context, of another word
            scorePropTemp.D = matObject2Word.iSum - iterationObject - scorePropTemp.A - scorePropTemp.B - scorePropTemp.D;

            // context is present is our current situation
            bool contextPresent = false;
            for (vector<string>::iterator   itInputContext = vContext.begin() ; itInputContext != vContext.end() ; itInputContext++)
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

    /*if (dScoreMax <= 0)
    {
    cout << "Can't take a decision on which ag to select" << endl;
    sResult = "none";
    }*/
    cout << "Object input is : " << sObject << endl;
    cout << "Context is : ";
    for (unsigned int i = 0 ; i < vContext.size() ; i++)
    {
        cout << "\t" << vContext[i];
    }
    cout << endl << "Final choice : " << sResult << "\t score : " << dScoreMax << endl;

    return pair<string, double>(sResult,dScoreMax);
}

void wordKnowledge::simulateData()
{
    int     iNbRepetition = 2;

    bool simulateGlass = true;
    if (simulateGlass)
    {
        for (int i = 0 ; i < iNbRepetition ; i++)
        {       
            vector<string> context;
            context.push_back("context_glass");
            context.push_back("context_blue_bottle");
            context.push_back("context_cross");
            //      context.push_back("context_triangle");
            //      context.push_back("context_john");
            //      context.push_back("context_green_bottle");

            addInstance("object_glass","word_glass",context);   
        }




        for (int i = 0 ; i < iNbRepetition ; i++)
        {       
            vector<string> context;
            context.push_back("context_glass");
            context.push_back("context_blue_bottle");
            context.push_back("context_cross");
            context.push_back("context_triangle");
            //      context.push_back("context_john");
            //      context.push_back("context_green_bottle");

            addInstance("object_glass","word_glass",context);   
        }

        for (int i = 0 ; i < iNbRepetition ; i++)
        {       
            vector<string> context;
            context.push_back("context_glass");
            //context.push_back("context_blue_bottle");
            context.push_back("context_cross");
            context.push_back("context_triangle");
            context.push_back("context_john");
            //      context.push_back("context_green_bottle");

            addInstance("object_glass","word_glass",context);   
        }

        for (int i = 0 ; i < iNbRepetition ; i++)
        {       
            vector<string> context;
            context.push_back("context_glass");
            //context.push_back("context_blue_bottle");
            //      context.push_back("context_cross");
            context.push_back("context_triangle");
            context.push_back("context_john");
            context.push_back("context_green_bottle");

            addInstance("object_glass","word_glass",context);   
        }

        for (int i = 0 ; i < iNbRepetition ; i++)
        {       
            vector<string> context;
            context.push_back("context_glass");
            //context.push_back("context_blue_bottle");
            //      context.push_back("context_cross");
            //      context.push_back("context_triangle");
            context.push_back("context_john");
            context.push_back("context_green_bottle");

            addInstance("object_glass","word_glass",context);   
        }

        for (int i = 0 ; i < iNbRepetition ; i++)
        {       
            vector<string> context;
            context.push_back("context_glass");
            context.push_back("context_blue_bottle");
            //      context.push_back("context_cross");
            //      context.push_back("context_triangle");
            //      context.push_back("context_john");
            context.push_back("context_green_bottle");

            addInstance("object_glass","word_glass",context);   
        }

        for (int i = 0 ; i < iNbRepetition ; i++)
        {       
            vector<string> context;
            context.push_back("context_glass");
            context.push_back("context_blue_bottle");
            //      context.push_back("context_cross");
            context.push_back("context_triangle");
            //      context.push_back("context_john");
            context.push_back("context_green_bottle");

            addInstance("object_glass","word_glass",context);   
        }

        for (int i = 0 ; i < iNbRepetition ; i++)
        {       
            vector<string> context;
            context.push_back("context_glass");
            context.push_back("context_blue_bottle");
            //      context.push_back("context_cross");
            //      context.push_back("context_triangle");
            context.push_back("context_john");
            context.push_back("context_green_bottle");

            addInstance("object_glass","word_glass",context);   
        }

    }
    iNbRepetition = 3;

    //  blue_bottle without green bottle


    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_blue_bottle");
        //      context.push_back("context_cross");
        //      context.push_back("context_triangle");
        context.push_back("context_john");
        context.push_back("context_glass");
        //      context.push_back("context_green_bottle");

        addInstance("object_blue_bottle","word_bottle",context);    
        addInstance("object_blue_bottle","word_blue_bottle",context);   
    }


    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_blue_bottle");
        context.push_back("context_cross");
        //      context.push_back("context_triangle");
        //      context.push_back("context_john");
        context.push_back("context_glass");
        //      context.push_back("context_green_bottle");

        addInstance("object_blue_bottle","word_bottle",context);    
        addInstance("object_blue_bottle","word_blue_bottle",context);   
    }


    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_blue_bottle");
        context.push_back("context_cross");
        context.push_back("context_triangle");
        //      context.push_back("context_john");
        //      context.push_back("context_glass");
        //      context.push_back("context_green_bottle");

        addInstance("object_blue_bottle","word_bottle",context);    
        addInstance("object_blue_bottle","word_blue_bottle",context);   
    }


    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_blue_bottle");
        //      context.push_back("context_cross");
        context.push_back("context_triangle");
        context.push_back("context_john");
        //      context.push_back("context_glass");
        //      context.push_back("context_green_bottle");

        addInstance("object_blue_bottle","word_bottle",context);    
        addInstance("object_blue_bottle","word_blue_bottle",context);   
    }


    // ADD GREEN BOTTLE

    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_blue_bottle");
        //      context.push_back("context_cross");
        //      context.push_back("context_triangle");
        context.push_back("context_john");
        context.push_back("context_glass");
        context.push_back("context_green_bottle");

        addInstance("object_blue_bottle","word_blue_bottle",context);   

    }


    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_blue_bottle");
        context.push_back("context_cross");
        //      context.push_back("context_triangle");
        //      context.push_back("context_john");
        context.push_back("context_glass");
        context.push_back("context_green_bottle");

        addInstance("object_blue_bottle","word_blue_bottle",context);   
    }


    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_blue_bottle");
        context.push_back("context_cross");
        context.push_back("context_triangle");
        //      context.push_back("context_john");
        //      context.push_back("context_glass");
        context.push_back("context_green_bottle");

        addInstance("object_blue_bottle","word_blue_bottle",context);   
    }


    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_blue_bottle");
        //      context.push_back("context_cross");
        context.push_back("context_triangle");
        context.push_back("context_john");
        //      context.push_back("context_glass");
        context.push_back("context_green_bottle");

        addInstance("object_blue_bottle","word_blue_bottle",context);   
    }


    //cout << "\t\t 4 : " << matWord2Object.get("word_bottle","context_blue_bottle","object_green_bottle") << endl;


    //-------------------------------- IDEM WITH GREEN BOTTLE OBJECT


    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_green_bottle");
        //      context.push_back("context_cross");
        //      context.push_back("context_triangle");
        context.push_back("context_john");
        context.push_back("context_glass");
        //      context.push_back("context_blue_bottle");

        addInstance("object_green_bottle","word_bottle",context);   
        addInstance("object_green_bottle","word_green_bottle",context); 
    }


    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_green_bottle");
        context.push_back("context_cross");
        //      context.push_back("context_triangle");
        //      context.push_back("context_john");
        context.push_back("context_glass");
        //      context.push_back("context_blue_bottle");

        addInstance("object_green_bottle","word_bottle",context);   
        addInstance("object_green_bottle","word_green_bottle",context); 
    }


    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_green_bottle");
        context.push_back("context_cross");
        context.push_back("context_triangle");
        //      context.push_back("context_john");
        //      context.push_back("context_glass");
        //      context.push_back("context_blue_bottle");

        addInstance("object_green_bottle","word_bottle",context);   
        addInstance("object_green_bottle","word_green_bottle",context); 

    }


    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_green_bottle");
        //      context.push_back("context_cross");
        context.push_back("context_triangle");
        context.push_back("context_john");
        //      context.push_back("context_glass");
        //      context.push_back("context_blue_bottle");

        addInstance("object_green_bottle","word_bottle",context);   
        addInstance("object_green_bottle","word_green_bottle",context); 

    }


    // ADD BLUE BOTTLE

    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_green_bottle");
        //      context.push_back("context_cross");
        //      context.push_back("context_triangle");
        context.push_back("context_john");
        context.push_back("context_glass");
        context.push_back("context_blue_bottle");

        addInstance("object_green_bottle","word_green_bottle",context); 
    }

    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_green_bottle");
        context.push_back("context_cross");
        //      context.push_back("context_triangle");
        //      context.push_back("context_john");
        context.push_back("context_glass");
        context.push_back("context_blue_bottle");

        addInstance("object_green_bottle","word_green_bottle",context); 
    }

    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_green_bottle");
        context.push_back("context_cross");
        context.push_back("context_triangle");
        //      context.push_back("context_john");
        //      context.push_back("context_glass");
        context.push_back("context_blue_bottle");

        addInstance("object_green_bottle","word_green_bottle",context); 
    }

    for (int i = 0 ; i < iNbRepetition ; i++)
    {       
        vector<string> context;
        context.push_back("context_green_bottle");
        //      context.push_back("context_cross");
        context.push_back("context_triangle");
        context.push_back("context_john");
        //      context.push_back("context_glass");
        context.push_back("context_blue_bottle");

        addInstance("object_green_bottle","word_green_bottle",context); 
    }

    vector<string> context;



    context.clear();
    context.push_back("context_green_bottle");
    //      context.push_back("context_cross");
    context.push_back("context_triangle");
    context.push_back("context_john");
    //      context.push_back("context_glass");
    context.push_back("context_blue_bottle");

    getObjectFromWord("word_bottle",context);
    getWordFromObject("object_green_bottle",context);



    context.clear();
    context.push_back("context_green_bottle");
    //      context.push_back("context_cross");
    context.push_back("context_triangle");
    context.push_back("context_john");
    //      context.push_back("context_glass");
    //context.push_back("context_blue_bottle");

    cout << "\t\t" << matWord2Object.get("word_bottle","context_green_bottle","object_green_bottle") << endl;

    getObjectFromWord("word_bottle",context);
    getWordFromObject("object_green_bottle",context);





    context.clear();
    context.push_back("context_green_bottle");
    //      context.push_back("context_cross");
    context.push_back("context_triangle");
    context.push_back("context_john");
    context.push_back("context_glass");
    context.push_back("context_blue_bottle");

    getObjectFromWord("word_glass",context);
    getWordFromObject("object_glass",context);


}




