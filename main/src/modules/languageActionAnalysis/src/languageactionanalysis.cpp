#include "languageactionanalysis.h"

/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Anne-Laure MEALIER
 * email:   anne.laure.mealier@gmail.com
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * wysiwyd/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/*

 * your program would get event descriptons, and then let the human subject ask qeustions about the events.  The questions would be based on those in slide 9, and they would allow ythe program to setermine wether it should use the force or result vector, and what to focus on.


loop {
read next event from file
finished = 0
While ( finished != 1) {
      use synthesiser to say "any questions?"
      if subject says "yes" then {
                    say "go ahead"
                    use speech recognizer to recognize subjects spoken question
                    determine if the question is "What happened" "how did that happen" or "what did the X do"
                    based on the question type, construct the appropriate response type using result or foce vector
                    }
       else {  subject is finished asking questions about this event, so we move on to the next event
                   finished = 1
                   }
      }

*/
LanguageActionAnalysis::LanguageActionAnalysis(ResourceFinder &rf)
{
    iCurrentInstance = -1;

}


LanguageActionAnalysis::~LanguageActionAnalysis()
{
    close();
}

bool LanguageActionAnalysis::configure(ResourceFinder &rf){
    bool	bEveryThingisGood = true;
    bool	bOptionnalModule  = true;

    moduleName    = rf.check("name",
                Value("languageActionAnalysis"),
                                     "module name (string)").asString();
    sKeyWord      = rf.check("keyword", Value("grammar")).toString().c_str();

    nameGrammarNode	 = rf.getContextPath().c_str();
    nameGrammarNode	+= rf.check("nameGrammarNode",  Value("/nameGrammarNode.xml")).toString().c_str();

    fvector	= rf.getContextPath().c_str();
    fvector += rf.check("vectorFile",  Value("vector.txt")).toString().c_str();

    cout << fvector << "        " << endl;

    // Open handler port
    string sName = getName();
    handlerPortName = "/"+ sName + "/rpc";

    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        bEveryThingisGood = false;
    }


    // Open port2speech
    port2SpeechRecogName = "/"+ sName + "/toSpeechRecog";

    if (!Port2SpeechRecog.open(port2SpeechRecogName.c_str())) {
        cout << getName() << ": Unable to open port " << port2SpeechRecogName << endl;
        bEveryThingisGood = false;
    }


    // Open port2iSpeak
    port2iSpeakName = "/" + sName + "/toiSpeak";

    if (!Port2iSpeak.open(port2iSpeakName.c_str())) {
        cout << getName() << ": Unable to open port " << port2iSpeakName << endl;
        bOptionnalModule = false;
    }

    attach(handlerPort);                  // attach to port

    //------------------------//
    //		iCub Client
    //------------------------//

    string ttsSystem = SUBSYSTEM_SPEECH;
    iCub = new ICubClient(moduleName.c_str(),"reservoirHandler","client.ini",true);

    char rep = 'n';
    while (rep!='y'&&!iCub->connect())
    {
        cout<<"iCubClient : Some dependencies are not running..."<<endl;
        break; //to debug
        Time::delay(1.0);
    }

    // Connect iCub Client, and ports
    bOptionnalModule &= Network::connect(port2SpeechRecogName.c_str(), "/speechRecognizer/rpc");
    bOptionnalModule &= Network::connect("/mainLoop/speechGrammar/keyword:o", handlerPortName.c_str());
    bOptionnalModule &= Network::connect(port2iSpeakName, "/iSpeak");

    if (!bOptionnalModule)
    {
        cout << endl << "Some dependencies are notyarp connect " <<endl<<endl;
    }

    if (!bEveryThingisGood || !bOptionnalModule)
        cout << endl << "Some dependencies are not running (ICubClient or port(s) connections)"<<endl<<endl;
    else
        cout << endl << endl << "----------------------------------------------" << endl << endl << "language ready !" << endl << endl;

    file.open(fvector.c_str());

    return mainNode();
}


bool LanguageActionAnalysis::interruptModule(){
    handlerPort.interrupt();
    Port2SpeechRecog.interrupt();
    Port2iSpeak.interrupt();
    return true;
}

bool LanguageActionAnalysis::close() {
    handlerPort.close();
    Port2SpeechRecog.close();
    iCub->close();
    Port2iSpeak.close();
    delete iCub;

    return true;
}

bool LanguageActionAnalysis::respond(const Bottle& command, Bottle& reply) {
    string helpMessage =  string(getName().c_str()) +
            " commands are: \n" +
            "help \n" +
            "quit \n";

    reply.clear();

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    else if (command.get(0).asString()==sKeyWord.c_str()) {
        mainNode();
    }

    return true;
}

bool LanguageActionAnalysis::updateModule() {
    return true;
}

double LanguageActionAnalysis::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 0.1;
}

string LanguageActionAnalysis::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    if (!isGrammar)
    {
        cout << "Error in LanguageActionAnalysis::grammarToString. Couldn't open file : " << sPath << "." << endl;
        return "Error in LanguageActionAnalysis::grammarToString. Couldn't open file";
    }

    string sLine;
    while( getline(isGrammar, sLine) )
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    return sOutput;
}

list<int> LanguageActionAnalysis::nbCaracters(string ssequence)
{

    int pos = 0;
    list<int> lposElements;
    for(pos = 0; pos < ssequence.size(); ++pos)
    {
        if (ssequence[pos]==','){
            lposElements.push_back(pos);
            cout << pos << " " << endl;
        }

    }
    return lposElements;
}


bool LanguageActionAnalysis::mainNode()
{
    if(getline(file,svector))
    {
        cout << svector << endl;
        iCub->say("Any questions");
        iquestion = languageNode();
    }
    else{
        iCub->say("We have finish the interaction");
    }
    return 0;
}

bool LanguageActionAnalysis::grammarNode()
{

        cout << "svector   " << svector << endl;
        cout << "iquestion :  " << iquestion << endl;
        cout << endl;
        int id = svector.find(";");
        int idf = svector.size();
        string sforce = svector.substr(6, id -7); //  => sforce=   "push(Ag1,object)"
        string sresult = svector.substr(id +8,idf-1-(id +8)); // =>   sresult=   "move(object)"

        string sverb, sagent1, sagent2, sobject;

        //force<give(Ag1,Ag2,object)>;result<received(Ag2,object)>

        if(iquestion == 1)  // => Case "What happened"  => Result part !!
        {
            int i = sresult.find("(");
            int iend = sresult.size();
            sverb = sresult.substr(0,i); // => "move"
            cout << "sverb : " << sverb << endl;

            sresult = sresult.substr(i+1,iend-(i+1)-1); // =>  "object"
            cout << "sresult : " << sresult << endl;

            list<int> lposElements = nbCaracters(sresult);
            int nb = lposElements.size();


            if (nb == 0){            //  "object"
                sobject=sresult;
                sanswer = "The " + sobject + " " + sverb;
                iCub->say(sanswer);
            }

            else if(nb == 1)   //  "Ag2,object"
            {
                list<int>::iterator it;

                for(it=lposElements.begin(); it!=lposElements.end(); ++it)
                {
                    sagent1=sresult.substr(0,*it);
                    cout << "sagent1 : " << sagent1 << endl;
                    sobject=sresult.substr(*it+1,lposElements.size()-(*it+1));
                    cout << "sobject : " << sobject << endl;
                }
                sanswer = sagent1 + " " + sverb + " the " + sobject;
                iCub->say(sanswer);
            }
        }
        else if(iquestion == 2)  // "what did Anne do ?" => active form
        {
            cout << "in iquestion == 2" << endl;

            int i = sforce.find("(");
            int iend = sforce.size();
            sverb = sforce.substr(0,i); // => "result"

            sforce = sforce.substr(i+1,iend-(i+1)-1); // =>  "Ag2,object"

            list<int> lposElements = nbCaracters(sforce);
            int nb = lposElements.size();

            if (nb == 0){            //  "object"
                sobject=sforce;
            }

            else if(nb == 1)   //  "Ag2,object"
            {
                list<int>::iterator it;
                int cpt=0;

                int tab[lposElements.size()];
                for(it=lposElements.begin(); it!=lposElements.end(); ++it)
                {
                    tab[cpt]=*it;
                    cpt++;
                }
                cout << tab << endl;
                sagent1=sforce.substr(0,tab[0]);
                sobject=sforce.substr(tab[0]+1,sforce.size()-sforce.size()-tab[0]+1);
                sanswer = sagent1 + " " + sverb + "s the " + sobject;
                iCub->say(sanswer);
            }
            else if(nb == 2)   //  "Ag1,Ag2,object"
            {
                list<int>::iterator it;
                int cpt=0;
                int tab[lposElements.size()];
                for(it=lposElements.begin(); it!=lposElements.end(); ++it)
                {
                    tab[cpt]=*it;
                    cpt++;
                    cout << tab << endl;
                }
                sagent1=sforce.substr(0,tab[0]);
                sagent2=sforce.substr(tab[0]+1,tab[1]-(tab[0]+1));
                sobject=sforce.substr(tab[1]+1,sforce.size()-tab[1]+1);
                sanswer = sagent1 + " " + sverb + "s the " + sobject + " to " + sagent2;
                iCub->say(sanswer);
            }
        }

        else if(iquestion == 3)  // "how did that happen" => passive form
        {
            cout << "in iquestion == 3" << endl;

            int i = sforce.find("(");
            int iend = sforce.size();
            string sverb = sforce.substr(0,i); // => "result"

            sforce = sforce.substr(i+1,iend-(i+1)-1); // =>  "Ag2,object"

            list<int> lposElements = nbCaracters(sforce);
            int nb = lposElements.size();

            if (nb == 0){            //  "object"
                string object=sforce;
            }

            else if(nb == 1)   //  "Ag2,object"
            {
                list<int>::iterator it;
                int cpt=0;

                int tab[lposElements.size()];
                for(it=lposElements.begin(); it!=lposElements.end(); ++it)
                {
                    tab[cpt]=*it;
                    cpt++;
                }
                cout << tab << endl;
                sagent1=sforce.substr(0,tab[0]);
                sobject=sforce.substr(tab[0]+1,sforce.size()-sforce.size()-tab[0]+1);
                sanswer = "The " + sobject + " has been " + sverb + " by " + sagent1;
                cout << "sanswer : " << endl;
                iCub->say(sanswer);
            }
            else if(nb == 2)   //  "Ag2,object"
            {
                list<int>::iterator it;
                int cpt=0;
                int tab[lposElements.size()];
                for(it=lposElements.begin(); it!=lposElements.end(); ++it)
                {
                    tab[cpt]=*it;
                    cpt++;
                    cout << tab << endl;
                }
                sagent1=sforce.substr(0,tab[0]);
                sagent2=sforce.substr(tab[0]+1,tab[1]-(tab[0]+1));
                sobject=sforce.substr(tab[1]+1,sforce.size()-tab[1]+1);
                sanswer = "The " + sobject + " has been " + sverb + "ed by " + sagent1 + " to " +sagent2;
                cout << "sanswer : " << endl;
                iCub->say(sanswer);
            }
        }
    //iCub->say("");
    cout << "##############################################################" << endl;
    return mainNode();
}

int LanguageActionAnalysis::languageNode()
{
    //iquestion = 3;
    //return grammarNode();

    sCurrentNode = "nodeQuestion";
    sCurrentGrammarFile = nameGrammarNode;
    ostringstream osError;			// Error message
    osError << "Error in  LanguageActionAnalysis | "<< sCurrentNode << " :: ";
    cout << endl << "In " << sCurrentNode << endl << endl;

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
            bMessenger, //to be send TO speech recog
            bAnswer, //response from speech recog without transfer information, including raw sentence
            bSemantic, // semantic information of the content of the recognition
            bSendReasoning,	// send the information of recall to the abmReasoning
            bSpeak,	// bottle for tts
            bTemp;

    bMessenger.addString("recogBottle");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());


    while (!fGetaReply)
    {
        bSpeechRecognized.clear();
        Port2SpeechRecog.write(bMessenger,bSpeechRecognized);

        //e.g. : Reply from Speech Recog : 1 ("I want you to produce language" (INFORMATION (type produce))) ACK
        cout << "Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 3)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }

        if (bSpeechRecognized.get(0).toString() == "0")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }


        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
    }

    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

    if (bAnswer.get(0).asString() == "stop")
    {
        iCurrentInstance = -1;
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        //return bOutput;
    }

    bAnswer = *bSpeechRecognized.get(1).asList();
    cout << "bAnswer.get(0).asString() : " << bAnswer.get(0).asString() << endl;

    if (bAnswer.get(0).asString() == "what happened")
    {
        iquestion = 1;
        return grammarNode();
    }
    else if (bAnswer.get(0).asString() == "what did Anne do ?" || bAnswer.get(0).asString() == "what did Laure do ?" || bAnswer.get(0).asString() == "what did Zahra do ?")
    {
        iquestion = 2;
        return grammarNode();
    }
    else if (bAnswer.get(0).asString() == "how did that happen")
    {
        iquestion = 3;
        return grammarNode();
    }

    else if (bAnswer.get(0).asString() == "no")
    {
        iCub->say("go ahead");
    }
    return languageNode();
}
