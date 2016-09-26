#include "discourseform.h"
#include <set>
#include <map>

using namespace yarp::os;
using namespace discourseform;
using namespace std;

// get a discrouse from speech recog, to lrh, to be put under the good format
bool meaningDiscourse::meaningToDiscourseForm(vector<string> vMeaning){
    cout << "Meaning to discourse ... ";
    // get each sentence
    for (vector<string>::iterator level1 = vMeaning.begin(); level1 != vMeaning.end(); level1++){
        //level1 is:  OCW, OCW OCW OCW, OCW OCW, P1, P2 A2 O2, P3 A3 ...
        if (*level1 != ""){

            // get the meaning:
            //cout << "level1 is: " << *level1 << endl;
            string delimiter = ",";
            size_t pos = 0;
            string token;
            string s = *level1;
            ostringstream osMeaning;
            int iPAORWordsInProp = 0;
            bool isFirst = true;
            while ((pos = s.find(delimiter)) != string::npos) {
                token = s.substr(0, pos);
                if (!isFirst) { osMeaning << " , "; }
                osMeaning << token;
                iPAORWordsInProp++;
                s.erase(0, pos + delimiter.length());
                //cout << "\t meaning is: " << osMeaning.str() << " - iPAORWordsInProp " << iPAORWordsInProp << endl;
                isFirst = false;
            }
            //cout << s << endl;

            vector<string>  meaningParsed;
            // meaning parsed by spaces and colon
            stringstream stringStream(*level1);
            string line;
            while (getline(stringStream, line))
            {
                size_t prev = 0, pos;
                while ((pos = line.find_first_of(" ,", prev)) != string::npos)
                {
                    if (pos > prev)
                        meaningParsed.push_back(line.substr(prev, pos - prev));
                    prev = pos + 1;
                }
                if (prev < line.length())
                    meaningParsed.push_back(line.substr(prev, string::npos));
            }

            // remove PAOR from meaningParsed:
            vector<string> meaningWords = vector<string>(meaningParsed.begin(), meaningParsed.begin() + meaningParsed.size() / 2);
            vector<string> meaningPAOR = vector<string>(meaningParsed.begin() + meaningParsed.size() / 2, meaningParsed.end());

            //        cout << "    meaningWords:  | ";
            for (vector < string >::iterator itWo = meaningWords.begin();
                itWo != meaningWords.end();
                itWo++){
                //            cout << *itWo << " | ";
            }
            //       cout << endl;

            //       cout << "    meaningPAOR:  | ";
            for (vector < string >::iterator itWo = meaningPAOR.begin();
                itWo != meaningPAOR.end();
                itWo++){
                //         cout << *itWo << " | ";
            }
            //       cout << endl;
            //separation of the propositions

            meaningSentence currentSentence;

            if (meaningPAOR.size() != meaningWords.size()){
                yWarning() << " in narrativeGraph::discourseform.cpp::meaningToDiscourseForm: Size of PAOR and OCW different";
                for (auto pa : meaningPAOR){
                    cout << pa << " ";
                }
                cout << "\t";
                for (auto mea : meaningWords){
                    cout << mea << " ";
                }
                cout << endl << "PAOR: " << meaningPAOR.size() << "\t OCW: " << meaningWords.size() << endl;
                cout << "OCW are : ";
                for (auto oc : meaningWords){
                    cout << oc << "\t";
                }
                cout << endl;
            }
            else{
                for (int iWords = 0; iWords < meaningPAOR.size(); iWords++){
                    int iNumberProposition = atoi(&(meaningPAOR[iWords].at(1)));

                    // if new proposition
                    if (currentSentence.vSentence.size() < iNumberProposition){
                        meaningProposition tmp;

                        currentSentence.vSentence.push_back(tmp);
                    }

                    // add the OCW and PAOR
                    currentSentence.vSentence[iNumberProposition - 1].vOCW.push_back(meaningWords[iWords]);
                    currentSentence.vSentence[iNumberProposition - 1].vRole.push_back(&meaningPAOR[iWords].at(0));
                }

                if (currentSentence.vSentence.size() != 0){
                    meanings.vDiscourse.push_back(currentSentence);
                }
            }
        }
    }

    cout << "done." << endl;

    //    print();

    return true;
}


void meaningDiscourse::print(){

    cout << "------------------ BEGIN  MEANINGDISCOURSE --------------------" << endl;

    cout << " --------" << endl;;
    for (vector<meaningSentence>::iterator level1 = meanings.vDiscourse.begin();
        level1 != meanings.vDiscourse.end();
        level1++
        )
    {
        for (vector<meaningProposition>::iterator level2 = level1->vSentence.begin();
            level2 != level1->vSentence.end();
            level2++){

            if (level2 == level1->vSentence.begin()){
                cout << "\t ----" << endl;
            }
            // proposition level:
            cout << "\t";
            for (vector<string>::iterator OCW = level2->vOCW.begin();
                OCW != level2->vOCW.end();
                OCW++){
                cout << *OCW << "\t";
            }
            //            cout << " \t IGARF linked: " << level2->kmLinkEvt[0].toString() <<endl;

            cout << endl << "\t ----" << endl;
        }
        cout << " --------" << endl;
    }
    cout << "------------------ END  MEANINGDISCOURSE --------------------" << endl;
}