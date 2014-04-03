#include "relation.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include "functions.h"

namespace efaa { namespace helpers { namespace knowledge { namespace language {

//A few enums and associated converting to string material
enum Goal { Ask, Describe }; const char *GoalStr[]={ "Ask","Describe"};
enum Tense { Present, Past, future }; const char *TenseStr[]={ "Present","Past","Future"};
enum GForm { Active, Passive }; const char *GFormStr[]={ "Active","Passive"};
enum TenseModifier { Ponctual, Continuous }; const char *TenseModifierStr[]={ "Ponctual","Continuous"};
std::string enum2str(Goal g) {return GoalStr[g];}
std::string enum2str(Tense g) {return TenseStr[g];}
std::string enum2str(GForm g) {return GFormStr[g];}
std::string enum2str(TenseModifier g) {return TenseModifierStr[g];}

struct VerbInfo
{
    std::string verbalRoot;
    std::vector<std::string> presentForms;
    std::vector<std::string> preteritForms;

    std::string participe;
    std::string ingForm;
    std::string toString()
    {
        std::ostringstream oss;
        oss<<verbalRoot<<std::endl
            <<'\t'<<"Present:"<<'\t'<<presentForms[0]<<'\t'<<presentForms[1]<<'\t'<<presentForms[2]<<'\t'<<presentForms[3]<<'\t'<<presentForms[4]<<std::endl
            <<'\t'<<"Preterit:"<<'\t'<<preteritForms[0]<<'\t'<<preteritForms[1]<<'\t'<<preteritForms[2]<<'\t'<<preteritForms[3]<<'\t'<<preteritForms[4]<<std::endl
            <<'\t'<<"Participe:"<<'\t'<<participe<<std::endl
            <<'\t'<<"Continuous:"<<'\t'<<ingForm<<std::endl;

        return oss.str();
    }

    yarp::os::Bottle asBottle()
    {
        yarp::os::Bottle b;

         yarp::os::Bottle sub;
         sub.addString("root");
         sub.addString(verbalRoot.c_str());
         b.addList() = sub;
         sub.clear();

         sub.addString("preterit");
         yarp::os::Bottle &formsP = sub.addList();
         formsP.addString(preteritForms[0].c_str());
         formsP.addString(preteritForms[1].c_str());
         formsP.addString(preteritForms[2].c_str());
         formsP.addString(preteritForms[3].c_str());
         formsP.addString(preteritForms[4].c_str());
         b.addList() = sub;
         sub.clear();
                  
         sub.addString("participe");
         sub.addString(participe.c_str());
         b.addList() = sub;
         sub.clear();

         sub.addString("ingForm");
         sub.addString(ingForm.c_str());
         b.addList() = sub;
         sub.clear();

         sub.addString("presentForms");
         yarp::os::Bottle &forms = sub.addList();
         forms.addString(presentForms[0].c_str());
         forms.addString(presentForms[1].c_str());
         forms.addString(presentForms[2].c_str());
         forms.addString(presentForms[3].c_str());
         forms.addString(presentForms[4].c_str());
         b.addList() = sub;
         sub.clear();
        return b;
    }

    void fromBottle(yarp::os::Bottle b)
    {
         verbalRoot = b.find("root").asString().c_str();
         participe = b.find("participe").asString().c_str();
         ingForm = b.find("ingForm").asString().c_str();

         yarp::os::Bottle *forms =  b.find("preteritForms").asList();
         preteritForms.resize(5);
         preteritForms[0] = forms->get(0).asString().c_str();
         preteritForms[1] = forms->get(1).asString().c_str();
         preteritForms[2] = forms->get(2).asString().c_str();
         preteritForms[3] = forms->get(3).asString().c_str();
         preteritForms[4] = forms->get(4).asString().c_str();

         forms =  b.find("presentForms").asList();
         presentForms.resize(5);
         presentForms[0] = forms->get(0).asString().c_str();
         presentForms[1] = forms->get(1).asString().c_str();
         presentForms[2] = forms->get(2).asString().c_str();
         presentForms[3] = forms->get(3).asString().c_str();
         presentForms[4] = forms->get(4).asString().c_str();
    }
};

struct VerbStatus
{
    std::string verbalRoot;
    Tense tense;
    TenseModifier mod;
    int person;
    std::string getGrammarExpression()
    {
        std::ostringstream oss;
        oss<<"#V#"<<verbalRoot<<"("<<tense<<","<<mod<<","<<person<<")";
        return oss.str();
    }
};

struct Conjugator
{
    std::map<std::string, VerbInfo> knownVerbs;
    std::map<std::string, VerbInfo> irregulars;

    yarp::os::Bottle asBottle()
    {
        yarp::os::Bottle b;

        yarp::os::Bottle bKnown;
        bKnown.addString("knownVerbs");
        yarp::os::Bottle& bVerbs = bKnown.addList();
        for(std::map<std::string, VerbInfo>::iterator it = knownVerbs.begin();it!=knownVerbs.end();it++)
        {
            bVerbs.addString(it->second.verbalRoot.c_str());
        }

        b.addList()=bKnown;

        yarp::os::Bottle bIrr;
        bIrr.addString("irregularVerbs");
        yarp::os::Bottle& bVerbsIrr = bIrr.addList();
        for(std::map<std::string, VerbInfo>::iterator it = irregulars.begin();it!=irregulars.end();it++)
        {
            bVerbsIrr.addList() = it->second.asBottle();
        }

        b.addList()=bIrr;
        return b;
    }

    void fromBottle(yarp::os::Bottle b)
    {
        yarp::os::Bottle* knownsB = b.find("knownVerbs").asList();
        for(int i=0;i<knownsB->size();i++)
        {
            std::string verbalRoot = knownsB->get(i).asString().c_str();
            knownVerbs[verbalRoot] = getInfo(verbalRoot);
        }
                
        yarp::os::Bottle* irregularB = b.find("irregularVerbs").asList();
        for(int i=0;i<irregularB->size();i++)
        {
            VerbInfo info;
            info.fromBottle( (*irregularB->get(i).asList()) );
            irregulars[info.verbalRoot] = info;
        }
    }
    VerbInfo getInfo(std::string verbalRoot)
    {
        std::map<std::string, VerbInfo>::iterator location = irregulars.find(verbalRoot);
        if (location != irregulars.end())
        {
            return location->second;
        }

        VerbInfo info;
        info.verbalRoot=verbalRoot;

        info.participe=verbalRoot+"d";
        info.ingForm=verbalRoot+"ing";

        info.preteritForms.resize(5);
        for(int i=0;i<5;i++)
        {
            if (verbalRoot[verbalRoot.size()-1] == 'e')
                info.preteritForms[i] = verbalRoot+"d";
            else
                info.preteritForms[i] = verbalRoot+"ed";
        }

        info.presentForms.resize(5);
        info.presentForms[0] = verbalRoot;
        info.presentForms[1] = verbalRoot;
        info.presentForms[2] = verbalRoot+"s";
        info.presentForms[3] = verbalRoot;
        info.presentForms[4] = verbalRoot;
        return info;
    }

    //std::string getPresent(std::string verbalRoot, int person)
    //{
    //    return getInfo(verbalRoot).presentForms[person];
    //}

    //std::string getPreterit(std::string verbalRoot)
    //{
    //    return getInfo(verbalRoot).preterit;
    //}

    //std::string getParticipe(std::string verbalRoot)
    //{
    //    return getInfo(verbalRoot).participe;
    //}
    bool isVerb(std::string word, VerbStatus *status)
    {
        for(std::map<std::string, VerbInfo>::iterator v = irregulars.begin(); v!= irregulars.end(); v++)
        {
            //Check present
            for(int i=0; i<5;i++)
            {
                if (v->second.presentForms[i] == word)
                {
                    status->verbalRoot = v->second.verbalRoot;
                    status->person = i;
                    status->tense = Present;
                    status->mod = Ponctual;
                    return true;
                }
            }

            //Check past
            for(int i=0; i<5;i++)
            {
                if (v->second.preteritForms[i] == word || v->second.participe == word )
                {
                    status->verbalRoot = v->second.verbalRoot;
                    status->person = i;
                    status->tense = Past;
                    status->mod = Ponctual;
                    return true;
                }
            }

            //Check Continuous
            if (v->second.ingForm == word)
            {
                status->verbalRoot = v->second.verbalRoot;
                status->person = 0;
                status->tense = Present;
                status->mod = Continuous;
                return true;
            }       
        }

        //Check regulars
        for(std::map<std::string, VerbInfo>::iterator v = knownVerbs.begin(); v!= knownVerbs.end(); v++)
        {
            //Check present
            for(int i=0; i<5;i++)
            {
                if (v->second.presentForms[i] == word)
                {
                    status->verbalRoot = v->second.verbalRoot;
                    status->person = i;
                    status->tense = Present;
                    status->mod = Ponctual;
                    return true;
                }
            }

            //Check past
            for(int i=0; i<5;i++)
            {
                if (v->second.preteritForms[i] == word || v->second.participe == word )
                {
                    status->verbalRoot = v->second.verbalRoot;
                    status->person = i;
                    status->tense = Past;
                    status->mod = Ponctual;
                    return true;
                }
            }

            //Check Continuous
            if (v->second.ingForm == word)
            {
                status->verbalRoot = v->second.verbalRoot;
                status->person = 0;
                status->tense = Present;
                status->mod = Continuous;
                return true;
            }       
        }
        return false;
    }

    void getVerbFromWord(std::string word, std::string &root, Tense &tense, TenseModifier &mod, bool &thirdPerson)
    {
        thirdPerson = false;

        //Continuous
        size_t pos = word.find("ing");
        if (pos != std::string::npos && pos == word.length()-3)
        {
            mod = Continuous;
            tense = Present;
            root = word.substr(0,word.length()-3);
            return;
        }

        //Past
        pos = word.find("ed");
        if (pos != std::string::npos && pos == word.length()-2)
        {
            mod = Ponctual;
            tense = Past;
            root = word.substr(0,word.length()-2);
            return;
        }        
        pos = word.find("en");
        if (pos != std::string::npos && pos == word.length()-2)
        {
            mod = Ponctual;
            tense = Past;
            root = word.substr(0,word.length()-2);
            return;
        }

        //Present, third person
        pos = word.find("s");
        if (pos != std::string::npos && pos == word.length()-1)
        {
            mod = Ponctual;
            tense = Present;
            root = word.substr(0,word.length()-1);
            thirdPerson = true;
            return;
        }

        //Present, other person
        mod = Ponctual;
        tense = Present;
        root = word;
    }
};

struct Sentence
{
    wysiwyd::wrdac::Relation         relation;
    Goal                            goal;
    Tense                           tense;
    GForm                           form;
    TenseModifier                   mod;
    std::string                     sentence;

    std::string                     getType()
    {
        std::ostringstream oss;
        oss<<goal<<" "<<tense<<" "<<form<<" "<<mod;
        return oss.str();
    }
    std::string                     toString()
    {
            std::ostringstream oss;
            oss<<"Sentence is: "<<std::endl
            <<"Goal: "<<enum2str(goal)<<std::endl
            <<"Tense: "<<enum2str(tense)<<std::endl
            <<"Mod: "<<enum2str(mod)<<std::endl
            <<"Form: "<<enum2str(form)<<std::endl
            <<"Relation: "<<relation.toString()<<std::endl;
            return oss.str();
    }
};

class Grammar
{
private:
    std::list<Sentence> corpus;    
    std::map<std::string, std::string> abstractCorpus;
    Conjugator          conjugator;

    std::list<std::string> knownSubjects;
    std::list<std::string> knownObjects;
    std::list<std::string> knownCP;
    std::list<std::string> knownCT;
    std::list<std::string> knownCM;

    void cinSentence(Sentence &s)
    {
        //We gather info about the sentence to come
        std::string subject;
        std::string verb;
        std::string object;
        std::string cm;
        std::string cp;
        std::string ct;
        //system("cls");
        std::cout<<s.toString();
        std::cout<<"Subject> ";
        std::cin>> subject;
        std::cout<<std::endl;
                
        //system("cls");
        //std::cout<<s.toString()<<std::endl;
        std::cout<<"Verb> ";
        std::cin>> verb;
        std::cout<<std::endl;
        //system("cls");
        //std::cout<<s.toString()<<std::endl;
        std::cout<<"Object (can be \"none\"): ";
        std::cin>> object;
        std::cout<<std::endl;
        //system("cls");
        //std::cout<<s.toString()<<std::endl;
        std::cout<<"Comp. Place (can be \"none\"): ";
        std::cin>> cp;
        std::cout<<std::endl;
        //system("cls");
        //std::cout<<s.toString()<<std::endl;
        std::cout<<"Comp. Time (can be \"none\"): ";
        std::cin>> ct;
        std::cout<<std::endl;
        //system("cls");
        //std::cout<<s.toString()<<std::endl;
        std::cout<<"Comp. Manner (can be \"none\"): ";
        std::cin>> cm;
        std::cout<<std::endl;

        //We use the info to build the relation
        wysiwyd::wrdac::Relation relation(subject,verb,object,cp,ct,cm);
        s.relation = relation;
                
        //system("cls");
        std::cout<<s.toString()<<std::endl;
        //We finally get the sentence
        std::cout<<">>";
        std::getline(std::cin>>std::ws,s.sentence);
        std::cout<<std::endl;
    }


    std::pair<std::string,std::string> abstractSentence(Sentence sent)
    {
        std::pair<std::string,std::string> rPair;
        //We add the verb to the known verbs if it not irregular
        if (conjugator.irregulars.find(sent.relation.verb())==conjugator.irregulars.end())
            conjugator.knownVerbs[sent.relation.verb()]=conjugator.getInfo(sent.relation.verb());
        rPair.first = sent.getType();
        rPair.second = sent.sentence;
        wysiwyd::wrdac::replace_all(rPair.second,sent.relation.subject(),"#S");
        wysiwyd::wrdac::replace_all(rPair.second,sent.relation.object(),"#O");
        wysiwyd::wrdac::replace_all(rPair.second,sent.relation.complement_manner(),"#CM");
        wysiwyd::wrdac::replace_all(rPair.second,sent.relation.complement_place(),"#CP");
        wysiwyd::wrdac::replace_all(rPair.second,sent.relation.complement_time(),"#CT");

        //Abstract conjugated verbs
        //Split the sentence
        std::string buf; 
        std::stringstream ss(rPair.second);
        std::vector<std::string> words;
        while (ss >> buf)
            words.push_back(buf);
        for(std::vector<std::string>::iterator w = words.begin(); w!=words.end();w++)
        {
            if (*w != "#S" && *w != "#O" && *w != "#CM" && *w != "#CP" && *w != "#CT")
            {
                VerbStatus status;
                if (conjugator.isVerb(*w,&status))
                {
                    //replace the verb with #V#verbalRoot(,,)
                    (*w) = status.getGrammarExpression();
                }
                else
                {
                    //We do nothing, it is an unhandled word...
                }
            }
        }
        //Fuse the words
        rPair.second = "";
        for(std::vector<std::string>::iterator w = words.begin(); w!=words.end();w++)
        {
            rPair.second += (*w) +" ";
        }
        return rPair;
    }

    void addToCorpus(Sentence sent)
    {            
        std::cout<<"[addToCorpus] > adding "<<sent.sentence<<std::endl;

        corpus.push_back(sent);
        std::pair<std::string,std::string> p = abstractSentence(sent);

        //Expand vocabulory
        if (std::find(knownSubjects.begin(),knownSubjects.end(),sent.relation.subject())==knownSubjects.end())
        {
            knownSubjects.push_back(sent.relation.subject());
        }
                
        if (std::find(knownObjects.begin(),knownObjects.end(),sent.relation.object())==knownObjects.end())
        {
            knownObjects.push_back(sent.relation.object());
        }
                
        if (std::find(knownCM.begin(),knownCM.end(),sent.relation.complement_manner())==knownCM.end())
        {
            knownCM.push_back(sent.relation.complement_manner());
        }
        
        if (std::find(knownCT.begin(),knownCT.end(),sent.relation.complement_time())==knownCT.end())
        {
            knownCT.push_back(sent.relation.complement_time());
        }
                        
        if (std::find(knownCP.begin(),knownCP.end(),sent.relation.complement_place())==knownCP.end())
        {
            knownCP.push_back(sent.relation.complement_place());
        }

        //Expand corpus
        if (abstractCorpus.find(p.first)==abstractCorpus.end())
        {
            abstractCorpus[p.first]=p.second;
            std::cout<<"[addToCorpus] > abstract form is : "<<p.second<<std::endl;
        }
        else
        {
            std::cout<<"[addToCorpus] > Status already present. Skipped."<<std::endl;
        }
    }

public:

    Grammar()
    {
        //Initialise some verbs
        VerbInfo info;
        info.presentForms.resize(5);
        info.preteritForms.resize(5);

        info.verbalRoot = "be";
        info.presentForms[0] = "am";
        info.presentForms[1] = "are";
        info.presentForms[2] = "is";
        info.presentForms[3] = "are";
        info.presentForms[4] = "are";

        info.preteritForms[0] = "was";
        info.preteritForms[1] = "were";
        info.preteritForms[2] = "was";
        info.preteritForms[3] = "were";
        info.preteritForms[4] = "were";

        info.participe = "been";
        info.ingForm = "being";
        this->conjugator.irregulars[info.verbalRoot] = info;

        info.verbalRoot = "have";
        info.presentForms[0] = "have";
        info.presentForms[1] = "have";
        info.presentForms[2] = "has";
        info.presentForms[3] = "have";
        info.presentForms[4] = "have";

        info.preteritForms[0] = "had";
        info.preteritForms[1] = "had";
        info.preteritForms[2] = "had";
        info.preteritForms[3] = "had";
        info.preteritForms[4] = "had";

        info.participe = "had";
        info.ingForm = "having";
        this->conjugator.irregulars[info.verbalRoot] = info;
                
        info.verbalRoot = "do";
        info.presentForms[0] = "do";
        info.presentForms[1] = "do";
        info.presentForms[2] = "does";
        info.presentForms[3] = "do";
        info.presentForms[4] = "do";
        info.preteritForms[0] = "did";
        info.preteritForms[1] = "did";
        info.preteritForms[2] = "did";
        info.preteritForms[3] = "did";
        info.preteritForms[4] = "did";
        info.participe = "done";
        info.ingForm = "doing";
        this->conjugator.irregulars[info.verbalRoot] = info;

        info.verbalRoot = "will";
        info.presentForms[0] = "will";
        info.presentForms[1] = "will";
        info.presentForms[2] = "will";
        info.presentForms[3] = "will";
        info.presentForms[4] = "will";
        info.preteritForms[0] = "will";
        info.preteritForms[1] = "will";
        info.preteritForms[2] = "will";
        info.preteritForms[3] = "will";
        info.preteritForms[4] = "will";
        info.participe = "will";
        info.ingForm = "will";
        this->conjugator.irregulars[info.verbalRoot] = info;
    }

    void learnIrregularVerb()
    {
        std::cout<<"Learn irregular verb:"<<std::endl;
        VerbInfo info;
        info.presentForms.resize(5);
        std::cout<<"verbal root?"<<std::endl;
        std::cin>>info.verbalRoot;
        for(int p = 0; p<5;p++)
        {
            std::cout<<"Present form, subject="<<p<<std::endl;
            std::cin>>info.presentForms[p];
        }
        for(int p = 0; p<5;p++)
        {
            std::cout<<"Preterit form, subject="<<p<<std::endl;
            std::cin>>info.preteritForms[p];
        }
        std::cout<<"Participate?"<<std::endl;
        std::cin>>info.participe;
        std::cout<<"ING form?"<<std::endl;
        std::cin>>info.participe;
        this->conjugator.irregulars[info.verbalRoot] = info;
    }

    void learnStep()
    {
        Sentence s;

        //Get the next unfilled status
        for ( int g = Ask; g <= Describe; g++ )
        {
            s.goal = static_cast<Goal>(g);
            for ( int f = Active; f <= Passive; f++ )
            {
                s.form = static_cast<GForm>(f);
                for ( int tm = Ponctual; tm <= Continuous; tm++ )
                {
                    s.mod = static_cast<TenseModifier>(tm);
                    for ( int t = Present; t <= future; t++ )
                    {
                        s.tense = static_cast<Tense>(t);
                        if (abstractCorpus.find(s.getType())==abstractCorpus.end())
                        {
                            //Get the sentence info from the user
                            cinSentence(s);
                            //And pass the baby to abstraction
                            addToCorpus(s);
                            return;
                        }
                    }
                }
            }
        }
    }

    void printStatus()
    {
        std::cout<<"*************GRAMMAR STATUS*****************"<<std::endl;
        std::cout<<"Irregular verbs:"<<std::endl;
        for(std::map<std::string, VerbInfo>::iterator it = conjugator.irregulars.begin(); it != conjugator.irregulars.end(); it++)
        {
            std::cout<<it->second.toString();
        }

        std::cout<<"Regular verbs knowns:"<<std::endl;
        for(std::map<std::string, VerbInfo>::iterator it = conjugator.knownVerbs.begin(); it != conjugator.knownVerbs.end(); it++)
        {
            std::cout<<it->second.verbalRoot<<" ";
        }
        std::cout<<std::endl;

        std::cout<<"Grammar completion: ";
        int maximumEntries = 2*2*2*3;
        int currentEntries = abstractCorpus.size();
        std::cout<<currentEntries<<" / "<<maximumEntries<<std::endl;
        std::cout<<"*******************************************"<<std::endl;
    }

    void save2file(std::string fileName)
    {
        std::ofstream f(fileName.c_str());
        f<<asBottle().toString().c_str()<<std::endl;
        f.close();
    }

    void loadFile(std::string fileName)
    {
        std::ifstream f(fileName.c_str());
        if (!f.is_open())
            return;
        std::string line;
        getline(f,line);
        yarp::os::Bottle bot;
        bot.fromString(line.c_str());
        this->fromBottle(bot);
        f.close();
        printStatus();
    }

    yarp::os::Bottle asBottle()
    {
        yarp::os::Bottle b;
        
        yarp::os::Bottle subCorpus;
        subCorpus.addString("abstractCorpus");
        yarp::os::Bottle& subCorpus2 = subCorpus.addList();
        for(std::map<std::string, std::string>::iterator it = abstractCorpus.begin(); it != abstractCorpus.end(); it++)
        {
            yarp::os::Bottle pairB;
            pairB.addString(it->first.c_str());
            pairB.addString(it->second.c_str());
            subCorpus2.addList() = pairB;
        }        
        b.addList()=subCorpus;

        yarp::os::Bottle subConjug;
        subConjug.addString("conjugator");
        subConjug.addList() = conjugator.asBottle();
        b.addList()=subConjug;
        return b;
    }

    void fromBottle(yarp::os::Bottle b)
    {
        yarp::os::Bottle *subCorpus = b.find("abstractCorpus").asList();
        for(int i=0; i<subCorpus->size(); i++)
        {
            std::string debu = subCorpus->toString().c_str();
            std::string key = subCorpus->get(i).asList()->get(0).asString().c_str();
            std::string val = subCorpus->get(i).asList()->get(1).asString().c_str();
            abstractCorpus[key]=val;
        }        
        conjugator.fromBottle(*b.find("conjugator").asList());
    }
};

}
}
}
}
