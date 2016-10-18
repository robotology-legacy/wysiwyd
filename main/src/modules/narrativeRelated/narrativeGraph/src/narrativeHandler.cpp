/*
* Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Grégoire Pointeau
* email:   gregoire.pointeau@inserm.fr
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

#include "narrativeHandler.h"
#include <stack>
#include <set>
#include <map>

using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;

bool narrativeHandler::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("narrativeGraph")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << moduleName << ": finding configuration files...";
    period = rf.check("period", Value(0.1)).asDouble();

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "narrativeGraph", "narrativeGraph.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

    // get grammar file
    GrammarNarration = rf.findFileByName(rf.check("GrammarNarration", Value("GrammarNarration.xml")).toString());
    GrammarYesNo = rf.findFileByName(rf.check("GrammarYesNo", Value("nodeYesNo.xml")).toString());

    if (!iCub->connect())
    {
        yInfo() << "iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }

    dThresholdDiffStory = rf.check("dThresholdDiffStory", Value(15.)).asDouble();
    iThresholdSizeStory = rf.check("iThresholdSizeStory", Value(6)).asInt();
    iThresholdSentence = rf.check("iThresholdSentence", Value(6)).asInt();
    iThresholdScoreIGARFPAOR = rf.check("iThresholdScoreIGARFPAOR", Value(7)).asInt();
    instanceStart = rf.check("instanceStart", Value(0)).asInt();
    instanceStop = rf.check("instanceStop", Value(100000)).asInt();
    storyToNarrate = rf.check("storyToNarrate", Value(1770)).asInt();
    nBackSize = rf.check("nBackSize", Value(200)).asInt();
    narrator = rf.check("narrator", Value("Narrator")).asString().c_str();
    lrh = rf.find("lrh").asInt() == 1;
    researchWindows = rf.find("researchWindows").asInt() == 1;

    Bottle *bNarration = rf.find("listScenario").asList();
    initializeScenarios(*bNarration, rf);

    Bottle *bMeaning = rf.find("listMeaning").asList();
    initializeMeaning(*bMeaning, rf);

    if (rf.find("loadNaives").asInt() == 1){
        Bottle *bNaives = rf.find("listNaives").asList();
        initializeNaives(*bNaives, rf);
        NaiveToPAOR();
    }

    shouldSpeak = rf.find("shouldSpeak").asInt() == 1;

    sIGARFfile = rf.findFile(rf.check("IGARFfile", Value("igarffile.txt")).toString());


    // Closed Class Words (from LRH)
    yarp::os::ResourceFinder rfLRH;
    rfLRH.setVerbose(isRFVerbose);
    rfLRH.setDefaultConfigFile("lrh.ini");
    rfLRH.setDefaultContext("lrh/conf");
    rfLRH.configure(0, NULL);
    string sclosed_class_words = rfLRH.check("closed_class_words", Value("to")).toString().c_str();
    vector <string> vCCW = split(sclosed_class_words, ',');
    for (string w : vCCW)
        storygraph::VocabularyHandler::addCCW(w);
    string sdiscourse_function_words = rfLRH.check("discourse_function_words", Value("and")).toString().c_str();
    vector <string> vDFW = split(sdiscourse_function_words, ',');
    for (string w : vDFW)
        storygraph::VocabularyHandler::enrichDFW(w);

    Bottle &bNarrativeGroup = rf.findGroup("narrative");
    if (!bNarrativeGroup.isNull()) {
        // Train output
        string trainOuputFileName = rfLRH.findFileByName(bNarrativeGroup.check("trainOutput", Value("Corpus/trainOutput.txt")).toString());
        fTrainOutput.open(trainOuputFileName, ofstream::out | ofstream::app);
        if (!fTrainOutput.is_open())
            yWarning() << "Train Ouput file not found.";
        else
            fTrainOutput << "\n";

        // Narrative input
        sNarrativeFolderName = rf.findPath(bNarrativeGroup.check("narrativeFolder", Value("Corpus")).toString()) + "/";
        sNarrativeFileName = bNarrativeGroup.check("narrativeFile", Value("narrative.txt")).toString();

        // Synonyms
        ifstream fSynonyms;
        string sSynonymFileName = rf.findFileByName(bNarrativeGroup.check("synonyms", Value("vocabSynonyms.txt")).toString());
        fSynonyms.open(sSynonymFileName);
        if (fSynonyms.is_open()) {
            while (fSynonyms.good()) {
                string s;
                getline(fSynonyms, s);
                vector<string> vWords = split(s, ' ');
                set<string> setWords(vWords.begin(), vWords.end());
                storygraph::VocabularyHandler::enrichSynonyms(setWords);
            }
            fSynonyms.close();
        }

        // Pronouns
        string pronouns = bNarrativeGroup.check("pronouns", Value("he,it,him")).toString();
        vector <string> vPronouns = split(pronouns, ',');
        storygraph::VocabularyHandler::setPronouns(vPronouns.at(0), vPronouns.at(1), vPronouns.at(2));
    }

    // Rendering
    Bottle &bSizes = rf.findGroup("rendering");
    if (!bSizes.isNull()) {
        sSVGFolderName = rf.findPath(bSizes.check("svgFolder", Value("situationModels")).asString()) + "/";
        sSVGFileName = bSizes.check("svgFile", Value("situationModel.svg")).toString();
        int wEvtBox = bSizes.check("wEvtBox", Value(160)).asInt();
        int hEvtBox = bSizes.check("hEvtBox", Value(80)).asInt();
        int hOffset = bSizes.check("hOffset", Value(40)).asInt();
        int vOffset = bSizes.check("vOffset", Value(70)).asInt();
        sm.initSizes(wEvtBox, hEvtBox, hOffset, vOffset);
    }

    yInfo() << "Rendering options:\n"
        << " sSVGFolderName: " << sSVGFolderName << "\n"
        << " sSVGFileName: " << sSVGFileName;

    cursorStories = 0;

    //rpc port
    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    if (!iCub->getABMClient())
    {
        yWarning() << "ABM not connected. narrativeHandler cannot start.";
        return false;
    }

    if (!iCub->getLRH())
        yWarning("LRH not connected");
    else
        iCub->getLRH()->bForwardABM = (rf.find("forwardABM").asInt() == 1);

    if (!iCub->getRecogClient())
    {
        iCub->say("Narrative handler warning speech recognizer not connected!");
        yWarning() << "Speech recognizer not connected!";
    }

    if (!Port2abmReasoning.open("/" + getName() + "ABMR")) {
        yWarning() << ": Unable to open port " << Port2abmReasoning.getName();
    }
    if (Network::connect(Port2abmReasoning.getName(), "/abmReasoning/rpc")){
        yWarning(" Cannot connect to ABMReasoning");
    }

    mentalOPC = new OPCClient(getName() + "/mentalOPC");
    int iTry = 0;
    while (!mentalOPC->isConnected())
    {
        yInfo() << "\t" << "narrativeHandler Connecting to mentalOPC ..." << mentalOPC->connect("mentalOPC");
        if (!mentalOPC->isConnected())
            Time::delay(0.5);
        iTry++;
        if (iTry > 1)
        {
            yInfo() << "\t" << "narrativeHandler failed to connect to mentalOPC";
            break;
        }
        mentalOPC->isVerbose = false;
    }

    if (mentalOPC->isConnected()){
        mentalOPC->checkout();
        mentalOPC->update();
    }



    std::string ttsOptions = rf.check("ttsOptions", yarp::os::Value("iCub")).toString();
    if (ttsOptions != "iCub") {
        if (iCub->getSpeechClient())
            iCub->getSpeechClient()->SetOptions(ttsOptions);
    }

    bInitial = true;
    counter = 0;
    findStories();


    storygraph::VocabularyHandler::initVoc(listStories);

    //    narrationToMeaning();

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";


    //    cout << "linking scenarios 3 3" << endl;
    //linkMeaningScenario(4,3);
    //cout << "linking scenarios 2 2" << endl;
    //linkMeaningScenario(2, 2);
    //cout << endl << endl;
    //displayDFW();

    /*
        ofstream IGARFfile;
        IGARFfile.open(sIGARFfile);
        listStories[0].displayNarration();
        sm.ABMtoSM(listStories.at(0), IGARFfile);*/

    return true;
}


bool narrativeHandler::interruptModule() {
    rpcPort.interrupt();
    mentalOPC->interrupt();

    yInfo() << "--Interrupting the synchronized yarp ports module...";
    return true;
}

bool narrativeHandler::close() {
    iCub->close();
    delete iCub;

    mentalOPC->close();
    delete mentalOPC;

    rpcPort.interrupt();
    rpcPort.close();

    fTrainOutput.close();

    return true;
}


bool narrativeHandler::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        " setNarrator + name: \n" +
        " addHumanNarration + instanceStory (default_value) + type (speech - default, text, auto) + scenario (if auto. default 1) \n" +
        " displayKnownNarrations \n" +
        " checkScenarios + scenario_number = all \n" +
        " narrate + instanceStory = default_value \n" +
        " displayKnownStories \n" +
        " displayStories + n-back = default_all \n" +
        " listenStory\n" +
        " cleanMental\n" +
        " cleanSM\n"
        " cleanLinks\n"
        " cleanDFW\n"
        " exportDFW\n" +
        " useDFW\n" +
        " ABMtoSM + storyNumber = last\n"
        " linkNarrationScenario + iNarration + iScenario\n" +
        " linkMeaningScenario + iNarration + iScenario\n" +
        " autoStructSM\n"
        " helpSM\n" +
        " SMtoTrain + sentence\n" +
        " SMandNarrativetoTrain + filename = trainOutput\n" +
        " SMtoLRH + lang = en\n" +
        " SMtoSVG file = svgFile + instanceIGARF = last\n" +
        " learnDFW + word\n" +
        " createLink + (instanceFromIGARF cPart instanceRel) + word + (instanceToIGARF cPart instanceRel)\n" +
        " LRHtoSM + meaning\n" +
        " LRHFiletoSM + filename"
        " LRHtoBlankSM + meaning\n" +
        " period\n" +
        " autoLink + instanceIGARF = last"
        " quit \n";

    yInfo() << " rpc command received: " << command.toString();

    reply.clear();
    reply.addVocab(Vocab::encode("many"));

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");

        rpcPort.reply(reply);
        return false;
    }
    else if (command.get(0).asString() == "setNarrator"){
        if (command.size() == 2){
            narrator = command.get(1).asString();
            reply.addString("narrator set to " + narrator);
        }
        else{
            reply.addString("error in narrativeHandler::setNarrator wrong size of input: should be: (setNarrator narrator)");
        }
    }
    else if (command.get(0).asString() == "cleanMental"){
        cleanMental();
    }
    else if (command.get(0).asString() == "displayKnownStories"){
        for (unsigned int ii = 0; ii < listStories.size(); ii++){
            cout << "story: " << ii << " -- instance start: " << listStories[ii].viInstances[0] << " -- size: " << listStories[ii].viInstances.size() << endl;
        }
    }
    else if (command.get(0).asString() == "checkScenarios"){
        int iScena = -1;
        if (command.size() == 2){
            iScena = command.get(1).asInt();
        }
        checkScenarios(iScena);
    }
    else if (command.get(0).asString() == "listeningStory"){
        listeningStory();
    }
    else if (command.get(0).asString() == "linkNarrationScenario"){
        if (command.size() != 3){
            reply.addString("linkNarrationScenario takes 2 arguments (narration and scenario)");
        }
        else{
            linkNarrationScenario(command.get(1).asInt(), command.get(2).asInt());
        }
    }
    else if (command.get(0).asString() == "linkMeaningScenario"){
        if (command.size() != 3){
            reply.addString("linkMeaningScenario takes 2 arguments (narration and scenario)");
        }
        else{
            reply.addString(linkMeaningScenario(command.get(1).asInt(), command.get(2).asInt() - 1));
        }
    }
    else if (command.get(0).asString() == "displayKnownNarrations"){
        for (map<int, vector<string>>::iterator itMp = listAutoScenarios.begin(); itMp != listAutoScenarios.end(); itMp++){
            cout << endl << "Narration number: " << itMp->first << endl;
            for (vector<string>::iterator itLi = itMp->second.begin(); itLi != itMp->second.end(); itLi++){
                cout << "\t" << *itLi << endl;
            }
        }
    }
    else if (command.get(0).asString() == "addHumanNarration"){
        int iIns = storyToNarrate;
        int iScena = 1;
        string sModality = "speech";

        if (command.size() > 1) {
            iIns = command.get(1).asInt();
        }
        if (command.size() > 2){
            sModality = command.get(2).asString();
        }
        if (command.size() > 3){
            iScena = command.get(3).asInt();
        }
        if (addHumanNarration(iIns, sModality, iScena)){
            reply.addString("ack");
        }
        else{
            reply.addString("nack");
        }
    }
    else if (command.get(0).asString() == "narrate"){
        int iIns = storyToNarrate;
        if (command.size() == 2) {
            iIns = command.get(1).asInt();
        }

        if (narrate(iIns)){
            reply.addString("ack");
        }
        else{
            reply.addString("nack");
        }
    }
    else if (command.get(0).asString() == "displayStories"){
        yInfo("Starting to display stories:");
        if (command.size() == 2){
            unsigned int nback = command.get(1).asInt();
            if (nback < listStories.size()){
                for (unsigned int jj = listStories.size() - nback; jj < listStories.size(); jj++){
                    listStories[jj].displayNarration();
                }
            }
            else{
                for (auto st : listStories){
                    st.displayNarration();
                }
            }
        }
        else{
            for (auto st : listStories){
                st.displayNarration();
            }
        }
        reply.addString("display finished");
    }
    else if (command.get(0).asString() == "setDefaultStory"){
        if (command.size() == 2) {
            storyToNarrate = command.get(1).asInt();
            yInfo() << " default instance of story set to: " << storyToNarrate;
            reply.addString("default instance of story set to " + storyToNarrate);
        }
        else{
            yWarning(" in narrativeHandler: command setDefaultStory: missing argument (int expected)");
            reply.addString(" in narrativeHandler: command setDefaultStory: missing argument (int expected)");
        }
    }
    else if (command.get(0).asString() == "cleanSM") {
        yInfo(" clean the situation model");
        sm.clear();
        yInfo(" cleaning sucessful");
        reply.addString("cleaning sucessful");
    }
    else if (command.get(0).asString() == "cleanLinks") {
        yInfo(" clean the links in the situation model");
        sm.cleanLinks();
        yInfo(" cleaning sucessful");
        reply.addString("cleaning sucessful");
    }
    else if (command.get(0).asString() == "cleanDFW") {
        yInfo(" clean the Discourse Function Words");
        vDFW.clear();
        yInfo(" cleaning sucessful");
        reply.addString("cleaning sucessful");
    }
    else if (command.get(0).asString() == "exportDFW") {
        yInfo(" exporting the Discourse Function Words");
        exportDFW();
        reply.addString("export sucessful");
    }
    else if (command.get(0).asString() == "useDFW") {
        yInfo(" using a Discourse Function Words");
        reply = useDFW(command);
    }
    else if (command.get(0).asString() == "ABMtoSM") {
        yInfo(" create the situation model from ABM");
        ofstream IGARFfile;
        IGARFfile.open(sIGARFfile);
        if (command.size() >= 2) {
            int i = command.get(1).asInt();
            if (i >= 0 && i < (int)listStories.size()) {
                loadSM(i);
                yInfo(" import and creation sucessful");
                reply.addString("creation sucessful");
            }
            else {
                yWarning() << " error: out of range, max: " << int(listStories.size() - 1);
                reply.addString("Error: out of range, max:");
                reply.addInt(int(listStories.size()) - 1);
            }
        }
        else {
            sm.ABMtoSM(listStories.back(), IGARFfile);
            yInfo(" import of last story and creation sucessful");
            reply.addString("creation sucessful");
        }
        IGARFfile.close();
    }
    else if (command.get(0).asString() == "autoStructSM") {
        yInfo(" auto structuration of the situation model");
        ofstream IGARFfile;
        IGARFfile.open(sIGARFfile);
        sm.makeStructure(IGARFfile);
        IGARFfile.close();
        yInfo(" autostruct sucessful");
        reply.addString("autostruct sucessful");
    }
    // -- Situation Model Related
    // Construction
    else if (command.get(0).asString() == "helpSM") {
        string help = string(getName().c_str()) +
            " situation model commands are: \n" +
            " listActionEvts\n" +
            " listRels\n" +
            " listIGARF\n" +
            " createEvt + predicate + agent + object = \"\" + recipient = \"\"\n" +
            " getRel + subject + verb + object\n" +
            " createIGARF\n" +
            " showIGARF + instanceIGARF\n" +
            " changeEvtIGARF + instanceIGARF + cPart + instanceEvt\n" +
            " changeCntIGARF + instanceIGARF + cPart + instanceSubIGARF\n" +
            " removeCntIGARF + instanceIGARF + cPart\n" +
            " addRelIGARF + instanceIGARF + cPart + instanceRel\n" +
            " remRelIGARF + instanceIGARF + cPart + instanceRel\n";
        reply.addString(help);
        cout << help << endl;
    }
    else if (command.get(0).asString() == "listActionEvts") {
        yInfo(" list events of the situation model");
        for (unsigned int i = 0; i < sm.vActionEvts.size(); i++) {
            std::cout << "[" << i << "] " << sm.getSentenceEvt(i) << std::endl;
            reply.addString(sm.getSentenceEvt(i));
        }
        yInfo(" listing sucessful");
        reply.addString(" listing sucessful");
    }
    else if (command.get(0).asString() == "listRels") {
        yInfo(" list relations of the situation model");
        for (unsigned int i = 0; i < sm.vRelations.size(); i++) {
            std::cout << "[" << i << "] " << sm.getSentenceRel(i) << std::endl;
            reply.addString(sm.getSentenceRel(i));
        }
        yInfo(" listing sucessful");
        reply.addString(" listing sucessful");
    }
    else if (command.get(0).asString() == "listIGARF") {
        yInfo(" list IGARF events of the situation model ");
        for (unsigned int i = 0; i < sm.vIGARF.size(); i++) {
            ofstream IGARFfile;
            IGARFfile.open(sIGARFfile);
            sm.showIGARF(i, IGARFfile);
            std::cout << std::endl;
            IGARFfile.close();
        }
        yInfo(" listing sucessful");
        reply.addString(" listing sucessful");
    }
    else if (command.get(0).asString() == "createEvt") {
        yInfo(" create an action event in the situation model");
        if (command.size() >= 3) {
            storygraph::sActionEvt a;
            a.predicate = command.get(1).asString();
            a.agent = command.get(2).asString();
            a.object = "";
            a.recipient = "";
            if (command.size() >= 4)
                a.object = command.get(3).asString();
            if (command.size() >= 5)
                a.recipient = command.get(4).asString();
            int i = sm.addNewActionEvt(a);
            yInfo(" creation sucessful of the " + to_string(i) + "-th action event");
            reply.addString(" creation sucessful of the " + to_string(i) + "-th action event");
        }
        else {
            yInfo(" Not enough arguments");
            reply.addString("Error: Needs at least 2 arguments");
        }
    }
    else if (command.get(0).asString() == "getRel") {
        yInfo(" add a relation in the situation model (if not already existing)");
        if (command.size() >= 4) {
            storygraph::sRelation r;
            r.subject = command.get(1).asString();
            r.verb = command.get(2).asString();
            r.object = command.get(3).asString();
            int i = sm.findRelation(r, true);
            yInfo(" creation sucessful of the " + to_string(i) + "-th relation");
            reply.addString(" creation sucessful of the " + to_string(i) + "-th relation");
        }
        else {
            yInfo(" Not enough arguments");
            reply.addString("Error: Needs 3 argument");
        }
    }
    else if (command.get(0).asString() == "createIGARF") {
        yInfo(" add an IGARF event in the situation model");
        int i = sm.createIGARF();
        yInfo(" creation sucessful of the " + to_string(i) + "-th IGARF");
        reply.addString(" creation sucessful of the " + to_string(i) + "-th IGARF");
    }
    else if (command.get(0).asString() == "showIGARF") {
        yInfo(" display tree view of an IGARF event ");
        if (command.size() >= 2) {
            int i = command.get(1).asInt();
            if (i >= 0 && i < (int)sm.vIGARF.size()) {
                ofstream IGARFfile;
                IGARFfile.open(sIGARFfile);
                sm.showIGARF(i, IGARFfile);
                IGARFfile.close();
                cout << endl;
                yInfo(" display sucessful");
                reply.addString(" display sucessful");
            }
            else {
                yInfo(" error: out of range");
                reply.addString("Error: out of range");
            }
        }
        else {
            yInfo(" Not enough arguments");
            reply.addString("Error: Needs 1 argument");
        }
    }
    else if (command.get(0).asString() == "createLink") {
        addLink(command, reply);
    }
    else if (command.get(0).asString() == "changeEvtIGARF") {
        yInfo(" change an IGARF event in the situation model");
        if (command.size() >= 4) {
            int  iIGARF = command.get(1).asInt();
            char cPart = command.get(2).asString()[0];
            int  iAct = command.get(3).asInt();

            sm.modifEventIGARF(iIGARF, cPart, iAct);
            yInfo(" modification sucessful");
            reply.addString(" modification sucessful");
        }
        else {
            yInfo(" Not enough arguments");
            reply.addString("Error: Needs 3 arguments");
        }
    }
    else if (command.get(0).asString() == "changeCntIGARF") {
        yInfo(" change an IGARF content in the situation model");
        if (command.size() >= 4) {
            int  iIGARF = command.get(1).asInt();
            char cPart = command.get(2).asString()[0];
            int  jIGARF = command.get(3).asInt();

            sm.modifContentIGARF(iIGARF, cPart, jIGARF);
            yInfo(" modification sucessful");
            reply.addString(" modification sucessful");
        }
        else {
            yInfo(" Not enough arguments");
            reply.addString("Error: Needs 3 arguments");
        }
    }
    else if (command.get(0).asString() == "removeCntIGARF") {
        yInfo(" remove an IGARF content in the situation model");
        if (command.size() >= 3) {
            int  iIGARF = command.get(1).asInt();
            char cPart = command.get(2).asString()[0];

            sm.remContentIGARF(iIGARF, cPart);
            yInfo(" removal sucessful");
            reply.addString(" removal sucessful");
        }
        else {
            yInfo(" Not enough arguments");
            reply.addString("Error: Needs 2 arguments");
        }
    }
    else if (command.get(0).asString() == "addRelIGARF") {
        yInfo(" add a relation to an IGARF event in the situation model");
        if (command.size() >= 4) {
            int  iIGARF = command.get(1).asInt();
            char cPart = command.get(2).asString()[0];
            int  iRel = command.get(3).asInt();

            sm.addRelationIGARF(iIGARF, cPart, iRel);
            yInfo(" addition sucessful");
            reply.addString(" addition sucessful");
        }
        else {
            yInfo(" Not enough arguments");
            reply.addString("Error: Needs 3 arguments");
        }
    }
    else if (command.get(0).asString() == "remRelIGARF") {
        yInfo(" remove a relation to an IGARF event in the situation model");
        if (command.size() >= 4) {
            int  iIGARF = command.get(1).asInt();
            char cPart = command.get(2).asString()[0];
            int  iRel = command.get(3).asInt();

            sm.removeRelationIGARF(iIGARF, cPart, iRel);
            yInfo(" removal sucessful");
            reply.addString(" removal sucessful");
        }
        else {
            yInfo(" Not enough arguments");
            reply.addString("Error: Needs 3 arguments");
        }
    }
    // -- LRH Related
    // Step 1: Train
    else if (command.get(0).asString() == "SMtoTrain") {
        yInfo(" create a train from a narrative sentence");
        if (command.size() >= 2) {
            sentenceToTrain(command.get(1).asString(), reply);
            sm.endSentence();
        }
        else {
            yInfo(" Not enough arguments");
            reply.addString("Error: Needs an argument (sentence)");
        }
    }
    else if (command.get(0).asString() == "SMandNarrativetoTrain") {
        yInfo(" create a train from a narrative");
        string name;
        if (command.size() > 1 && command.get(1).asString() != "")
            name = command.get(1).asString();
        else
            name = sNarrativeFileName;
        yInfo() << " Opening " << sNarrativeFolderName + name;
        ifstream fNarrativeInput;
        fNarrativeInput.open(sNarrativeFolderName + name);
        if (fNarrativeInput.is_open()) {
            while (fNarrativeInput.good()) {
                string s;
                getline(fNarrativeInput, s);
                sentenceToTrain(s, reply);
            }
            fNarrativeInput.close();
            sm.endSentence();
            yInfo(" creation successful");
            reply.addString(" creation successful:");
        }
        else {
            yInfo(" narrative not found");
            reply.addString(" narrative not found");
        }
    }
    // -- Narrative Links related
    // Step 2: Understand
    else if (command.get(0).asString() == "learnDFW") {
        yInfo(" add a DFW in the Vocabulary");
        if (command.size() >= 2) {
            string word = command.get(1).asString();
            storygraph::VocabularyHandler::enrichDFW(word);
            yInfo(" enrichment sucessful");
            reply.addString(" enrichment sucessful");
        }
        else {
            yInfo(" Not enough arguments");
            reply.addString("Error: Needs 1 argument");
        }
    }
    else if (command.get(0).asString() == "createLink") {
        addLink(command, reply);
    }
    else if (command.get(0).asString() == "LRHtoSM") {
        yInfo(" creating a link in the situation model");
        if (command.size() >= 2) {
            // Get meaning
            string m = command.get(1).asString();
            LRHtoSM(m, reply);
        }
        else {
            yInfo(" Not enough arguments");
            reply.addString("Error: Needs 1 arguments");
        }
    }
    else if (command.get(0).asString() == "LRHFiletoSM") {
        string name;
        if (command.size() > 1 && command.get(1).asString() != "") {
            name = command.get(1).asString();
            yInfo() << " Opening " << sNarrativeFolderName + name;
            ifstream fNarrativeInput;
            fNarrativeInput.open(sNarrativeFolderName + name);
            if (fNarrativeInput.is_open()) {
                while (fNarrativeInput.good()) {
                    string s;
                    getline(fNarrativeInput, s);
                    LRHtoSM(s, reply);
                }
                fNarrativeInput.close();
                sm.endSentence();
                yInfo(" creation successful");
                reply.addString(" creation successful:");
            }
        }
        else {
            yInfo(" narrative not found");
            reply.addString(" narrative not found");
        }
    }
    else if (command.get(0).asString() == "LRHtoBlankSM") {
        yInfo(" creating event and/or link in the situation model");
        if (command.size() >= 2) {
            // Get meaning
            string m = command.get(1).asString();
            LRHtoSM(m, reply, true);
        }
        else {
            yInfo(" Not enough arguments");
            reply.addString("Error: Needs 1 arguments");
        }
    }
    else if (command.get(0).asString() == "period") {
        yInfo(" end sentence");
        sm.endSentence();
        yInfo(" sentence ended");
        reply.addString(" sentence ended");
    }
    else if (command.get(0).asString() == "autoLink") {
        yInfo(" create links in the SM");
        int nIGARF;
        if (command.size() > 1)
            nIGARF = command.get(2).asInt();
        else
            nIGARF = sm.vIGARF.size() - 1;
        if (nIGARF >= 0 && nIGARF < (int)sm.vIGARF.size()) {
            sm.autoLink(nIGARF);
            yInfo(" creation successful");
            reply.addString(" creation sucessful");
        }
        else {
            yInfo(" error: out of range");
            reply.addString("Error: out of range");
        }
    }
    else if (command.get(0).asString() == "SMtoLRH") {
        yInfo(" produce basic meaning using links");
        if (command.size() > 1 && command.get(1).asString() != "")
            sm.SMtoLRH(command.get(1).asString());
        else
            sm.SMtoLRH();
        yInfo(" production successful");
        reply.addString(" production sucessful");
    }
    else if (command.get(0).asString() == "listLinks") {
        for (storygraph::sDiscourseLink lk : sm.vDiscourseLinks) {
            cout << lk.word << ": From " << lk.fromEvt.cPart << " of IGARF " << lk.fromEvt.iIGARF
                << " to " << lk.toEvt.cPart << " of IGARF " << lk.toEvt.iIGARF << endl;
        }
        yInfo(" listing sucessful");
        reply.addString(" listing sucessful");
    }
    // --
    else if (command.get(0).asString() == "SMtoSVG") {
        yInfo(" create a SVG representation of an IGARF");
        string name = to_string(sm.instanceBegin);
        name += "_";
        int nIGARF;
        if (command.size() > 1 && command.get(1).asString() != "")
            name += command.get(1).asString();
        else
            name += sSVGFileName;
        if (command.size() > 2)
            nIGARF = command.get(2).asInt();
        else
            nIGARF = sm.vIGARF.size() - 1;
        yInfo() << "Opening " << sSVGFolderName + name;
        ofstream fSVG_Output;
        fSVG_Output.open(sSVGFolderName + name);
        if (fSVG_Output.is_open()) {
            sm.writeSVG(fSVG_Output, nIGARF);
            fSVG_Output.close();
            yInfo(" creation successful");
            reply.addString(" creation sucessful");
        }
        else {
            yInfo(" Error: Can't open file");
            reply.addString(" Error: Can't open file");
        }
    }
    else if (command.get(0).asString() == "isActionVoc") {
        bool is = storygraph::VocabularyHandler::isActionVoc(command.get(1).asString());
        reply.addString(is ? "yes" : "no");
        yInfo(is ? " yes" : " no");
    }
    else{
        reply.addString(helpMessage);
        cout << helpMessage << endl;
    }

    rpcPort.reply(reply);

    return true;
}

/* Called periodically every getPeriod() seconds */
bool narrativeHandler::updateModule() {

    return true;
}


void narrativeHandler::findStories()
{
    // initial: if it is the initialisation: 2 choices: window or n-back
    // else only from last story

    ostringstream osRequest;
    if (bInitial){
        if (researchWindows){
            yInfo() << " BEGIN FINDSTORIES INITIAL from: " << instanceStart << " to " << instanceStop;
            if (instanceStart >= instanceStop){
                yWarning(" in narrativeGraph::narrativeHandler.cpp::findStories instanceStart >= instanceStop.");
            }
            osRequest << "SELECT time, begin, instance FROM main WHERE instance between " << instanceStart << " and " << instanceStop << " ORDER by instance";

        }

        else{
            yInfo() << " BEGIN FINDSTORIES INITIAL n-back: last " << nBackSize;
            osRequest << "SELECT time, begin, instance FROM main WHERE instance in (SELECT instance FROM main ORDER BY instance DESC LIMIT " << nBackSize << ") ORDER by instance";
        }
    }
    else{
        yInfo() << " BEGIN FINDSTORIES from " << instanceStart;
        osRequest << "SELECT time, begin, instance FROM main WHERE instance > " << instanceStart;
    }

    bInitial = false;

    story currentStory;
    currentStory.iThresholdSentence = iThresholdSentence;

    //    int iCurrentInstance = iInstance;

    Bottle  bAllInstances = iCub->getABMClient()->requestFromString(osRequest.str());
    // Bottle bMessenger;
    int numberInstances = bAllInstances.size();

    // vector<int> vError;
    yInfo() << "\t" << "found " << numberInstances << " instance(s)";
    double mDiff;
    int Id2 = 0;
    for (int j = 1; j < (numberInstances); j++)
    {

        //Id = atoi(bAllInstances.get(j - 1).asList()->get(2).toString().c_str());
        Id2 = atoi(bAllInstances.get(j).asList()->get(2).toString().c_str());

        string sT1 = (bAllInstances.get(j - 1).asList()->get(0).toString().c_str());
        string sT2 = (bAllInstances.get(j).asList()->get(0).toString().c_str());

        myTimeStruct m1 = string2Time(sT1),
            m2 = string2Time(sT2);

        mDiff = timeDiff(m1, m2);

        if (strcmp((bAllInstances.get(j).asList()->get(1).toString().c_str()), "f") == 1)
        {
            if (mDiff > dThresholdDiffStory){
                if (currentStory.viInstances.size() > iThresholdSizeStory)
                {
                    currentStory.counter = counter;
                    counter++;
                    listStories.push_back(currentStory);
                }
                currentStory.viInstances.clear();
                currentStory.viInstances.push_back(Id2);
            }
            else{
                currentStory.viInstances.push_back(Id2);
            }
        }
        else{
            currentStory.viInstances.push_back(Id2);
        }
    }

    if (currentStory.viInstances.size() > iThresholdSizeStory)
    {
        currentStory.counter = counter;
        currentStory.iBasedOn = -1;
        counter++;
        listStories.push_back(currentStory);
    }

    //for (auto& itSt : listStories)
    //{
    //    cout << "Story " << ii << ": ";
    //    for (auto& itIn : itSt.viInstances)
    //    {
    //        cout << itIn << " ";
    //    }
    //    cout << endl;
    //    osRequest.str("");
    //    osRequest << "SELECT subject, verb, object FROM relation WHERE instance = " << *itSt.viInstances.begin() << " AND verb != 'isAtLoc'";
    //    bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
    //    //if (bMessenger.toString() != "NULL")   cout << "before: " << bMessenger.toString() << endl;

    //    osRequest.str("");
    //    osRequest << "SELECT subject, verb, object FROM relation WHERE instance = " << itSt.viInstances[itSt.viInstances.size() - 1] << " AND verb != 'isAtLoc'";
    //    bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
    //    //if (bMessenger.toString() != "NULL")   cout << "after : " << bMessenger.toString() << endl;

    //    ii++;
    //}

    initializeStories();
    findNarration();

    instanceStart = Id2;


    yInfo(" END FINDSTORIES ");

    yInfo() << " Found: " << listStories.size() << " stories.";
}


void narrativeHandler::findNarration()
{
    yInfo() << " BEGIN SEARCH NARRATION";

    //    int iCurrentInstance = iInstance;

    ostringstream osRequest;
    Bottle bListInstances;


    for (auto &target : listStories){
        int iTarget = target.viInstances[0];

        osRequest.str("");
        osRequest << "SELECT instance, role, argument FROM contentarg WHERE instance in (";
        osRequest << "SELECT contentarg.instance FROM contentarg, main WHERE contentarg.instance = main.instance and main.activitytype = 'narration' and contentarg.role = 'story' and contentarg.argument = '"
            << iTarget << "' order by contentarg.instance"
            << ") order by instance";
        bListInstances = iCub->getABMClient()->requestFromString(osRequest.str());
        int iCurrentElm = 0;
        bool stop = false;

        if (bListInstances.toString() != "NULL"){
            yInfo() << " Narration found for story: " << iTarget << ", size: " << bListInstances.size();
            target.humanNarration.clear();
            target.meaningStory.clear();


            int counter = atoi(bListInstances.get(0).asList()->get(0).toString().c_str());
            // for each element of narration found
            for (int ii = 0; ii < bListInstances.size(); ii++){
                //int itp = atoi(bListInstances.get(ii).asList()->get(0).asString().c_str());
                if (!stop){

                    Bottle bMessenger;
                    if (bListInstances.toString() != "NULL"){
                        for (int ll = 0; ll < bListInstances.size(); ll++){
                            if (atoi(bListInstances.get(ll).asList()->get(0).toString().c_str()) == counter){
                                bMessenger.addList() = (*bListInstances.get(ll).asList()).tail();
                            }
                        }
                    }
                    counter++;

                    string sMeaning = bMessenger.check("meaning", Value("none")).asString();
                    string sSentence = bMessenger.check("sentence", Value("none")).asString();
                    int iRank = atoi(bMessenger.check("rank", Value(0)).asString().c_str());
                    int iStory = atoi(bMessenger.check("story", Value(0)).asString().c_str());

                    // wrong order of element of narration
                    if (iRank != iCurrentElm || iStory != iTarget){
                        stop = true;
                    }
                    else{
                        target.humanNarration.push_back(sSentence);
                        target.meaningStory.push_back(sMeaning);
                    }
                    iCurrentElm = iRank + 1;
                }
            }
            //target.displayNarration();
        }
    }
    yInfo(" END SEARCH NARRATION");
}


// return the diff between two actions in seconds
double narrativeHandler::timeDiff(myTimeStruct tm1, myTimeStruct tm2, bool bPrint)
{
    //  struct tm diffTime;
    long int iYears,
        iMonth,
        iDays,
        iHours,     // number of hours of differences
        iMinutes,
        iSecond,
        iMilliSec;   // number of minutes of differences

    iYears = tm2.m_tm.tm_year - tm1.m_tm.tm_year;
    if (bPrint) cout << " iYears: " << iYears;
    iMonth = iYears * 12 + (tm2.m_tm.tm_mon - tm1.m_tm.tm_mon);
    if (bPrint) cout << " iMonth: " << iMonth;
    iDays = iMonth * 30 + (tm2.m_tm.tm_mday - tm1.m_tm.tm_mday);
    if (bPrint) cout << " iDays: " << iDays;
    iHours = iDays * 24 + (tm2.m_tm.tm_hour - tm1.m_tm.tm_hour);
    if (bPrint) cout << " iHours: " << iHours;
    iMinutes = iHours * 60 + (tm2.m_tm.tm_min - tm1.m_tm.tm_min);
    if (bPrint) cout << " iMinutes: " << iMinutes;
    iSecond = iMinutes * 60 + (tm2.m_tm.tm_sec - tm1.m_tm.tm_sec);
    if (iSecond > dThresholdDiffStory) return 10000.;
    if (bPrint) cout << " iSecond: " << iSecond;
    iMilliSec = iSecond * 1000 + (tm2.iMilliSec - tm1.iMilliSec);
    if (bPrint) cout << " iMilliSec: " << iMilliSec << endl;

    return (0.001*iMilliSec);
}

myTimeStruct  narrativeHandler::string2Time(string sTime)
{
    char *cBuffer;
    cBuffer = (char*)sTime.c_str();
    unsigned int i = 0;
    int iLevel = 0;
    //  int iHH,iMM,iSS; //iYear,iMonth,iDay,
    string sYear, sMonth, sDay, sHH, sMM, sMS, sSS = "";
    //  bool bYear,bMonth,bDay,bHH,bMM = false;
    while (cBuffer[i] != '\0')
    {
        char cTemp = cBuffer[i];
        if (cTemp == ' ' || cTemp == '-' || cTemp == ':' || cTemp == '+' || cTemp == '.')
        {
            iLevel++;
        }
        else if (cTemp != '"')
        {
            switch (iLevel)
            {
            case 0:
                sYear += cTemp;
                break;
            case 1:
                sMonth += cTemp;
                break;
            case 2:
                sDay += cTemp;
                break;
            case 3:
                sHH += cTemp;
                break;
            case 4:
                sMM += cTemp;
                break;
            case 5:
                sSS += cTemp;
                break;
            case 6:
                sMS += cTemp;
                break;
            }
        }
        i++;
    }
    struct tm tOutput;
    time_t myTime;
    time(&myTime);

    tOutput = *localtime(&myTime);
    tOutput.tm_hour = atoi(sHH.c_str());
    tOutput.tm_min = atoi(sMM.c_str());
    tOutput.tm_sec = atoi(sSS.c_str());
    tOutput.tm_year = atoi(sYear.c_str());
    tOutput.tm_mon = atoi(sMonth.c_str());
    tOutput.tm_mday = atoi(sDay.c_str());
    mktime(&tOutput);

    myTimeStruct mtsOut;
    mtsOut.m_tm = tOutput;
    mtsOut.iMilliSec = long(atoi(sMS.c_str()) * pow(10, 3 - sMS.size()));

    return mtsOut;
}


Bottle narrativeHandler::unfoldGoal(string goal)
{
    bool bVerbose = false;
    Bottle bOutput;

    if (bVerbose) cout << endl << "Starting to unfold: " << goal << endl;

    bool isRole = true;
    bool bIsFirst = true;
    istringstream iss(goal);
    Bottle bTemp;
    do
    {
        string sub;
        iss >> sub;
        if (sub[0] == '(') sub = sub.erase(0, 1);
        if (sub.size() != 0){
            if (sub[sub.size() - 1] == ')')   sub = sub.erase(sub.size() - 1);
            if (bVerbose) cout << "Substring: " << sub << endl;
            if (isRole){
                if (!bIsFirst) bOutput.addList() = bTemp;
                bTemp.clear();
                bTemp.addString(sub);
            }
            else{
                bTemp.addString(sub);
            }
            bIsFirst = false;
            isRole = !isRole;
        }

    } while (iss);

    if (bVerbose) cout << "bOutput: " << bOutput.toString() << endl;

    if (bVerbose) cout << "find is: " << bOutput.find("predicate").toString() << endl;

    return bOutput;
}


void narrativeHandler::initializeStories()
{
    yInfo() << " initializeStories from: " << cursorStories;

    vector<int>    toDelete; // vector of the stories to delete from the list.
    unsigned int iSto = 0;
    for (unsigned int jj = cursorStories; jj < listStories.size(); jj++){

        story &itSt = listStories[jj];
        itSt.vEvents.clear();
        ostringstream osRequest;
        osRequest.str("");
        osRequest << "SELECT time FROM main WHERE instance in ( " << *(itSt.viInstances.begin()) << " , " << itSt.viInstances[itSt.viInstances.size() - 1] << ") order by instance";
        Bottle bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
        itSt.timeBegin = string2Time(bMessenger.get(0).asList()->get(0).toString());
        itSt.timeEnd = string2Time(bMessenger.get(1).asList()->get(0).toString());

        ostringstream osRelation;
        ostringstream osMain;
        ostringstream osContentarg;

        osRelation << "SELECT instance, subject, verb, object FROM relation WHERE verb != 'isAtLoc' AND instance in (";
        osMain << "SELECT instance, activityname, activitytype, begin FROM main WHERE instance in (";
        osContentarg << "SELECT instance, argument, role, subtype FROM contentarg WHERE instance in (";


        bool bFirst = true;
        for (auto& itInst : itSt.viInstances){

            if (!bFirst) {
                osRelation << ", ";
                osMain << ", ";
                osContentarg << ", ";
            }
            osRelation << itInst;
            osMain << itInst;
            osContentarg << itInst;
            bFirst = false;
        }
        osRelation << ") ORDER BY instance";
        osMain << ") ORDER BY instance";
        osContentarg << ") ORDER BY instance";

        Bottle bAllRelation = iCub->getABMClient()->requestFromString(osRelation.str());
        Bottle bAllMain = iCub->getABMClient()->requestFromString(osMain.str());
        Bottle bAllContentA = iCub->getABMClient()->requestFromString(osContentarg.str());

        for (auto& itInst : itSt.viInstances){

            Bottle bTmpMain;
            if (bAllMain.toString() != "NULL"){
                for (int ll = 0; ll < bAllMain.size(); ll++){
                    if (atoi(bAllMain.get(ll).asList()->get(0).toString().c_str()) == itInst){
                        bTmpMain.addList() = (*bAllMain.get(ll).asList()).tail();
                    }
                }
            }

            Bottle bTmpArg;
            if (bAllContentA.toString() != "NULL"){
                for (int ll = 0; ll < bAllContentA.size(); ll++){
                    if (atoi(bAllContentA.get(ll).asList()->get(0).toString().c_str()) == itInst){
                        bTmpArg.addList() = (*bAllContentA.get(ll).asList()).tail();
                    }
                }
            }

            Bottle bTmpRel;
            if (bAllRelation.toString() != "NULL"){
                for (int ll = 0; ll < bAllRelation.size(); ll++){
                    if (atoi(bAllRelation.get(ll).asList()->get(0).toString().c_str()) == itInst){
                        // removing first element: instance
                        bTmpRel.addList() = (*bAllRelation.get(ll).asList()).tail();
                    }
                }
            }

            evtStory evtTemp;
            vector<string> tempOCW = initializeEVT(evtTemp, itInst, bTmpMain, bTmpArg, bTmpRel);

            itSt.vEvents.push_back(evtTemp);
            itSt.addOCW(tempOCW);
        }

        createNarration(itSt);
        if (itSt.sentenceStory.size() < 2){
            toDelete.push_back(iSto);
        }
        iSto++;
    }
    cout << endl;

    cout << "Found before deleting: " << listStories.size() << " stories;" << endl;

    yInfo() << "starting deletion of stories: " << toDelete.size();
    reverse(toDelete.begin(), toDelete.end());
    //for (auto del : toDelete){
    //    cout << "deleting: " << del;
    //    cout << " size: " << listStories[del].viInstances.size();
    //    cout << " events: " << listStories[del].vEvents.size();
    //    cout << " sentences: " << listStories[del].sentenceStory.size() << endl;
    //    for (auto sent : listStories[del].sentenceStory){
    //        cout << " \t" << sent << endl;
    //    }
    //    cout << "events: " << endl;
    //    for (auto ev : listStories[del].vEvents){
    //        ev.print();
    //    }
    //}


    for (auto del : toDelete)
    {
        listStories[del].toString();
        listStories.erase(listStories.begin() + del);
    }

    cursorStories = listStories.size();

    yInfo() << "End of initialisation of stories";
    yInfo() << listStories.size() << " stories found";
    cout << endl;

}


void narrativeHandler::updateScoreStory(story &st){

    st.mapScoreEvent.clear();

    unsigned int _size = st.vEvents.size();

    // create the map of each OCW with a vector of double for the score at each instance of the story
    for (auto& itS : st.vOCWEvt){
        vector<double> vTemp(_size);
        st.mapScoreEvent[itS] = vTemp;
    }

    // for each instance update the score of each OCW corresponding to a few rules
    //for (auto& itE : st.vEvents){
    //    // TODO
    //}
}


vector<string> narrativeHandler::initializeEVT(evtStory &evt, int _instance, Bottle bActivity, Bottle bArguments, Bottle _bRelations){
    evt.instance = _instance;
    evt.isNarration = false;
    evt.bRelations = _bRelations;
    vector<string>   vOCW;

    //    cout << endl;
    //    cout << "initialize: instance: " << _instance << endl << "bActivity: " << bActivity.toString() << endl
    //        << "bargu " << bArguments.toString() << endl
    //        << "brela " << _bRelations.toString() << endl;


    vector<string> vPredicate{ "predicate", "action", "action1", "action2", "action3", "verb", "verb1", "verb2", "verb3" };
    vector<string> vAgent{ "agent", "agent1", "agent2", "agent3" };
    vector<string> vObject{ "object", "object1", "object2", "object2" };
    vector<string> vRecipient{ "recipient", "spatial", "spatial1", "spatial2", "spatial3" };

    vector<string>  vNoPAOR{ "subsystem", "provider", "vector", "sentence" };

    evt.activity_type = (bActivity.get(0).asList())->get(1).toString();
    evt.activity_name = (bActivity.get(0).asList())->get(0).toString();

    if (evt.activity_type == ""){
        yWarning() << " in narrativeHandler::evtStory::evtStory no activity_type.";
    }
    if (evt.activity_name == ""){
        yWarning() << " in narrativeHandler::evtStory::evtStory no activity_type.";
    }

    evt.begin = (bActivity.get(0).asList())->get(2).toString() == "t";

    for (int kk = 0; kk < bArguments.size(); kk++){
        if (bArguments.get(kk).isList()) {
            Bottle bTemp = *bArguments.get(kk).asList();

            //if (bTemp.get(1).toString() == "status" || bTemp.get(2).toString() == "status"){
            //    yInfo() << "STATUS !!! STATUS !!!" << evt.instance << " " << bTemp.get(0).toString() << "-" << bTemp.get(1).toString() << "-" << bTemp.get(2).toString();
            //}

            if (isIn(vPredicate, bTemp.get(1).toString())) evt.predicate = bTemp.get(0).toString();
            else if (isIn(vPredicate, bTemp.get(2).toString())) evt.predicate = bTemp.get(0).toString();
            else if (isIn(vAgent, bTemp.get(1).toString())) evt.agent = bTemp.get(0).toString();
            //       else if (evt.isIn(vAgent, bTemp.get(2).toString())) evt.agent = bTemp.get(0).toString();
            else if (isIn(vObject, bTemp.get(1).toString()))    evt.object = bTemp.get(0).toString();
            //       else if (evt.isIn(vObject, bTemp.get(2).toString()))    evt.object = bTemp.get(0).toString();
            else if (isIn(vRecipient, bTemp.get(1).toString())) evt.recipient = bTemp.get(0).toString();
            else if (isIn(vRecipient, bTemp.get(2).toString())) evt.recipient = bTemp.get(0).toString();
            else {
                pair<string, string> ptemp(bTemp.get(1).toString(), bTemp.get(0).toString());
                evt.vArgument.push_back(ptemp);
            }

            if (!isIn(vNoPAOR, bTemp.get(1).toString())) vOCW.push_back(bTemp.get(0).asString());
            else if (!isIn(vNoPAOR, bTemp.get(2).toString())) vOCW.push_back(bTemp.get(0).asString());
        }
    }

    if (evt.activity_name == "production" || evt.activity_name == "comprehension" || evt.activity_name == "sentence"){
        evt.predicate = "say";
        for (int kk = 0; kk < bArguments.size(); kk++){
            if (bArguments.get(kk).isList()) {
                Bottle bTemp = *bArguments.get(kk).asList();
                if (bTemp.get(1).toString() == "speaker") evt.agent = bTemp.get(0).toString();
                else if (bTemp.get(1).toString() == "addressee") evt.recipient = bTemp.get(0).toString();
                else if (bTemp.get(1).toString() == "sentence") evt.object = bTemp.get(0).toString();
            }
        }

        string presentAgent = "partner";
        ostringstream osRequest;
        osRequest << "SELECT name FROM agent WHERE instance = " << _instance << " AND presence = true";
        Bottle bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
        if (bMessenger.toString() != "NULL"){
            if (bMessenger.size() != 0){
                for (int ll = 0; ll < bMessenger.size(); ll++){
                    if (bMessenger.get(ll).toString() != "partner"){
                        presentAgent = bMessenger.get(ll).toString();
                    }
                }
            }
        }

        if (evt.activity_type == "recog"){
            if (evt.agent == "" || evt.agent == "none" || evt.agent == "partner"){
                evt.agent = presentAgent;
            }
        }
        if (evt.activity_type == "say"){
            evt.agent = "iCub";
            if (evt.recipient == "" || evt.recipient == "none" || evt.recipient == "partner"){
                evt.recipient = presentAgent;
            }
        }

        evt.isNarration = (evt.agent == narrator);
    }

    if (evt.activity_type == "drives"){
        evt.object = evt.agent;
        evt.agent = "iCub";
        evt.predicate = "wants to";
    }

    if (_bRelations.toString() != "NULL"){
        for (int kk = 0; kk < _bRelations.size(); kk++){
            Bottle bTemp = (*_bRelations.get(kk).asList());
            for (int jj = 0; jj != bTemp.size(); jj++){
                vOCW.push_back(bTemp.get(jj).toString());
            }
        }
    }

    if (evt.activity_type == "action"){
        if (evt.predicate == "none" || evt.predicate == ""){
            evt.predicate = evt.activity_name;
            vOCW.push_back(evt.predicate);
        }
    }

    evt.addUnderscore();

    return vOCW;

}


void narrativeHandler::compareNarration(story &target){
    yInfo() << "BEGIN compareNarration: start to compare Narration from target: " << target.counter;
    for (auto& currentStory : listStories){
        comparator.clear();
        if (currentStory.counter != target.counter){ // not comparing the target to itself
            if (target.vEvents.size() <= currentStory.vEvents.size()){ //if there is not more in the target than in the current
                unsigned int K = 0; // possibility to pass narration
                bool stillOk = true;
                unsigned int cursor = 0;

                for (auto& tarEvt : target.vEvents){  // for each event of the target
                    K = cursor;
                    bool found = false;
                    if (stillOk){
                        for (unsigned int j = K; j < currentStory.vEvents.size(); j++){
                            evtStory evt = currentStory.vEvents[j];
                            if (!evt.isNarration && !found){

                                vector<string>  vOriginal, vCopy;
                                vOriginal.push_back(evt.agent);
                                vOriginal.push_back(evt.predicate);
                                vOriginal.push_back(evt.object);
                                vOriginal.push_back(evt.recipient);
                                vCopy.push_back(tarEvt.agent);
                                vCopy.push_back(tarEvt.predicate);
                                vCopy.push_back(tarEvt.object);
                                vCopy.push_back(tarEvt.recipient);

                                bool isEqual = checkListPAOR(vOriginal, vCopy);

                                if (isEqual){
                                    found = true;
                                    cursor = j + 1;
                                }
                            }
                        }
                        stillOk &= found;
                    }
                }
                if (stillOk){
                    yInfo("I found a matching story.");
                    if (currentStory.humanNarration.size() > 0){
                        yInfo(" adapting meaning !");
                        for (auto mean : currentStory.meaningStory){
                            target.meaningStory.push_back(adaptMeaning(mean));
                        }
                    }
                    yInfo(" adapted meanings are:");
                    target.humanNarration.clear();
                    for (auto meani : target.meaningStory){
                        cout << "\t" << meani << endl;
                        target.humanNarration.push_back(iCub->getLRH()->meaningToSentence(meani));
                    }
                }
            }
        }
    }

    target.displayNarration();
    yInfo() << "\nEND compareNarration";
}



void narrativeHandler::sayNarrationSimple(story target){

    cout << "Start Narration Simple : " << endl;
    cout << "size of human narration: " << target.humanNarration.size() << endl;

    if (target.humanNarration.size() > 2){
        iCub->opc->checkout();
        iCub->lookAtPartner();
        cout << endl << "********************************\nbegin saying from human: " << target.viInstances[0] << " with " << target.humanNarration.size() << " events and " << target.sentenceStory.size() << " sentences." << endl;
        iCub->say("I'll tell you what I did the other day.", true, false, "default", false);
        for (auto ii : target.humanNarration){
            iCub->opc->checkout();
            iCub->lookAtPartner();
            cout << "\t" << ii << endl;
            iCub->say(ii, true, false, "default", false);
        }
        cout << endl << "********************************" << endl;
        iCub->home();
        return;
    }
    else{
        if (target.sentenceStory.size() > iThresholdSentence){
            iCub->opc->checkout();
            cout << endl << "begin display narration of story: " << counter - 1 << " with " << target.vEvents.size() << " events and " << target.sentenceStory.size() << " sentence." << endl;
            for (auto itSt : target.sentenceStory){
                iCub->opc->checkout();
                iCub->lookAtPartner();
                iCub->say(itSt, true, false, "default", false);
            }
        }
        else{
            return;
        }
    }
}


void narrativeHandler::createNarration(story &sto)
{
    bool VERBOSE = false;

    vector<string>  vsOutput;

    ostringstream  osCurrent;

    tuple<string, string, string, string>  PAOR;

    //Bottle bPreviousRelation;

    osCurrent << "story: " << sto.timeBegin.toString() << " (" << sto.viInstances[0] << ")  to " << sto.timeEnd.toString() << " (" << sto.viInstances[sto.viInstances.size() - 1] << ")" << endl;


    vsOutput.push_back(osCurrent.str());
    int cursor = 0;
    for (unsigned int currentElement = 0; currentElement != sto.vEvents.size(); currentElement++){
        evtStory currentEvent = sto.vEvents[currentElement];
        currentEvent.addUnderscore();

        if (VERBOSE) cout << currentEvent.toString();

        bool addEvt = true;
        if (!currentEvent.isNarration)
        {
            osCurrent.str("");
            //osCurrent << currentEvent.instance;
            if (VERBOSE) cout << currentElement << "...";

            if (((currentEvent.predicate == "" || currentEvent.predicate == "none")
                && (currentEvent.activity_name == "" || currentEvent.activity_name == "none"))
                || (currentEvent.agent == "" || currentEvent.agent == "none"))
            {
                addEvt = false;
            }

            // initial situation
            if (cursor == 0){
                if (currentEvent.bRelations.toString() != "NULL"){
                    osCurrent << "\t\t\t" << "At the beginning, ";
                    for (int jj = 0; jj < currentEvent.bRelations.size(); jj++){
                        if (jj != 0){
                            osCurrent << " and ";
                        }
                        if (lrh && currentEvent.bRelations.get(jj).isList()){
                            string meaning = createMeaning(currentEvent.bRelations.get(jj).asList()->get(0).toString(),
                                currentEvent.bRelations.get(jj).asList()->get(1).toString(),
                                currentEvent.bRelations.get(jj).asList()->get(2).toString());
                            string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                            osCurrent << tmpSentence;
                        }
                        else{
                            osCurrent << currentEvent.bRelations.get(jj).toString();
                        }
                    }
                    if (osCurrent.str() != "") osCurrent << endl;
                }
            }
            // end initial situation

            if (currentEvent.predicate == "look"){
                //do nothing
                addEvt = false;
            }
            // if it is an ACTION
            else if (currentEvent.activity_type == "action"){

                // if the action begin
                if (currentEvent.agent == "iCub" || currentEvent.agent == "icub"){
                    currentEvent.agent = "iCub";
                }

                if (currentEvent.begin){
                    if (currentEvent.predicate == "babbling"){
                        osCurrent << "\t\t\tI moved a finger" << endl;
                    }
                    else if (cursor == 0){
                        if (lrh){
                            string meaning = createMeaning(currentEvent.agent,
                                currentEvent.predicate,
                                currentEvent.object,
                                currentEvent.recipient);
                            string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                            osCurrent << "\t\t\t" << tmpSentence;
                        }
                        else{
                            osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.predicate << " the " << currentEvent.object;
                        }
                        if (currentEvent.recipient != "none" && currentEvent.recipient != "")  osCurrent << " " << currentEvent.recipient;
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "adv1" || iarg->first == "adv2") osCurrent << " " << iarg->second;
                        }
                        if (osCurrent.str() != "") osCurrent << endl;
                    }
                    else{
                        // if the previous instance wasn't already an action
                        if (currentEvent.activity_type != sto.vEvents[currentElement - 1].activity_type || false == sto.vEvents[currentElement - 1].begin){
                            if (lrh){
                                string meaning = createMeaning(currentEvent.agent,
                                    currentEvent.predicate,
                                    currentEvent.object,
                                    currentEvent.recipient);
                                string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                                osCurrent << "\t\t\t" << tmpSentence;
                            }
                            else{
                                osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.predicate << " the " << currentEvent.object;
                            }
                            if (currentEvent.recipient != "none" && currentEvent.recipient != "")  osCurrent << " " << currentEvent.recipient;
                            for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                                if (iarg->first == "adv1" || iarg->first == "adv2") osCurrent << " " << iarg->second;
                            }
                            if (osCurrent.str() != "") osCurrent << endl;
                        }
                    }
                }
                else{ // the action ends
                    if (cursor == 0){
                        addEvt = false;
                    }
                    else{
                        if (VERBOSE) cout << endl;
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (VERBOSE) cout << iarg->first << " " << iarg->second << " - " << endl;
                        }
                        // if previous instance was not a beggining of action
                        if (sto.vEvents[currentElement - 1].begin || sto.vEvents[currentElement - 1].activity_type != "action"){
                            for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                                if (iarg->first == "status" && iarg->second == "failed"){
                                    osCurrent << "\t\t\t" << "But it failed.";
                                    addEvt = true;
                                }
                            }
                            if (osCurrent.str() != "") osCurrent << endl;
                        }
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "reason"){
                                osCurrent << "\t\t\t" << "Because " << iarg->second << "." << endl;
                            }
                        }
                    }
                }
            }
            else if (currentEvent.activity_name == "sentence"
                || currentEvent.activity_name == "comprehension"
                || currentEvent.activity_name == "production") {
                string speaker = currentEvent.agent,
                    addressee = currentEvent.recipient,
                    sentence = "none";
                for (auto& iarg : currentEvent.vArgument){
                    if (iarg.first == "speaker") speaker = iarg.second;
                    else if (iarg.first == "addressee")     addressee = iarg.second;
                    else if (iarg.first == "sentence")    sentence = iarg.second;
                }
                if (speaker == "none" && currentEvent.activity_type == "say"){
                    speaker = "iCub";
                }
                else{
                    if (speaker != "icub" && speaker != "iCub"){
                        if (addressee == "none"){
                            addressee = "iCub";
                        }
                        //                        speaker = "You";
                        //                        addressee = "me";
                    }
                }
                if (sentence == "none" || sentence == ""){
                    addEvt = false;
                }
                else{
                    addUnderscoreString(sentence);
                }
                //if (speaker == "iCub"){
                //    speaker = "I";
                //    addressee = "you";
                //   // addEvt = false;
                //}
                //else{
                //    speaker = "You";
                //    addressee = "me";

                //}
                if (lrh){
                    string meaning = createMeaning(speaker, "said", sentence, addressee);
                    string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                    removeUnderscoreString(tmpSentence);
                    osCurrent << "\t\t\t" << tmpSentence;
                }
                else{
                    osCurrent << "\t\t\t" << speaker << " said to " << addressee << ": " << sentence;
                }
                if (osCurrent.str() != "") osCurrent << endl;
            }
            // if not action or sentence
            else if (currentEvent.activity_type == "reasoning"){
                /* for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                osCurrent << iarg->second << " ";
                }*/
                // if the action begin
                if (currentEvent.begin){

                    // if the previous instance wasn't already an action
                    //if (current_activitytype != previous_activitytype || previous_begin != current_begin){
                    if (lrh){
                        string meaning = createMeaning(currentEvent.agent, "tries", currentEvent.activity_name);
                        string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                        osCurrent << "\t\t\t" << tmpSentence;
                    }
                    else{
                        osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.activity_name;
                    }
                    if (osCurrent.str() != "") osCurrent << endl;
                    for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                        if (iarg->first == "goal"){
                            Bottle bUnfolded = unfoldGoal(iarg->second);
                            osCurrent << "\t\t\t" << "The goal was that: ";
                            if (lrh){
                                string meaning = createMeaning(bUnfolded.find("agent").toString(),
                                    "try",
                                    bUnfolded.find("predicate").toString(),
                                    bUnfolded.find("object").toString());
                                string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                                osCurrent << tmpSentence;

                            }
                            else{
                                osCurrent << bUnfolded.find("agent").toString() << " " << bUnfolded.find("predicate").toString() << " the " << bUnfolded.find("object").toString();
                            }
                            if (bUnfolded.find("recipient").toString() != "")  osCurrent << " " << bUnfolded.find("recipient").toString();
                            if (osCurrent.str() != "") osCurrent << endl;
                        }
                    }
                }
                // the action ends
                else{
                    if (cursor == 0){
                        bool bStatusFound = false;
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "status" && iarg->second == "failed"){
                                osCurrent << "\t\t\t" << "But it failed";
                                bStatusFound = true;
                            }
                        }
                        if (!bStatusFound){
                            //                            osCurrent << "\t\t\t" << "And it worked." << endl;
                        }
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "reason"){
                                osCurrent << " because " << iarg->second;
                            }
                        }
                        osCurrent << "." << endl;
                    }
                    else if (sto.vEvents[currentElement - 1].begin || sto.vEvents[currentElement - 1].activity_type != "action"){// if previous instance was not a beggining of action
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "status" && iarg->second == "failed"){
                                osCurrent << "\t\t\t" << "But it failed." << endl;
                            }
                        }
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "reason"){
                                osCurrent << " because " << iarg->second << "." << endl;
                            }
                        }
                    }
                    else{
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "reason"){
                                osCurrent << " because " << iarg->second << "." << endl;
                            }
                        }
                    }
                }
            }
            // nor action/reasoning/sentence
            else {
                //for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                //    osCurrent << iarg->first << " " << iarg->second << "; ";
                //    //                osCurrent << bTemp.toString() << endl;
                //}
                //if (osCurrent.str() != "") osCurrent << endl;
                // if the action begin
                if (currentEvent.agent == "iCub" || currentEvent.agent == "icub"){
                    currentEvent.agent = "iCub";
                }

                if (currentEvent.begin){
                    bool hasPredicate = false;
                    if (currentEvent.predicate != "" && currentEvent.predicate != "none"){
                        hasPredicate = true;
                    }
                    if (currentEvent.predicate == "rename"){
                        currentEvent.predicate = "renamed";
                        if (currentEvent.object == "partner") currentEvent.object = "you";
                        if (currentEvent.object == "unknown_self") currentEvent.object = "this finger";
                        osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.predicate << " " << currentEvent.object << " into " << currentEvent.recipient << endl;
                    }
                    else if (currentEvent.predicate == "look"){
                        addEvt = false;
                    }
                    else if (currentEvent.predicate == "babbling"){
                        currentEvent.predicate = "moved";
                        if (currentEvent.object == "partner") currentEvent.object = "you";
                        osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.predicate << " a body part." << endl;
                    }
                    else if (currentEvent.predicate == "tagging"){
                        if (currentEvent.object == "unknown_object"){
                            currentEvent.predicate = "wanted to explore";
                            currentEvent.object = "an unknown object";
                        }
                        if (currentEvent.object == "unknown_self"){
                            currentEvent.predicate = "wanted to explore";
                            currentEvent.object = "an unknown body part";
                        }
                        osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.predicate << " " << currentEvent.object << endl;
                    }
                    else if (currentEvent.predicate == "wants_to"){
                        currentEvent.predicate = "wanted";
                        if (currentEvent.object == "partner") currentEvent.object = "you";
                        osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.predicate << " to ";
                        if (currentEvent.object == "tagging") currentEvent.object = "explore";
                        osCurrent << currentEvent.object << endl;
                    }
                    else if (currentEvent.predicate == "point"){
                        currentEvent.predicate = "pointed";
                        //if (currentEvent.agent != "I") currentEvent.agent = "You";
                        osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.predicate << " to it." << endl;
                    }
                    else if (cursor == 0){
                        if (lrh){
                            string meaning = createMeaning(currentEvent.agent,
                                "tries",
                                currentEvent.activity_name,
                                currentEvent.object);
                            string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                            osCurrent << "\t\t\t" << tmpSentence;
                        }
                        else{
                            if (hasPredicate){
                                osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.predicate;
                            }
                            else{
                                if (currentEvent.agent != "" && currentEvent.agent != "none"){
                                    osCurrent << "\t\t\t" << currentEvent.agent << " tries to " << currentEvent.activity_name;
                                }
                            }
                        }
                        if (currentEvent.object != "") osCurrent << " the " << currentEvent.object;
                        if (currentEvent.recipient != "") osCurrent << " the " << currentEvent.recipient;
                        if (osCurrent.str() != "") osCurrent << endl;
                    }
                    // if the previous instance wasn't already an action
                    else if (currentEvent.activity_type != sto.vEvents[currentElement - 1].activity_type || currentEvent.begin != sto.vEvents[currentElement - 1].begin){
                        if (lrh){
                            string meaning = createMeaning(currentEvent.agent,
                                "tries",
                                currentEvent.activity_name,
                                currentEvent.object);
                            string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                            osCurrent << "\t\t\t" << tmpSentence;
                        }
                        else{
                            if (hasPredicate){
                                osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.predicate;
                            }
                            else{
                                osCurrent << "\t\t\t" << currentEvent.agent << " tries to " << currentEvent.activity_name;
                            }
                        }
                        if (currentEvent.object != "") osCurrent << " the " << currentEvent.object;
                        if (currentEvent.recipient != "") osCurrent << " the " << currentEvent.recipient;
                        if (osCurrent.str() != "") osCurrent << endl;
                    }
                }
                // the action ends
                else {
                    if (cursor == 0){
                        bool bStatusFound = false;
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "status" && iarg->second == "failed"){
                                osCurrent << "\t\t\t" << "But it failed";
                                bStatusFound = true;
                            }
                        }
                        if (!bStatusFound){
                            //                            osCurrent << "\t\t\t" << "And it worked." << endl;
                        }
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "reason"){
                                osCurrent << " because " << iarg->second << "." << endl;
                            }
                        }
                    }
                    else if (sto.vEvents[currentElement - 1].begin || sto.vEvents[currentElement - 1].activity_type != "action"){// if previous instance was not a beggining of action
                        bool bStatusFound = false;
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "status" && iarg->second == "failed"){
                                osCurrent << "\t\t\t" << "But it failed";
                                bStatusFound = true;
                            }
                        }
                        if (!bStatusFound){
                            //                          osCurrent << "\t\t\t" << "And it worked." << endl;
                        }
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "reason"){
                                osCurrent << " because " << iarg->second << "." << endl;
                            }
                        }
                    }
                }
            }

            // changes in the relations:
            if (currentElement != sto.vEvents.size() && cursor != 0){

                if (currentEvent.bRelations != sto.vEvents[currentElement - 1].bRelations)
                {
                    if (currentEvent.bRelations.toString() != "NULL"){
                        osCurrent << "\t\t\t" << "And now, ";
                        for (int jj = 0; jj < currentEvent.bRelations.size(); jj++){
                            if (jj != 0){
                                osCurrent << " and ";
                            }
                            osCurrent << currentEvent.bRelations.get(jj).toString();
                        }
                        if (osCurrent.str() != "") osCurrent << endl;
                    }
                }
            }

            // final situation
            if (currentElement == sto.vEvents.size() - 1){
                if (currentEvent.bRelations.toString() != "NULL"){
                    osCurrent << "\t\t\t" << "In the end, ";
                    for (int jj = 0; jj < currentEvent.bRelations.size(); jj++){
                        if (jj != 0){
                            osCurrent << " and ";
                        }
                        if (lrh && currentEvent.bRelations.get(jj).isList()){
                            string meaning = createMeaning(currentEvent.bRelations.get(jj).asList()->get(0).toString(),
                                currentEvent.bRelations.get(jj).asList()->get(1).toString(),
                                currentEvent.bRelations.get(jj).asList()->get(2).toString());
                            string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                            osCurrent << tmpSentence;
                        }
                        else{
                            osCurrent << currentEvent.bRelations.get(jj).toString();
                        }
                    }
                    if (osCurrent.str() != "") osCurrent << endl;
                }
            }


            // Add only one appearance and one dissapearance per object
            if (currentEvent.predicate == "appear"){
                for (unsigned int prev = 0; prev < currentElement; prev++){
                    if (sto.vEvents[prev].predicate == "appear"
                        && sto.vEvents[prev].agent == currentEvent.agent){
                        addEvt = false;
                    }
                }
                addEvt = false;
            }
            if (currentEvent.predicate == "disappear"){
                for (unsigned int prev = 0; prev < currentElement; prev++){
                    if (sto.vEvents[prev].predicate == "disappear"
                        && sto.vEvents[prev].agent == currentEvent.agent){
                        addEvt = false;
                    }
                }
                addEvt = false;
            }
            addEvt &= osCurrent.str() != "";

            if (VERBOSE) cout << osCurrent.str();
            if (addEvt){
                string sentenceWithoutUnderscore = osCurrent.str();
                removeUnderscoreString(sentenceWithoutUnderscore);
                vsOutput.push_back(sentenceWithoutUnderscore);
            }
            cursor++;
        }
        // else is not narrration
        else {
            if (VERBOSE) cout << currentEvent.toString();
            for (auto arg : currentEvent.vArgument){
                if (arg.first == "meaning"){
                    sto.meaningStory.push_back(arg.second);
                }
            }
        }
    }

    if (VERBOSE) cout << endl << endl;
    sto.sentenceStory = vsOutput;
}


string narrativeHandler::createMeaning(string agent, string predicate, string object, string recipient){
    ostringstream osMeaning;
    bool bO = object != "";
    bool bR = recipient != "";

    // at least agent and predicate:
    osMeaning << ", " << predicate << " " << agent << " ";
    if (bO) osMeaning << object << " ";
    if (bR) osMeaning << recipient << " ";

    osMeaning << "<o> [_-_-_-_-_-_-_-_][A-P-";
    bO ? osMeaning << "O-" : osMeaning << "_-";
    bR ? osMeaning << "R-" : osMeaning << "_-";
    osMeaning << "_-_-_-_][_-_-_-_-_-_-_-_] <o>";
    return osMeaning.str();
}


// compare a list of PAOR to the ones that could be used in another story
bool narrativeHandler::checkListPAOR(vector<string> vOriginal, vector<string> vCopy){

    if (vOriginal.size() != vCopy.size()){
        //        yInfo(" Error in narrativeHandler::checkListPAOR - different sizes of input");
        return false;
    }

    for (unsigned int i = 0; i < vOriginal.size(); i++){
        bool found = false;
        for (auto &PAIR : comparator){
            if (!found){
                if (PAIR.first == vOriginal[i]){
                    found = true;
                    if (PAIR.second != vCopy[i]){
                        return false;
                    }
                }
            }
        }
        if (!found){
            pair<string, string>  pTmp(vOriginal[i], vCopy[i]);
            comparator.push_back(pTmp);
        }
    }

    return true;
}


string narrativeHandler::adaptMeaning(string meaning){
    //cout << "adapt   |   input: " << meaning;
    string str = meaning;
    for (unsigned int jj = 0; jj < comparator.size(); jj++){
        if (comparator[jj].first != ""){
            //            cout << " first: " << comparator[jj].first << " second: " << comparator[jj].second;
            size_t pos = 0;
            while ((pos = str.find(comparator[jj].first, pos)) != std::string::npos){
                str.replace(pos, comparator[jj].first.length(), comparator[jj].second);
                pos += comparator[jj].second.length();
            }
            //cout << "str is:" << str << endl;
        }
    }
    //cout << "   |  output: " << str << endl;
    return str;
}


bool narrativeHandler::tellingStoryFromMeaning(story target){

    yInfo() << " start telling story from meaning";
    vector<string>    tmpStory;
    for (auto mean : target.meaningStory){
        cout << "\t mean: " << mean << endl;
        tmpStory.push_back(iCub->getLRH()->meaningToSentence(mean));
    }
    yInfo() << " tmpStory size is: " << tmpStory.size();
    if (tmpStory.size() < 2)
    {
        return false;
    }


    for (auto sen : tmpStory){
        cout << "\t\t" << sen << endl;
    }
    return true;
}

bool narrativeHandler::narrate(int iIns){
    yInfo(" begin narrate");
    if (shouldSpeak)    iCub->say("Starting to narrate !", true, false, "default", false);
    // if (shouldSpeak)    iCub->say("But for now I don't want to speak !", true, false, "default", false);

    //    findStories(iMinInstance);

    //bool canNarrate = false;

    for (auto target : listStories){
        if (target.viInstances[0] == iIns){
            //target.displayNarration();
            sayNarrationSimple(target);
        }
    }

    //    narrationToSpeech(target);

    yInfo(" Narration finished.");

    return true;
}

bool narrativeHandler::addHumanNarration(int iStory, string sModality, int iScena){
    yInfo(" BEGIN addHumanNarration");

    yInfo() << " Parameters are: modality - " << sModality << " / iScena - " << iScena;

    findStories();
    bool found = false;

    if (iStory >= 0 && iStory < (int)listStories.size()) {
        story &target = listStories.at(iStory);
        found = true;
        target.displayNarration();

        if (sModality == "speech"){
            addNarrationToStory(target);
        }
        else if (sModality == "text"){
            addTextNarration(target);
        }
        else if (sModality == "auto"){
            addAutoNarration(target, iScena);
        }
        else{
            yWarning(" in narrativeHandler::addHumanNarration - unknown type of modality (should be speech - text - auto)");
        }
    }


    if (!found){
        yWarning(" in narrativeHandler::addHumanNarration - story not found.");
    }

    //yInfo() << " size of narration by human: " << listStories[listStories.size() - 1].meaningStory.size();
    //yInfo() << "telling:";
    //tellingStoryFromMeaning(listStories[listStories.size() - 1]);
    //yInfo("display");
    //listStories[listStories.size() - 1].displayNarration();

    yInfo(" END addHumanNarration");
    cout << endl;

    return true;
}



bool narrativeHandler::narrationToSpeech(story target){

    // look at agent;
    iCub->opc->checkout();
    //bool bFound = false;
    list<Entity*> entities = iCub->opc->EntitiesCache();

    cout << "narration to speech: " << target.humanNarration.size() << endl;

    //if (target.humanNarration.size() > 2){
    //    cout << "begin narration to speech from human:" << endl;
    //    for (auto ii : target.humanNarration){
    //        cout << "\t to speech: " << ii << endl;
    //        if (shouldSpeak){
    //            iCub->look(ag->name());
    //            iCub->say(ii, true, false, "default", false);
    //        }
    //    }
    //}
    if (target.humanNarration.size() > 2){
        cout << "begin narration to speech from human:" << endl;
        for (auto ii : target.humanNarration){
            cout << "\t to speech: " << ii << endl;
            if (shouldSpeak){
                iCub->lookAtPartner();
                iCub->say(ii, true, false, "default", false);
            }
        }
    }
    else{
        if (target.sentenceStory.size() > iThresholdSentence){
            cout << endl << "begin display narration to speech of story: " << counter << " with " << target.vEvents.size() << " events and " << target.sentenceStory.size() << " sentence." << endl;

            bool removeFirst = true;
            for (auto itSt : target.sentenceStory){
                cout << "\t to speech: " << itSt;
                if (shouldSpeak){
                    iCub->lookAtPartner();
                    if (!removeFirst) iCub->say(itSt, true, false, "default", false);
                }
                removeFirst = false;
            }
            cout << endl << endl;
        }
    }


    return true;
}

void narrativeHandler::addLink(const Bottle& command, Bottle& reply) {
    yInfo(" creating a link in the situation model");
    if (command.size() >= 4) {
        // Get the From sKeyMean
        storygraph::sKeyMean from;
        from.iIGARF = command.get(1).asList()->get(0).asInt();
        from.cPart = command.get(1).asList()->get(1).asString()[0];
        from.iRel = command.get(1).asList()->get(2).asInt();
        // Get the To sKeyMean
        storygraph::sKeyMean to;
        to.iIGARF = command.get(3).asList()->get(0).asInt();
        to.cPart = command.get(3).asList()->get(1).asString()[0];
        to.iRel = command.get(3).asList()->get(2).asInt();
        // Get meaning
        std::string m = command.get(2).asString();

        int last = sm.vDiscourseLinks.size() - 1;
        sm.createLink(from, to, m);
        for (int i = last; i < (int)sm.vDiscourseLinks.size(); i++) {
            const storygraph::sDiscourseLink& lk = sm.vDiscourseLinks.at(i);
            string link = lk.word + ": From " + lk.fromEvt.cPart + " of IGARF " + to_string(lk.fromEvt.iIGARF)
                + " to " + lk.toEvt.cPart + " of IGARF " + to_string(lk.toEvt.iIGARF);
            yInfo(link);
            reply.addString(link);
        }
        yInfo(" creation sucessful");
        reply.addString(" creation sucessful");
    }
    else {
        yInfo(" Not enough arguments");
        reply.addString("Error: Needs 3 arguments");
    }
}

void narrativeHandler::LRHtoSM(string m, Bottle& reply, bool create) {
    if (m != "") {
        if (m == ".") {
            sm.endSentence();
            yInfo(" (period)");
            reply.addString("(period)");
            return;
        }
        int last = sm.vDiscourseLinks.size();
        sm.LRHtoSM(m, create);
        // Display links created
        for (int i = last; i < (int)sm.vDiscourseLinks.size(); i++) {
            const storygraph::sDiscourseLink& lk = sm.vDiscourseLinks.at(i);
            string link = lk.word + ": From " + lk.fromEvt.cPart + " of IGARF " + to_string(lk.fromEvt.iIGARF)
                + " to " + lk.toEvt.cPart + " of IGARF " + to_string(lk.toEvt.iIGARF);
            yInfo(link);
            reply.addString(link);
        }
        if (last == (int)sm.vDiscourseLinks.size()) {
            yInfo(" no link");
            reply.addString(" no link");
        }
    }
    else {
        yInfo(" meaning is empty");
        reply.addString(" meaning is empty");
    }
}

void narrativeHandler::sentenceToTrain(string s, Bottle& reply) {
    if (s != "") {
        if (s == ".") {
            sm.endSentence();
            yInfo(" (period)");
            reply.addString("(period)");
            return;
        }
        string trainLine = sm.SMtoTrain(s);
        if (trainLine != "" && trainLine != "NoMatch") {
            yInfo(" " + trainLine);
            reply.addString(trainLine);
            if (fTrainOutput.is_open()) {
                fTrainOutput << trainLine << "\n";
                fTrainOutput.flush();
            }
        }
        else {
            yInfo(" No matching event found, can't create a meaning.");
            reply.addString(" no match found");
        }
    }
}


string narrativeHandler::lowerKey(string input){
    string tmp = input;
    transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
    return tmp;
}




void narrativeHandler::loadSM(int iScenario){
    if (iScenario < 0 || iScenario >= listStories.size()) {
        yWarning(" in NarrativeGraph::useDFW check instance scenario (out of range, sent to 0");
        iScenario = 0;
    }

    listStories.at(iScenario).displayNarration();

    ofstream IGARFfile;
    IGARFfile.open(sIGARFfile);
    sm.ABMtoSM(listStories.at(iScenario), IGARFfile);
    IGARFfile.close();
}



discourseform::meaningSentence narrativeHandler::meaningToEvent(string level1){

    ostringstream os;

    if (level1 != ""){

        // get the meaning:
        //cout << "level1 is: " << *level1 << endl;
        string delimiter = ",";
        size_t pos = 0;
        string token;
        string s = level1;
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
        stringstream stringStream(level1);
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

        discourseform::meaningSentence currentSentence;

        if (meaningPAOR.size() != meaningWords.size()){
            yWarning() << " in narrativeGraph::discourseform.cpp::meaningToDiscourseForm: Size of PAOR and OCW different";
            os << " in narrativeGraph::discourseform.cpp::meaningToDiscourseForm: Size of PAOR and OCW different" << endl;
            for (auto pa : meaningPAOR){
                cout << pa << " ";
                os << pa << " ";
            }
            cout << "\t";
            os << "\t";
            for (auto mea : meaningWords){
                cout << mea << " ";
                os << mea << " ";
            }
            cout << endl << "PAOR: " << meaningPAOR.size() << "\t OCW: " << meaningWords.size() << endl;
            os << endl << "PAOR: " << meaningPAOR.size() << "\t OCW: " << meaningWords.size() << endl;
            cout << "OCW are : ";
            os << "OCW are: ";
            for (auto oc : meaningWords){
                cout << oc << "\t";
                os << oc << "\t";
            }
            cout << endl;
        }
        else{
            for (int iWords = 0; iWords < meaningPAOR.size(); iWords++){
                int iNumberProposition = atoi(&(meaningPAOR[iWords].at(1)));

                // if new proposition
                if (currentSentence.vSentence.size() < iNumberProposition){
                    discourseform::meaningProposition tmp;

                    currentSentence.vSentence.push_back(tmp);
                }

                // add the OCW and PAOR
                currentSentence.vSentence[iNumberProposition - 1].vOCW.push_back(meaningWords[iWords]);
                currentSentence.vSentence[iNumberProposition - 1].vRole.push_back(&meaningPAOR[iWords].at(0));
            }
        }
        return currentSentence;
    }
}