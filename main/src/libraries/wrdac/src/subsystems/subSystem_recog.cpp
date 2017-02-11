#include <yarp/os/all.h>
#include "wrdac/subsystems/subSystem_recog.h"

bool wysiwyd::wrdac::SubSystem_Recog::connect() {
    // paste master name of
    ABMconnected = (SubABM->Connect());
    yInfo() << ((ABMconnected) ? "Recog connected to ABM" : "Recog didn't connect to ABM");

    if (!yarp::os::Network::isConnected(ears_port.getName(), "/ears/rpc")) {
        if (yarp::os::Network::connect(ears_port.getName(), "/ears/rpc")) {
            yInfo() << "Recog connected to ears";
        }
        else {
            yDebug() << "Recog didn't connect to ears at start";
        }
    }

    if (yarp::os::Network::isConnected(portRPC.getName(), "/speechRecognizer/rpc")){
        return true;
    }
    else {
        return yarp::os::Network::connect(portRPC.getName(), "/speechRecognizer/rpc");
    }
}

wysiwyd::wrdac::SubSystem_Recog::SubSystem_Recog(const std::string &masterName) : SubSystem(masterName){
    portRPC.open(("/" + m_masterName + "/recog:rpc").c_str());
    ears_port.open("/" + m_masterName + "/ears:o");
    m_type = SUBSYSTEM_RECOG;
    SubABM = new SubSystem_ABM(m_masterName + "/from_recog");
}

void wysiwyd::wrdac::SubSystem_Recog::Close() {
    portRPC.interrupt();
    portRPC.close();
    ears_port.interrupt();
    ears_port.close();
    SubABM->Close();

    delete SubABM;
}

bool wysiwyd::wrdac::SubSystem_Recog::setSpeakerName(std::string speaker)
{
    speakerName_ = speaker;
    yInfo() << " [subSystem_Recog] : speaker is now " << speakerName_;
    return true;
}

void wysiwyd::wrdac::SubSystem_Recog::listen(bool on) {
    if (!yarp::os::Network::isConnected(ears_port.getName(), "/ears/rpc")){
        yarp::os::Network::connect(ears_port.getName(), "/ears/rpc");
    }
    if (yarp::os::Network::isConnected(ears_port.getName(), "/ears/rpc")) {
        yarp::os::Bottle cmd, reply;
        cmd.addString("listen");
        if (on) {
            cmd.addString("on");
        }
        else {
            cmd.addString("off");
        }
        yDebug() << "[listen] Listen sending command" << cmd.toString();
        ears_port.write(cmd, reply);
        yDebug() << "[listen] Listen got reply" << reply.toString();
    }
    else {
        yWarning() << "[listen] No connection to ears available...";
    }
}

void wysiwyd::wrdac::SubSystem_Recog::waitForEars() {
    if (!yarp::os::Network::isConnected(ears_port.getName(), "/ears/rpc")){
        yarp::os::Network::connect(ears_port.getName(), "/ears/rpc");
    }
    if (yarp::os::Network::isConnected(ears_port.getName(), "/ears/rpc")) {
        yarp::os::Bottle cmd, reply;
        cmd.addString("listen");
        cmd.addString("offShouldWait");
        yDebug() << "[waitForEars] Listen sending command" << cmd.toString();
        ears_port.write(cmd, reply);
        yDebug() << "[waitForEars] Listen got reply" << reply.toString();
    }
    else {
        yWarning() << "[waitForEars] No connection to ears available...";
    }
}

bool wysiwyd::wrdac::SubSystem_Recog::interruptSpeechRecognizer() {
    yarp::os::Bottle bMessenger, bReply;
    bMessenger.addString("interrupt");
    // send the message
    portRPC.write(bMessenger, bReply);
    if(bReply.get(1).asString() != "OK") {
        yError() << "speechRecognizer was not interrupted";
        yDebug() << "Reply from speechRecognizer:" << bReply.toString();
        return false;
    } else {
        return true;
    }
}

yarp::os::Bottle wysiwyd::wrdac::SubSystem_Recog::recogFromGrammar(std::string &sInput)
{
    if (!yarp::os::Network::isConnected(portRPC.getName(), "/speechRecognizer/rpc")){
        if (!yarp::os::Network::connect(portRPC.getName(), "/speechRecognizer/rpc")){
            yarp::os::Bottle bReply;
            bReply.addInt(0);
            bReply.addString("recog not connected");
            yWarning(" recog not connected");
            return bReply;
        }
    }
    // turn on the main grammar through ears
    listen(false);

    yarp::os::Bottle bMessenger;
    yarp::os::Bottle bReply;
    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(sInput);
    portRPC.write(bMessenger, bReply);

    listen(true);

    return bReply;
    // turn off the main grammar through ears
}

yarp::os::Bottle wysiwyd::wrdac::SubSystem_Recog::recogFromGrammarLoop(std::string sInput, int iLoop, bool isEars, bool forwardABM)
{
    if (!yarp::os::Network::isConnected(portRPC.getName(), "/speechRecognizer/rpc")){
        if (!yarp::os::Network::connect(portRPC.getName(), "/speechRecognizer/rpc")){
            yarp::os::Bottle bReply;
            bReply.addInt(0);
            bReply.addString("recog not connected");
            yWarning(" recog not connected");
            return bReply;
        }
    }
    std::ostringstream osError;
    bool fGetaReply = false;
    yarp::os::Bottle bMessenger, //to be send TO speech recog
            bAnswer,
            bReply, //response from speech recog without transfer information, including raw sentence
            bOutput; // semantic information of the content of the recognition

    if (!isEars) {
        listen(false);

        interruptSpeechRecognizer();

        waitForEars();
    }

    bMessenger.clear();
    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(sInput);

    int loop;
    (iLoop == -1) ? loop = -3 : loop = 0;

    // listen off

    while (!fGetaReply && loop < iLoop)
    {
        // send the message
        portRPC.write(bMessenger, bReply);

        yInfo() << " Reply from Speech Recog : " << bReply.toString();

        if (bReply.toString() == "NACK" || bReply.size() < 2)
        {
            bOutput.addInt(0);
            osError << "Check grammar";
            bOutput.addString(osError.str());
            yError() << " " << osError.str();
            if (!isEars) listen(true);
            return bOutput;
        }
        else if (bReply.get(0).toString() == "0")
        {
            bOutput.addInt(0);
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            yInfo() << " " << osError.str();
            if (!isEars) listen(true);
            return bOutput;
        }
        else if (bReply.get(0).toString() == "ACK")
        {
            if (bReply.get(1).toString() == "-1")
            {
                yError() << "Only found Garbage...";
                yError() << "Check Why this happens";
            }
            else
            {
                yInfo() << "Sentence Acknowledged";
                bAnswer = *bReply.get(1).asList();

                if (bAnswer.toString() != "" && !bAnswer.isNull())
                {
                    fGetaReply = true;
                    bOutput.addInt(1);
                    bOutput.addList() = bAnswer;

                    // send the result of recognition to the ABM
                    if (ABMconnected && forwardABM)
                    {
                        std::list<std::pair<std::string, std::string> > lArgument;
                        lArgument.push_back(std::pair<std::string, std::string>(bAnswer.get(0).toString(), "sentence"));
                        if(bAnswer.size() > 1) {
                            lArgument.push_back(std::pair<std::string, std::string>(bAnswer.get(1).toString(), "semantic"));
                        }
                        lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                        //add speaker name. name should be sent through fonction before
                        if (speakerName_.empty()){
                            speakerName_ = "partner";
                            yWarning() << " [subSystem_Recog] " << "name of the speaker has been assigned to the default value : " << speakerName_;
                        }
                        lArgument.push_back(std::pair<std::string, std::string>(speakerName_, "speaker"));
                        SubABM->sendActivity("action",
                                             "sentence",
                                             "recog",
                                             lArgument,
                                             true);
                    }
                }
            }

        }
        if (iLoop != -1)
            loop++;
    }

    if (!fGetaReply)
    {
        bOutput.addInt(0);
        osError << "no vocal input";
        bOutput.addString(osError.str());
        yDebug() << osError.str();
    }
    if (!isEars) listen(true);
    return bOutput;
}
