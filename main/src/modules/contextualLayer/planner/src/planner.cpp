#include "planner.h"
#include "wrdac/subsystems/subSystem_recog.h"

bool Planner::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("planner")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << moduleName << " : finding configuration files...";
    period = rf.check("period", Value(0.1)).asDouble();

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "planner", "client.ini", isRFVerbose);
    iCub->opc->isVerbose = false;
    if (!iCub->connect())
    {
       yInfo() << " iCubClient : Some dependencies are not running...";
       Time::delay(1.0);
    }
    
    grpPlans = rf.findGroup("PLANS");
    avaiPlansList = *grpPlans.find("plans").asList();

    bool ears = 1;
    bool BM = 1;
    bool homeo = 1;
    bool SM = 1;

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    if (ears)
    {
        while (!Network::connect("/ears/target:o",rpc.getName())) {
            yWarning() << "ears is unreachable";
            yarp::os::Time::delay(0.8);
        }
    }

    if (BM)
    {
        portToBehavior.open("/" + moduleName + "/behavior/cmd:o");
        while (!Network::connect(portToBehavior.getName(),"/BehaviorManager/trigger:i")) {
            yWarning() << "BehaviorManager is unreachable";
            yarp::os::Time::delay(0.8);
        }
        yDebug()<<"Connected to BM!";
    }

    if (homeo)
    {
        toHomeo.open("/manager/toHomeostasis/rpc:o");
        while (!Network::connect(toHomeo.getName(),"/homeostasis/rpc")) {
            yWarning() << "homeostasis is unreachable";
            yarp::os::Time::delay(0.8);
        }
    }

    if (SM)
    {
        getState.open("/planner/state:i");
        while (!Network::connect(getState.getName(), "/SensationManager/rpc")) {
            yWarning() << "sensationManager is unreachable.";
            yarp::os::Time::delay(0.8);
        }
    }

    actPt = action_list.begin();
    prioPt = priority_list.begin();

    priority_list.clear();
    action_list.clear();
    id=0;
    fulfill=0;
    attemptCnt = 0;

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";
    
    return true;
}


bool Planner::close() {
    if(iCub) {
        iCub->close();
        delete iCub;
    }

    portToBehavior.interrupt();
    portToBehavior.close();

    port_behavior_context.interrupt();
    port_behavior_context.close();

    rpc.interrupt();
    rpc.close();

    return true;
}

bool Planner::freeze_all()
{
    // Prepare command
    gandalf.clear();
    gandalf.addString("freeze");
    gandalf.addString("all");
    // Send command
    if (!Network::isConnected(toHomeo.getName(),"/homeostasis/rpc"))
    {
       yInfo() << Network::connect(toHomeo.getName(),"/homeostasis/rpc");
       yarp::os::Time::delay(0.1);
    }
    toHomeo.write(gandalf);
    yInfo() << "Gandalf has spoken.";

    return true;
}

bool Planner::unfreeze_all()
{
    // Prepare command
    gandalf.clear();
    gandalf.addString("unfreeze");
    gandalf.addString("all");
    // Send command
    if (!Network::isConnected(toHomeo.getName(),"/homeostasis/rpc"))
    {
        yInfo() << Network::connect(toHomeo.getName(),"/homeostasis/rpc");
        yarp::os::Time::delay(0.1);
    }
    toHomeo.write(gandalf);
    yInfo() << "Gandalf has rescinded.";

    return true;
}

bool Planner::checkKnown(const Bottle& command, Bottle& avaiPlansList, string foundPlan) {
    // check if plan exists in ini file
    for (int i = 0; i < avaiPlansList.size(); i++)
        {
            // iterate through list of drives to check if action plan is known
            if (avaiPlansList.get(i).asString() == command.get(1).asList()->get(0).asString())
            {
                foundPlan = avaiPlansList.get(i).asString();
                yInfo() << "plan is known: " + foundPlan;
                return true;
            }
        }

    return false;
}

bool Planner::respond(const Bottle& command, Bottle& reply) {
    LockGuard lg(mutex);
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "quit \n" +
        "help \n" +
        "follow \n" +
        "new (plan priority (objectType object)) \n" +
        "freeze \n" +
        "unfreeze \n" +
        "actions \n" +
        "listplans \n" +
        "stopfollow \n" +
        "close \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        yInfo() << "quitting";
        return false;
    }else if (command.get(0).asString() == "help") {
        yInfo() << helpMessage;
        reply.addString(helpMessage);
    }
    else if (command.get(0).asString() == "listplans") {
        yInfo() << "Listing available plans";
        cout << avaiPlansList.toString() << "\n";
        reply.addString("ack");
    }
    else if (command.get(0).asString() == "stopfollow") {
        fulfill = 0;
        reply.addString("ack");
    }
    else if ((command.get(0).asString() == "new")&& (command.get(1).asList()->size() != 2)){
        // rpc command was of type "new (plan priority (objectType object))"
        ordering = true;
        newPlan.push_back(command);
        reply.addString("ack");
    }
    else if ((command.get(0).asString() == "new") && (command.get(1).asList()->size() == 2)){
        // rpc command was of type "new (plan (objectType object))"
        ordering = true;
        Bottle ncmd;
        Bottle ctxt;
        Bottle deet;
        ctxt.addString(command.get(1).asList()->get(1).asList()->get(0).asString());
        ctxt.addString(command.get(1).asList()->get(1).asList()->get(2).asString());
        deet.addString(command.get(1).asList()->get(0).asString());
        deet.addString("1");
        deet.addList()=ctxt;
        ncmd.addString("new");
        ncmd.addList()=deet;
        newPlan.push_back(ncmd);
        reply.addString("ack");

    }
        // (To-Do) Check goal not in list
    else if (command.get(0).asString() == "close"){
        yInfo() << "closing module planner...";
        reply.addString("ack");
        close();
    }
    else if (command.get(0).asString() == "priorities")
    {
        for (vector<int>::const_iterator i = priority_list.begin(); i != priority_list.end(); ++i)
        {
            cout << *i << '\n';
        }
        reply.addString("ack");
    }
    else if (command.get(0).asString() == "actions")
    {
        for (vector<string>::const_iterator i = action_list.begin(); i != action_list.end(); ++i)
        {
            cout << *i << '\n';
        }
        reply.addString("ack");
    }
    else if (command.get(0).asString() == "freeze")
    {
        freeze_all();
        reply.addString("ack");
    }
    else if (command.get(0).asString() == "unfreeze")
    {
        unfreeze_all();
        reply.addString("ack");
    }
    else if(command.get(0).asString() == "follow"){ //Toggle follow goals o goals accomplished
        if (command.size() == 1) { fulfill = 1; yInfo() << "fulfill " << fulfill; }
        else if (command.size()>1) { fulfill = 1; yInfo() << "fulfill " << fulfill; }
        else{
            reply.addString("nack");
            yInfo()<<"rpc command not valid.";
            return false;
        }
        if (fulfill)
            yInfo() << "I need to fulfill my goals " ;
        else{
            yInfo() << "I don't need to fulfill my goals anymore " ;
        }
        reply.addString("ack");
    }
    else {
        yInfo() << helpMessage;
        reply.addString("wrong command");
    }

    return true;
}

/* Called periodically every getPeriod() seconds */
bool Planner::updateModule() {

    if (ordering)
    {
        for (unsigned int it = 0; it < newPlan.size(); it++)
        {
            bool knownPlan = false;
            string planExe;
            int priority;
            Bottle command = newPlan[it];
            knownPlan = checkKnown(command, avaiPlansList, planExe);
            string planName = command.get(1).asList()->get(0).asString();
            objectType = command.get(1).asList()->get(2).asList()->get(0).asString();
            object = command.get(1).asList()->get(2).asList()->get(1).asString();

            if (knownPlan)
            {
                yInfo() << "plan known.";
                // determine which parts of plan needs to be executed depending on state of object
                int stepID;
                // assumption is that the action plan is complete enough that at least one set of prerequisites is met
                bool assumption = false;
                Bottle bot;
                Bottle rep;
                bool state;

                for (int ii = grpPlans.find(planName + "-totactions").asInt(); ii > 0; ii--)
                {
                    string actionName = grpPlans.find(planName + "action" + to_string(ii)).asString();
                    // checking if preconditions are met starting from last action
                    state = true;
                    Bottle preconds = *grpPlans.find(planName + "-" + to_string(ii) + "pre").asList();
                    std::list<std::pair<std::string, std::string >> lArgument;
                    for (int i = 0; i < preconds.size(); i++)
                    {
                        Bottle* msg = preconds.get(i).asList()->get(1).asList();
                        getState.write(*preconds.get(i).asList()->get(1).asList(), rep);
                        bool indiv;
                        string attach = preconds.get(i).asList()->get(0).asString();
                        if (attach == "not")
                        {
                            indiv = !rep.get(1).asInt();
                        }
                        else
                        {
                            indiv = rep.get(1).asInt();
                        }
                        state = state && indiv;

                        // formulate step for recording in ABM
                        string strind;
                        if (indiv == true) { strind = "true"; }
                        else { strind = "false"; }
                        lArgument.push_back(std::pair<std::string, std::string>(msg->get(0).asString(), "predicate"+to_string(i)));
                        lArgument.push_back(std::pair<std::string, std::string>(msg->get(1).asString(), "agent"+to_string(i)));
                        lArgument.push_back(std::pair<std::string, std::string>(msg->get(2).asString(), "object"+to_string(i)));
                        lArgument.push_back(std::pair<std::string, std::string>(strind, "result"+to_string(i)));
                    }

                    // record action selection reasoning in ABM
                    iCub->getABMClient()->sendActivity("reasoning",
                        actionName,
                        "planner",  // expl: "pasar", "drives"...
                        lArgument,
                        true);
                    yInfo() << actionName + " reasoning has been recorded in the ABM";

                    if (state)
                    {
                        stepID = ii;
                        assumption = true;
                        break;
                    }
                }

                if (assumption)
                {
                    // check if necessary to rank actions according to priority
                    int insertID = 0;
                    bool rankPriority = true;
                    priority = command.get(1).asList()->get(1).asInt();
                    if (priority_list.size() != 0)
                    {
                        for (unsigned int i = 0; i < priority_list.size(); i++)
                        {
                            if (priority == 1)
                            {
                                yInfo() << "lowest priority - append to bottom of list";
                                rankPriority = false;
                                break;
                            }
                            else if (priority < priority_list[i])
                            {
                                insertID = i + 1;
                            }
                        }
                    }
                    else
                    {
                        yInfo() << "new plan when initial state is empty";
                        rankPriority = false;
                    }

                    // insert actions into the various lists
                    if (!rankPriority)
                    {
                        for (int i = stepID; i < grpPlans.find(planName + "-totactions").asInt() + 1; i++)
                        {
                            string act = grpPlans.find(planName + "-action" + to_string(i)).asString();
                            yInfo() << "adding action " << act;
                            action_list.push_back(act);
                            priority_list.push_back(priority);
                            plan_list.push_back(planName);
                            object_list.push_back(object);
                            type_list.push_back(objectType);
                            actionPos_list.push_back(i);

                            // log in ABM
                            std::list<std::pair<std::string, std::string> > lArgument;
                            lArgument.push_back(std::pair<std::string, std::string>("add_action", "predicate"));
                            lArgument.push_back(std::pair<std::string, std::string>("priority", "agent"));
                            lArgument.push_back(std::pair<std::string, std::string>(act, "object"));
                            lArgument.push_back(std::pair<std::string, std::string>("bottom", "result"));
                            iCub->getABMClient()->sendActivity("reasoning",
                                "priority",
                                "planner",  // expl: "pasar", "drives"...
                                lArgument,
                                true);
                            yInfo() << "addition of " + act + " to bottom of list has been recorded in the ABM";
                        }

                    }
                    else
                    {   vector<string>::iterator idx = action_list.begin() + insertID;
                        vector<int>::iterator indx = priority_list.begin() + insertID;
                        for (int i = grpPlans.find(planName + "-totactions").asInt(); i > 0; i--)
                        {
                            string act = grpPlans.find(planName + "-action" + to_string(i)).asString();
                            yInfo() << "adding action " << act;
                            idx = plan_list.insert(idx, planName);
                            idx = object_list.insert(idx, object);
                            idx = type_list.insert(idx, objectType);
                            idx = action_list.insert(idx, act);
                            indx = actionPos_list.insert(indx, i);
                            indx = priority_list.insert(indx, priority);

                            // log in ABM
                            std::list<std::pair<std::string, std::string> > lArgument;
                            lArgument.push_back(std::pair<std::string, std::string>("add_action", "predicate"));
                            lArgument.push_back(std::pair<std::string, std::string>("priority", "agent"));
                            lArgument.push_back(std::pair<std::string, std::string>(act, "object"));
                            lArgument.push_back(std::pair<std::string, std::string>(to_string(insertID), "result"));
                            iCub->getABMClient()->sendActivity("reasoning",
                                "priority",
                                "planner",  // expl: "pasar", "drives"...
                                lArgument,
                                true);
                            yInfo() << "addition of " + act + " to index " << insertID << " of list has been recorded in the ABM";
                        }
                    }

                }
                else
                {
                    yWarning() << "None of the sets of prerequisites are met, unable to handle plan.";
                    yarp::os::Time::delay(1.5);
                }
            }
            else
            {
                yInfo() << "Plan " + command.get(1).asList()->get(0).asString() + " is not known. Use listplans to see available plans.";
            }
        }

        newPlan.clear();
        ordering = false;
    }

    //Check need to fulfill goals
    if (fulfill)
    {
        Value val;  //Likely whould be global and not local
        int skip = 0;

        if (action_list.size() != 0)
        {
            // yInfo() << "executing "<<action_list<<"...";
            yInfo() << "putting homeostasis on hold.";
            freeze_all();
        }

        else
        {
            fulfill = 0;
            yDebug() << "no actions in list, fulfill is set to 0.";
            skip = 1;
        }
        
        if (!skip)
        {
            // execute action
            bool actionCompleted = false;

            // request for BM to execute an action, waits for reply
            string act = action_list[0];
            yInfo()<<"Sending action " + act + " to the BM.";
            actionCompleted = triggerBehavior(Bottle(act));
            yInfo() << "action has been completed: " << actionCompleted;

            // record list of action in the ABM
            std::list<std::pair<std::string, std::string >> lArgument;
            for (unsigned int i = 0; i < action_list.size(); i++)
            {
                lArgument.push_back(std::pair<std::string, std::string>(action_list[i], "action"));
            }
            iCub->getABMClient()->sendActivity("reasoning",
                "action_list",
                "planner",  // expl: "pasar", "drives"...
                lArgument,
                true);
            yInfo() << "sent action_list to ABM";
            lArgument.clear();

            // check for completed state
            string planName = plan_list[0];
            Bottle stateOI = *grpPlans.find(planName + "-" + to_string(actionPos_list[0]) + "post").asList();

            // checking for post condition fulfillment.
            int stateCheck = 1;
            for (int i = 0; i < stateOI.size(); i++)
            {
                Bottle bot;
                Bottle rep;
                bot.clear();
                rep.clear();
                bot = *stateOI.get(i).asList()->get(1).asList();
                getState.write(bot, rep);
                int indiv;

                if (stateOI.get(i).asList()->get(0).asString() == "not")
                {
                    indiv = !rep.get(1).asInt();
                }
                else
                {
                    indiv = rep.get(1).asInt();
                }

                stateCheck = indiv && stateCheck;
            }

            if (actionCompleted && stateCheck)
            {
                yInfo() << "removing action " << *action_list.begin();
                action_list.erase(action_list.begin());
                priority_list.erase(priority_list.begin());
                plan_list.erase(plan_list.begin());
                object_list.erase(object_list.begin());
                type_list.erase(type_list.begin());
                actionPos_list.erase(actionPos_list.begin());
                attemptCnt = 0;

                yInfo() << "action completed and removed from lists.";
            }
            else
            {
                attemptCnt += 1;

                // log in ABM
                std::list<std::pair<std::string, std::string> > lArgument;
                lArgument.push_back(std::pair<std::string, std::string>("keep_action", "predicate"));
                lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
                lArgument.push_back(std::pair<std::string, std::string>(action_list[0], "object"));
                iCub->getABMClient()->sendActivity("reasoning",
                    "iCub",
                    "planner",  // expl: "pasar", "drives"...
                    lArgument,
                    true);
                yInfo() << action_list[0] + " failure and reattempt has been recorded in the ABM";

                if (attemptCnt > 2)
                {
                    yInfo() << "reached threshold for action attempts.";
                    iCub->say("I have tried too many times and failed to do " + action_list[0] + ". Do it yourself or help me.");
                    // remove action and/or plan?
                }
                // wait for a second before trying again
                Time::delay(1.0);
            }

            // check again if all actions in the list have been completed
            if(action_list.size() == 0)
            {
                fulfill = 0;
                yInfo() << "resuming homeostatic dynamics.";
                unfreeze_all();
            }
        }
    }
    else{
        yDebug() << "I need to fulfill plans, but I have none in my list...";
    }

    return true;
}


bool Planner::triggerBehavior(Bottle cmd)
{
    yDebug()<<"sending behavior...";
    
    Bottle reply;
    reply.clear();
    portToBehavior.write(cmd,reply);
    if (reply.get(0).asString() == "ack")
    {
        return true;
    }
    else
    {
        return false;
    }
}