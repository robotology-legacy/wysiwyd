#include "planner.h"
#include "wrdac/subsystems/subSystem_recog.h"

bool Planner::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("planner")).asString().c_str();
    setName(moduleName.c_str());

    manual = false;

    yInfo() << moduleName << " : finding configuration files...";
    period = rf.check("period", Value(0.1)).asDouble();
    
    grpPlans = rf.findGroup("PLANS");
    avaiPlansList = *grpPlans.find("plans").asList();

    fulfill = rf.check("fulfill",Value("true")).asBool();

    bool ears = rf.check("ears",Value("true")).asBool();
    bool homeo = rf.check("homeostasis",Value("true")).asBool();
    useABM = rf.check("ABM",Value("true")).asBool();
    bool SM = 1;
    bool BM = 1;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "planner", "client.ini", isRFVerbose);
    iCub->opc->isVerbose = false;
    while (!iCub->connect())
    {
       yInfo() << " iCubClient : Some dependencies are not running...";
       Time::delay(1.0);
    }

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    portToBehavior.open("/" + moduleName + "/behavior/cmd:o");
    toHomeo.open("/planner/toHomeostasis/rpc:o");
    getState.open("/planner/state:i");

    if (ears)
    {
        while (!Network::connect("/ears/target:o",rpc.getName())) {
            yWarning() << "ears is unreachable";
            yarp::os::Time::delay(0.8);
        }
    }

    if (BM)
    {
        while (!Network::connect(portToBehavior.getName(),"/BehaviorManager/trigger:i")) {
            yWarning() << "BehaviorManager is unreachable";
            yarp::os::Time::delay(0.8);
        }
        yDebug()<<"Connected to BM!";
    }

    if (homeo)
    {
        while (!Network::connect(toHomeo.getName(),"/homeostasis/rpc")) {
            yWarning() << "homeostasis is unreachable";
            yarp::os::Time::delay(0.8);
        }
    }

    if (SM)
    {
        while (!Network::connect(getState.getName(), "/SensationManager/rpc")) {
            yWarning() << "sensationManager is unreachable.";
            yarp::os::Time::delay(0.8);
        }
    }

    priority_list.clear();
    action_list.clear();
    id = 0;
    attemptCnt = 0;
    planNr = 0;

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";
    
    return true;
}


bool Planner::exit() {
    if(iCub) {
        iCub->close();
        delete iCub;
    }

    portToBehavior.interrupt();
    portToBehavior.close();

    toHomeo.interrupt();
    toHomeo.close();

    getState.interrupt();
    getState.close();

    // currently not in use, will be implemented when BM receives context for actions as well
    // port_behavior_context.interrupt();
    // port_behavior_context.close();

    rpc.interrupt();
    rpc.close();

    return true;
}

bool Planner::freeze_all()
{
    if(!manual) {
        Bottle cmd;
        // Prepare command
        cmd.clear();
        cmd.addString("freeze");
        cmd.addString("all");

        // Send command
        toHomeo.write(cmd);
    }

    return true;
}

bool Planner::unfreeze_all()
{
    if(!manual) {
        Bottle cmd;
        // Prepare command
        cmd.clear();
        cmd.addString("unfreeze");
        cmd.addString("all");
        // Send command
        toHomeo.write(cmd);
    }

    return true;
}

bool Planner::checkKnown(const Bottle& command, Bottle& avaiPlansList) {
    // check if plan exists in ini file
    for (int i = 0; i < avaiPlansList.size(); i++)
    {
            // iterate through list of drives to check if action plan is known
        if (avaiPlansList.get(i).asString() == command.get(1).asList()->get(0).asString())
        {
            yInfo() << "plan is known: " + avaiPlansList.get(i).asString();
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
    "actionPos \n" +
    "listplans \n" +
    "stopfollow \n" +
    "manual \n"
    "exit \n";

    reply.clear();

    yDebug() << "Command received: " << command.toString();

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
        reply.addList().append(avaiPlansList);
    }
    else if (command.get(0).asString() == "stopfollow") {
        fulfill = false;
        yInfo() << "fulfill has been set to false.";
        reply.addString("ack");
    }
    else if ((command.get(0).asString() == "new")&& (command.get(1).asList()->size() != 2)){
        // rpc command was of type "new (plan priority (objectType object))"
        // newPlan.push_back(command);
        newPlan = orderPlans(newPlan, command);
        reply.addString("ack");
    }
    else if ((command.get(0).asString() == "new") && (command.get(1).asList()->size() == 2)){
        // rpc command was of type "new (plan (objectType object))"
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
        yDebug() << "adding to newPlan: " << ncmd.toString();
        // newPlan.push_back(ncmd);
        newPlan = orderPlans(newPlan, ncmd);
        reply.addString("ack");
    }
    // (To-Do) Check goal not in list
    else if (command.get(0).asString() == "exit"){
        yInfo() << "closing module planner...";
        reply.addString("ack");
        exit();
    }
    else if (command.get(0).asString() == "priorities")
    {
        if (priority_list.size() == 0) { yInfo() << "priority list is empty."; }
        for (vector<int>::const_iterator i = priority_list.begin(); i != priority_list.end(); ++i)
        {
            cout << *i << '\n';
        }
        reply.addString("ack");
    }
    else if (command.get(0).asString() == "actions")
    {
        if (action_list.size() == 0) { yInfo() << "action list is empty."; }
        for (vector<string>::const_iterator i = action_list.begin(); i != action_list.end(); ++i)
        {
            cout << *i << '\n';
        }
        reply.addString("ack");
    }
    else if (command.get(0).asString() == "actionpos")
    {
        if (actionPos_list.size() == 0) { yInfo() << "action position list is empty."; }
        for (vector<int>::const_iterator i = actionPos_list.begin(); i != actionPos_list.end(); ++i)
        {
            cout << *i << '\n';
        }
        reply.addString("ack");
    }
    else if (command.get(0).asString() == "planid")
    {
        if (planNr_list.size() == 0) { yInfo() << "planNr list is empty."; }
        for (vector<int>::const_iterator i = planNr_list.begin(); i != planNr_list.end(); ++i)
        {
            cout << *i << '\n';
        }
        reply.addString("ack");
    }
    else if (command.get(0).asString() == "newplan")
    {
        if (newPlan.size() == 0) { yInfo() << "newPlan list is empty."; }
        for (auto& item : newPlan)
        {
            yInfo() << item.toString();
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
        fulfill = true;
        yInfo() << "fulfill has been changed to true";
        reply.addString("ack");
    }
    else if (command.get(0).asString() == "manual") {
        if (command.get(1).asString() == "on") {
            manual = true;
        } else if (command.get(1).asString() == "off") {
            manual = false;
        }
        yDebug() << "Now manual mode is " << manual;
        reply.addString("ack");
    }
    else {
        yInfo() << helpMessage;
        reply.addString("wrong command");
    }

    return true;
}

vector<Bottle> Planner::orderPlans(vector<Bottle>& cmdList, const Bottle& cmd) {
    // takes the cmd in format (new (planName priority (objectType objectname))) and inserts it into the list in the right order
    yDebug() << "checking priority of cmd";
    int p = cmd.get(1).asList()->get(1).asInt();
    bool knownPlan = checkKnown(cmd, avaiPlansList);

    if (knownPlan)
    {
        if (p == 0) { yError() << "parsing of command is faulty."; }
        else { yInfo() << "command to be added to newPlan: " << cmd.toString(); }
        if ((p == 1) || (cmdList.size()==0))
        {
            cmdList.push_back(cmd);
        }
        else
        {
            for (unsigned int i = 0; i < cmdList.size(); ++i)
            {
                if (cmdList[i].get(1).asList()->get(1).asInt() < p)
                {
                    yInfo() << "command inserted at position: " << i;
                    cmdList.insert(cmdList.begin() + i, cmd);
                    break;
                }
            }
        }
    }
    else { yInfo() << "Plan " + cmd.get(1).asList()->get(0).asString() + " is not known. Use listplans to see available plans."; }

    return cmdList;
}


/* Called periodically every getPeriod() seconds */
bool Planner::updateModule() {

    if ((!newPlan.empty()) && (action_list.empty()))
    {
        // organise actions for next plan to be executed
        unsigned int it = 0;
        string planName;
        string objectType;
        string object;
        // iterating through first new goal
        int priority;
        Bottle command = newPlan[it];
        if (command.isNull())
        {
            yError() << "bottle command is empty, newPlan: ";
            for (auto& item : newPlan)
            {
                yDebug() << item.toString();
            }
        }
        else
        {
            planName = command.get(1).asList()->get(0).asString();
            objectType = command.get(1).asList()->get(2).asList()->get(0).asString();
            object = command.get(1).asList()->get(2).asList()->get(1).asString();
            newPlan.erase(newPlan.begin());
        }

        // assumption is that the action plan is complete enough that at least one set of prerequisites is met
        bool assumption = false;
        Bottle rep;
        bool state;
        bool presence = false;

        // holding bottles that will be reversed and appended if assumption is met
        vector<string> plan_store;
        vector<string> action_store;
        vector<string> object_store;
        vector<string> type_store;
        vector<int> priority_store;
        vector<int> actionPos_store;
        vector<int> planNr_store;
        // holding the prerequisites that failed
        vector<string> preqFail;

        for (int ii = grpPlans.find(planName + "-totactions").asInt(); ii > 0; ii--)
        {
            // string actionName = grpPlans.find(planName + "action" + to_string(ii)).asString();
            Bottle *fullAction = grpPlans.find(planName + "-action" + to_string(ii)).asList();
            if (fullAction->isNull()) { yError() << "fullAction is empty"; }
            string actionName = fullAction->get(0).asString();
            Bottle args = fullAction->tail();

            // checking if preconditions are met starting from last action
            state = true;
            Bottle preconds = *grpPlans.find(planName + "-" + to_string(ii) + "pre").asList();
            std::list<std::pair<std::string, std::string >> lArgument;
            Bottle auxMsg;
            if (preconds.size() > 0)
            {
                yInfo() << "there are preconditions to check for.";

                for (int k = 0; k < preconds.size(); k++)
                {
                    Bottle* msg = preconds.get(k).asList()->get(1).asList();
                    if (msg->isNull()) { yError() << "msg is empty, preconds is :" << preconds.get(k).asString(); }
                    // format message to sensationsManager
                    for (int i = 0; i < msg->size(); i++)
                    {
                        string aux = msg->get(i).asString();
                        for (int j = 0; j < args.size(); j++)
                        {
                            // check and replace matches in pre/postrequisites and action (e.g. _obj in point)
                            if (args.get(j).asString() == msg->get(i).asString())
                            {
                                aux = object;
                            }
                        }
                        auxMsg.addString(aux);
                    }
                    getState.write(auxMsg, rep);
                    bool indiv;
                    bool negate;
                    string attach = preconds.get(k).asList()->get(0).toString();
                    if (attach.empty()) { yError() << "attach is empty"; }
                    if (attach == "not")
                    {
                        yDebug() << "not";
                        indiv = !rep.get(1).asBool();
                        negate = true;
                    }
                    else
                    {
                        indiv = rep.get(1).asBool();
                        negate = false;
                    }
                    yDebug() << "indiv from rep is " << indiv;
                    yDebug() << "state before && is " << state;
                    state = state && indiv;

                    // record a failed prerequisite
                    string failState;
                    if (!indiv)
                    {
                        // assumes priority implicit in order of prerequisites
                        string keyword = auxMsg.get(2).asString();
                        if (negate) { failState = keyword; }
                        else { failState = "not " + keyword; }

                        if ((keyword == "present") && !presence)
                        {
                            presence = true;
                            preqFail.push_back(failState);
                        }
                        else if ( !((presence) &&
                            (keyword == Object::objectAreaAsString(ObjectArea::ROBOT) ||
                               keyword == Object::objectAreaAsString(ObjectArea::SHARED) ||
                               keyword == Object::objectAreaAsString(ObjectArea::HUMAN) )) )
                        {
                            preqFail.push_back(failState);
                        }
                    }

                    // formulate step for recording in ABM
                    string recState;
                    if (negate) { recState = "not " + auxMsg.get(2).asString(); }
                    else { recState = auxMsg.get(2).asString(); }
                    string strind;
                    if (indiv == true) { strind = "true"; }
                    else { strind = "false"; }
                    lArgument.push_back(std::pair<std::string, std::string>(auxMsg.get(0).asString(), "predicate"+to_string(k)));
                    lArgument.push_back(std::pair<std::string, std::string>(auxMsg.get(1).asString(), "agent"+to_string(k)));
                    lArgument.push_back(std::pair<std::string, std::string>(recState, "object"+to_string(k)));
                    lArgument.push_back(std::pair<std::string, std::string>(strind, "result"+to_string(k)));
                    yDebug() << "lArgument formed for condition";

                    auxMsg.clear();
                }
            }
            else
            {
                yInfo() << "the action has no preconditions.";
                lArgument.push_back(std::pair<std::string, std::string>("none", "predicate"));
                auxMsg.clear();
            }

            if (useABM)
            {
                // record action selection reasoning in ABM
                if (iCub->getABMClient()->Connect())
                {
                    yDebug() << "ABM is connected, recording...";
                    iCub->getABMClient()->sendActivity("reasoning",
                        actionName,
                        "planner",  // expl: "pasar", "drives"...
                        lArgument,
                        true);
                    yInfo() << actionName + " reasoning has been recorded in the ABM";
                }
                else { yInfo() << "ABMClient is not connected, unable to record."; }
            }

            // change objectType to action type (pull or push) for ask behaviour
            if ((actionName == "ask") || (actionName == "moveObject" ))
            {
                yInfo() << "Action is ask/moveObject, objectType now describes if desired action.";
                yInfo() << "direction: " << args.get(0).toString();
                objectType = args.get(0).toString();
            }
            else if ( (actionName == "followingOrder"))
            {
                yInfo() << "Action is ask/moveObject, objectType now describes if desired action.";
                yInfo() << "direction: " << args.get(0).toString();
                objectType = args.get(1).toString();
                object = args.get(0).toString();
            }

            // insert actions into the various holding lists (IN REVERSE ORDER)
            action_store.push_back(actionName);
            priority_store.push_back(priority);
            plan_store.push_back(planName);
            object_store.push_back(object);
            type_store.push_back(objectType);
            actionPos_store.push_back(ii);
            planNr_store.push_back(planNr);

            if (state)
            {
                assumption = true;
                break;
            }
        }

        if (assumption)
        {
            bool rankPriority = false;
                // insert actions into the various lists
            if (!rankPriority)
            {
                for (int a = (int)action_store.size() - 1; a >= 0; a--)
                {
                    yInfo() << "adding action " << action_store[a];
                    action_list.push_back(action_store[a]);
                    priority_list.push_back(priority_store[a]);
                    plan_list.push_back(plan_store[a]);
                    object_list.push_back(object_store[a]);
                    type_list.push_back(type_store[a]);
                    actionPos_list.push_back(actionPos_store[a]);
                    planNr_list.push_back(planNr_store[a]);

                    if (useABM)
                    {
                        // log in ABM
                        if (iCub->getABMClient()->Connect())
                        {
                            std::list<std::pair<std::string, std::string> > lArgument;
                            lArgument.push_back(std::pair<std::string, std::string>("add_action", "predicate"));
                            lArgument.push_back(std::pair<std::string, std::string>("priority", "agent"));
                            lArgument.push_back(std::pair<std::string, std::string>(action_store[a], "object"));
                            lArgument.push_back(std::pair<std::string, std::string>("bottom", "result"));
                            iCub->getABMClient()->sendActivity("reasoning",
                                "priority",
                                "planner",  // expl: "pasar", "drives"...
                                lArgument,
                                true);
                            yInfo() << "addition of " + action_store[a] + " to bottom of list has been recorded in the ABM";
                        }
                        else { yInfo() << "ABMClient is not connected."; }
                    }
                }
            }
        }
        else
        {
            yWarning() << "None of the sets of prerequisites are met, unable to handle plan.";
            string errorMsg = "I could not execute the plan " + planName + " because the " + object + " is ";
            for (unsigned int ii = 0; ii<preqFail.size(); ii++)
            {
                if (ii!=0)
                {
                    errorMsg += " and " + preqFail[ii];
                }
                else
                {
                    errorMsg += preqFail[ii];
                }
            }
            iCub->lookAtPartner();
            yInfo() << errorMsg;
            iCub->say(errorMsg);
            iCub->home();
        }

        // remove elements from holding lists to make way for new plan input
        action_store.clear();
        priority_store.clear();
        plan_store.clear();
        object_store.clear();
        type_store.clear();
        actionPos_store.clear();
        planNr_store.clear();

        planNr += 1;

        preqFail.clear();
    }

    //Check need to fulfill goals
    if (fulfill)
    {
        int skip = 0;

        if (action_list.size() != 0)
        {
            // yInfo() << "executing "<<action_list<<"...";
            yInfo() << "putting homeostasis on hold.";
            freeze_all();
        }
        else
        {
            Time::delay(1.0);
            skip = 1;
        }
        
        if (!skip)
        {
            // execute action
            bool actionCompleted = false;

            // request for BM to execute an action, waits for reply
            Bottle act;
            act.addString(action_list[0]);
            yInfo()<<"Sending " + action_list[0] + " to the BM...";
            Bottle args;
            args.addString(object_list[0]);
            args.addString(type_list[0]);
            act.addList()=args;
            actionCompleted = triggerBehavior(Bottle(act));
            yInfo() << "action has been completed: " << actionCompleted;

            if (useABM)
            {
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
            }

            // check for completed state
            string planName = plan_list[0];
            Bottle stateOI = *grpPlans.find(planName + "-" + to_string(actionPos_list[0]) + "post").asList();
            args.clear();
            args = *grpPlans.find(planName + "-action" + to_string(actionPos_list[0])).asList();
            args = args.tail();

            Bottle emptyBottle("()");
            Bottle sent = *grpPlans.check(planName + "-" + to_string(actionPos_list[0]) + "success", emptyBottle.get(0)).asList();
            string success_sentence;
            for (int i=0;i<sent.size();i++)
            {
                if (sent.get(i).asString()=="_obj")
                    success_sentence = success_sentence +  object_list[0];
                else
                    success_sentence = success_sentence + sent.get(i).asString();
            }
            yDebug() << "sentence_success constructed";

            Bottle objectives = *grpPlans.find(planName + "-objectiveState").asList();

            // checking for ultimate state fulfillment.
            bool desiredState = false;
            bool stateCheck = true;
            if (objectives.size() != 0)
            {
                desiredState = true;
                for (int Ob = 0; Ob < objectives.size(); Ob++)
                {
                    Bottle bot;
                    Bottle rep;

                    Bottle* msg = objectives.get(Ob).asList()->get(1).asList();
                    if (msg->isNull()) { yError() << "bottle msg is empty, objectives was " << objectives.get(Ob).asString(); }
                    for (int i = 0; i < msg->size(); i++)
                    {
                        string aux = msg->get(i).asString();
                        for (int j = 0; j < args.size(); j++)
                        {
                            if (args.get(j).asString() == msg->get(i).asString())
                            {
                                aux = object_list[0];
                            }
                        }
                        bot.addString(aux);
                    }
                    getState.write(bot, rep);
                    yDebug() << bot.toString();
                    bot.clear();
                    bool indiv;
                    string attach = objectives.get(Ob).asList()->get(0).toString();

                    if (attach == "not")
                    {
                        indiv = !rep.get(1).asBool();
                    }
                    else
                    {
                        indiv = rep.get(1).asBool();
                    }

                    desiredState = indiv && desiredState;
                }
            }

            // checking for post condition fulfillment if ultimate state is not met
            if (!desiredState)
            {
                for (int k = 0; k < stateOI.size(); k++)
                {
                    Bottle bot;
                    Bottle rep;

                    Bottle* msg = stateOI.get(k).asList()->get(1).asList();
                    if (msg->isNull()) { yError() << "bottle msg is empty, contents of stateOI is " << stateOI.get(k).asString(); }
                    for (int i = 0; i < msg->size(); i++)
                    {
                        string aux = msg->get(i).asString();
                        for (int j = 0; j < args.size(); j++)
                        {
                            if (args.get(j).asString() == msg->get(i).asString())
                            {
                                aux = object_list[0];
                            }
                        }
                        bot.addString(aux);
                    }
                    getState.write(bot, rep);
                    yDebug() << bot.toString();
                    bot.clear();
                    bool indiv;
                    string attach = stateOI.get(k).asList()->get(0).toString();

                    if (attach == "not")
                    {
                        indiv = !rep.get(1).asBool();
                    }
                    else
                    {
                        indiv = rep.get(1).asBool();
                    }

                    stateCheck = indiv && stateCheck;
                    yDebug() << "State is" << stateCheck;
                }
                yDebug() << "final state is " << stateCheck;
            }

            if (actionCompleted && stateCheck)
            {
                iCub->say(success_sentence);
                yDebug() << "removing action " << action_list[0];
                int currPlan = planNr_list[0];
                action_list.erase(action_list.begin());
                priority_list.erase(priority_list.begin());
                plan_list.erase(plan_list.begin());
                object_list.erase(object_list.begin());
                type_list.erase(type_list.begin());
                actionPos_list.erase(actionPos_list.begin());
                planNr_list.erase(planNr_list.begin());
                attemptCnt = 0;

                if (desiredState)
                {
                    unsigned int length = planNr_list.size();
                    for (unsigned int extra = 0; extra < length; extra++)
                    {
                        if (planNr_list[0] == currPlan)
                        {
                            yDebug() << "removing subsequent action " << action_list[0];
                            action_list.erase(action_list.begin());
                            priority_list.erase(priority_list.begin());
                            plan_list.erase(plan_list.begin());
                            object_list.erase(object_list.begin());
                            type_list.erase(type_list.begin());
                            actionPos_list.erase(actionPos_list.begin());
                            planNr_list.erase(planNr_list.begin());
                        }
                        else { break; }
                    }
                }
            }
            else
            {
                attemptCnt += 1;

                if (useABM)
                {
                    // log in ABM
                    if (iCub->getABMClient()->Connect())
                    {
                        std::list<std::pair<std::string, std::string> > lArgument;
                        lArgument.push_back(std::pair<std::string, std::string>("reasoning", "predicate"));
                        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
                        lArgument.push_back(std::pair<std::string, std::string>(action_list[0], "object"));
                        iCub->getABMClient()->sendActivity("reasoning",
                            "iCub",
                        "planner",  // expl: "pasar", "drives"...
                        lArgument,
                        true);
                        yInfo() << action_list[0] + " failure and reattempt has been recorded in the ABM";
                    }
                    else { yInfo() << "ABMClient is not connected."; }
                }

                if (attemptCnt > 2)
                {                    
                    iCub->say("I have tried too many times and failed. Do it yourself or help me.");
                    yDebug() << "iCub has said that attemptCnt reached.";

                    // removing all actions related to the same plan
                    yDebug() << "removing actions";
                    int currPlan = planNr_list[0];
                    action_list.erase(action_list.begin());
                    priority_list.erase(priority_list.begin());
                    plan_list.erase(plan_list.begin());
                    object_list.erase(object_list.begin());
                    type_list.erase(type_list.begin());
                    actionPos_list.erase(actionPos_list.begin());
                    planNr_list.erase(planNr_list.begin());
                    attemptCnt = 0;

                    unsigned int length = planNr_list.size();
                    for (unsigned int extra = 0; extra < length; extra++)
                    {
                        if (planNr_list[0] == currPlan)
                        {
                            action_list.erase(action_list.begin());
                            priority_list.erase(priority_list.begin());
                            plan_list.erase(plan_list.begin());
                            object_list.erase(object_list.begin());
                            type_list.erase(type_list.begin());
                            actionPos_list.erase(actionPos_list.begin());
                            planNr_list.erase(planNr_list.begin());
                        }
                        else { break; }
                    }
                }
                // wait for a second before continuing (recover from your humiliation, iCub)
                Time::delay(1.0);
            }

            // check again if all actions in the list have been completed
            if(action_list.size() == 0)
            {
                yInfo() << "resuming homeostatic dynamics.";
                unfreeze_all();
            }
        }
    }
    else{
        Time::delay(1);
    }

    return true;
}


bool Planner::triggerBehavior(Bottle cmd)
{
    Bottle reply;
    reply.clear();
    portToBehavior.write(cmd,reply);
    yDebug() << "Bottle contents: " << cmd.toString();
    if (reply.get(0).asString() == "ack")
    {
        return true;
    }
    else
    {
        return false;
    }
}
