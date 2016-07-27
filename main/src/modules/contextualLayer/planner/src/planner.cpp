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
    //iCub = new ICubClient(moduleName, "goalManager", "client.ini", isRFVerbose);
    //iCub->opc->isVerbose = false;
    //if (!iCub->connect())
    //{
    //    yInfo() << " iCubClient : Some dependencies are not running...";
    //    Time::delay(1.0);
    //}
    
    grpPlans = rf.findGroup("PLANS");
    avaiPlansList = *grpPlans.find("plans").asList();

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);
    // receive input from ears module
    while (!Network::connect("/ears/target:o",rpc.getName())) {
        yWarning() << "ears is not reachable";
        yarp::os::Time::delay(0.5);
    }

    portToBehavior.open("/" + moduleName + "/behavior/cmd:o");
    while (!Network::connect(portToBehavior.getName(),"/BehaviorManager/trigger:i")) {
        yWarning() << "Behavior is not reachable";
        yarp::os::Time::delay(0.5);
    }
    yDebug()<<"Connected to BM!";
 
    toHomeo.open("/manager/toHomeostasis/rpc:o");
    while (!Network::connect(toHomeo.getName(),"/homeostasis/rpc")) {
        yWarning() << "homeostasis is not reachable";
        yarp::os::Time::delay(0.5);
    }

    // port_behavior_context.open("/" + moduleName + "/target:o");
    // yInfo()<<"created port to behaviors";

    actPt = action_list.begin();
    prioPt = priority_list.begin();

    yDebug()<<"clearing vectors";
    plan_list.clear();
    priority_list.clear();
    action_list.clear();
    // current_goal = new Bottle();
    // current_goal.clear();
    yDebug()<<"cleared all bottles";
    id=0;
    fulfill=0;

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
            yInfo() << "iterating through list, action plan " << avaiPlansList.get(i).asString();

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
    else if (command.get(0).asString() == "new"){
        // rpc command was of type "new (plan priority (objectType object))"
        yInfo() << "Received input of type <new (plan priority (objectType object))> from ears.";
        ordering = true;
        newPlan.push_back(command);
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

    yInfo() << "ordering status: " << ordering;
    if (ordering)
    {
        for (int it = 0; it < newPlan.size(); it++)
        {
            bool knownPlan = false;
            string planExe;
            int priority;
            Bottle command = newPlan[it];
            knownPlan = checkKnown(command, avaiPlansList, planExe);

            if (knownPlan)
            {
                int insertID = 0;
                bool rankPriority = true;
                priority = command.get(1).asList()->get(1).asInt();
                if (priority_list.size() != 0)
                {
                    for (int i = 0; i < priority_list.size(); i++)
                    {
                        if (priority == 1)
                        {
                            yInfo() << "lowest priority - append to bottom of list";
                            rankPriority = false;
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
                string planName = command.get(1).asList()->get(0).asString();
                plan_list.push_back(planName);

                if (!rankPriority)
                {
                    for (int i = 1; i < grpPlans.find(planName + "-totactions").asInt() + 1; i++)
                    {
                        string act = grpPlans.find(planName + "-action" + to_string(i)).asString();
                        yInfo() << "adding action " << act;
                        action_list.push_back(act);
                        priority_list.push_back(priority);
                    }

                }
                else
                {
                    yInfo() << "insertID:" << insertID;
                    vector<string>::iterator idx = action_list.begin() + insertID;
                    vector<int>::iterator indx = priority_list.begin() + insertID;
                    for (int i = grpPlans.find(planName + "-totactions").asInt(); i > 0; i--)
                    {
                        string act = grpPlans.find(planName + "-action" + to_string(i)).asString();
                        yInfo() << "adding action " << act;
                        idx = action_list.insert(idx, act);
                        indx = priority_list.insert(indx, priority);
                    }
                    // if (fulfill)
                    // {
                    //     actPt += grpPlans.find(planName + "-totactions").asInt();
                    //     prioPt += grpPlans.find(planName + "-totactions").asInt();
                    // }
                }

                objectType = command.get(1).asList()->get(2).asList()->get(0).asString();
                object = command.get(1).asList()->get(2).asList()->get(1).asString();
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

        if (action_list.size() != 0)
        {
            yInfo() << "executing "<<action_list<<"...";
            yInfo() << "putting homeostasis on hold.";
            freeze_all();
        }

        else
        {
            fulfill = 0;
            yDebug() << "no actions in list, fulfill is set to 0.";
        }
        
        // execute action
        bool actionCompleted = false;

        // keep requesting to execute an action until it does get executed
        string act = action_list[0];
        yInfo()<<"Sending action " + act + " to the BM.";
        actionCompleted = triggerBehavior(Bottle(act));
        yInfo() << "action has been completed: " << actionCompleted;

        // wait for a second before trying again
        if (!actionCompleted) { Time::delay(1.0); }
        else
        {
            // action has been successfully completed
            yInfo() << "removing action " << *action_list.begin();
            action_list.erase(action_list.begin());
            priority_list.erase(priority_list.begin());

            yInfo() << "action completed and removed";
        }

        if(action_list.size() == 0)
        {
            fulfill = 0;
            yInfo() << "resuming homeostatic dynamics.";
            unfreeze_all();
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