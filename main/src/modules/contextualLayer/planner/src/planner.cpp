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
    // while (!Network::connect("/ears/target:o",rpc.getName())) {
    //     yWarning() << "Behavior is not reachable";
    //     yarp::os::Time::delay(0.5);
    // }

    portToBehavior.open("/" + moduleName + "/behavior/cmd:o");
    while (!Network::connect(portToBehavior.getName(),"/BehaviorManager/trigger:i")) {
        yWarning() << "Behavior is not reachable";
        yarp::os::Time::delay(0.5);
    }
    yDebug()<<"Connected!";

    // port_behavior_context.open("/" + moduleName + "/target:o");
    // yInfo()<<"created port to behaviors";

    yDebug()<<"clearing bottles";
    plan_list.clear();
    yDebug()<<"clearing bottles";
    // current_goal = new Bottle();
    current_goal.clear();
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


bool Planner::respond(const Bottle& command, Bottle& reply) {
    LockGuard lg(mutex);
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "quit \n" +
        "help \n" +
        "follow\n" +
        "new <plan>\n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        yInfo() << "quitting";
        return false;
    }else if (command.get(0).asString() == "help") {
        yInfo() << helpMessage;
        reply.addString(helpMessage);
    }else if(command.get(0).asString() == "new"){
        yInfo()<<"Received new plan to execute.";
        plan_list.addString(command.get(1).asString());
        fulfill = 1;
        yInfo()<<command.get(1).asString()<<" is the command given.";
        // (To-Do) Check goal not in list

    }else if(command.get(0).asString() == "follow"){ //Toogle follow goals o goals accomplished
        if (command.size()==1) { fulfill=1; }
        else if (command.size()>1) { fulfill = 1; }
        else{
            reply.addString("nack");
            yInfo()<<"rpc command not valid.";
            return false;
        }
        if (fulfill)
            yInfo()<<"I need to fulfill my goals " ;
        else{
            yInfo()<<"I don't need to fulfill my goals anymore " ;
            current_goal.clear();
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

    //Check need to fulfill goals
    if (fulfill){
        Value val;  //Likely whould be global and not local
        yInfo()<<"plan_list is " << plan_list.toString();
        if ((current_goal.empty()) && (plan_list.size() != 0))
        {
            yDebug() << "executing "<<plan_list.toString()<<"...";
            val = plan_list.pop();
            yDebug() << val.toString();
            current_goal = val.toString();
            yInfo()<<"current_goal set as val.";
            // yDebug() << current_goal->toString();
            yDebug() << "new Current Goal";
        }
        else
        {
            // yDebug() << current_goal->toString();
            yDebug() << "old Current Goal";
        }

        // if (current_goal->size()!=0){
        //     if (current_goal->get(1).asList()->get(0).asString() == "point"){
        //         //Send Cpontext
        //         Bottle &bToTarget = port_behavior_context.prepare();
        //         bToTarget.clear();
        //         bToTarget.append(*current_goal->get(1).asList());
        //         port_behavior_context.write();
        //         yDebug() << "Sending " + bToTarget.toString();

        //         //Trigger behavior
        //         Bottle behCommand;
        //         behCommand.clear();
        //         behCommand.addString("pointingOrder");
        //         behCommand.addList()=*current_goal->get(1).asList();
        //         fulfill = (int)(!triggerBehavior(behCommand));
        //         current_goal->clear();

        //     }
        bool knownPlan = false;
        string planExe;
        for (int i = 0; i < avaiPlansList.size(); i++)
        {
            // iterate through list of drives to check if action plan is known
            yInfo() << "iterating through list, action plan " << avaiPlansList.get(i).asString();
            yInfo()<<"current_goal is " << current_goal;
            if (avaiPlansList.get(i).asString() == current_goal)
            {
                knownPlan = true;
                planExe = avaiPlansList.get(i).asString();
                yInfo() << "plan is known: " + planExe;
            }
        }
// TESTED TILL HERE, ALL IS SO FAR WELL @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        if (knownPlan)
        {
            // execute action plan
            for (int i = 1; i < grpPlans.find(planExe + "-totactions").asInt(); i++)
            {
                yInfo() << "executing action " << i << " of plan " + planExe;
                bool actionCompleted = false;
                while (!actionCompleted)
                {
                    // keep requesting to execute an action until it does get executed
                    string act = grpPlans.find(planExe + "-action" + to_string(i)).asString();
                    yInfo()<<"Sending action " + act + " to the BM.";
                    actionCompleted = (!triggerBehavior(grpPlans.find(planExe + "-action" + i).asString()));

                    // wait for a second before trying again
                    if (!actionCompleted) { Time::delay(1.0); }
                }
            }

            current_goal.clear();
            // yInfo() << "current_goal has been cleared, contents of bottle: " + current_goal->get(0).asString();
            yInfo() << "current_goal has been cleared, contents of bottle: " + current_goal;
        }
        else{
            yDebug() << "I couldn't understand the goal, or I don't have a plan to execute it...";
        }
    }
    else{
        yDebug() << "I need to fulfill goals, but I have none in my list...";
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
        // current_goal->clear();
        return true;
    }
    else
    {
        return false;
    }
}