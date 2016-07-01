#include "goalManager.h"
#include "wrdac/subsystems/subSystem_recog.h"

bool GoalManager::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("goalManager")).asString().c_str();
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
    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);
    while (!Network::connect("/ears/target:o",rpc.getName())) {
        yWarning() << "Behavior is not reachable";
        yarp::os::Time::delay(0.5);
    }

    portToBehavior.open("/" + moduleName + "/behavior/cmd:o");
    while (!Network::connect(portToBehavior.getName(),"/BehaviorManager/trigger:i ")) {
        yWarning() << "Behavior is not reachable";
        yarp::os::Time::delay(0.5);
    }
    yDebug() << "Connected!";

    port_behavior_context.open("/" + moduleName + "/target:o");
    yInfo()<<"created port to behaviors";
    
    goal_need_port.open("/" + moduleName + "/goal:o");
    yInfo()<<"created port to planner";
    if (Network::connect(goal_need_port.GetName(),"/planner/rpc")) {
        yInfo() << "planner and goalManager are connected!";
    }
    else { yInfo() << "planner and goalManager are NOT connected."; }

    yDebug()<<"clearing bottles";
    goal_list.clear();
    yDebug()<<"clearing bottles";
    current_goal = new Bottle();
    current_goal->clear();
    yDebug()<<"cleared all bottles";
    id=0;
    fulfill=0;

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";
    
    return true;
}


bool GoalManager::close() {
    if(iCub) {
        iCub->close();
        delete iCub;
    }

    portToBehavior.interrupt();
    portToBehavior.close();

    port_behavior_context.interrupt();
    port_behavior_context.close();

    // goal_need_port.interrupt();
    // goal_need_port.close();

    rpc.interrupt();
    rpc.close();

    return true;
}


bool GoalManager::respond(const Bottle& command, Bottle& reply) {
    LockGuard lg(mutex);
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "quit \n" +
        "help \n" +
        "new <goal> \n" +
        "follow \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }else if (command.get(0).asString() == "help") {
        yInfo() << helpMessage;
        reply.addString(helpMessage);
    }else if(command.get(0).asString() == "new"){
        yInfo()<<"Received new goal.";
        // (To-Do) Check goal not in list

        //add goal to list
        Bottle aux;
        aux.clear();
        aux.addInt(id);
        id++;
        aux.addList()=*(command.get(1).asList());
        goal_list.addList()=aux;
        yInfo()<<"New goal added to the list: " << aux.toString();
        reply.addString("ack");
    }else if(command.get(0).asString() == "follow"){ //Toogle follow goals o goals accomplished
        if (command.size()==1)
            fulfill=true;
        else if (command.size()>1)
            fulfill = command.get(1).asInt();
        else{
            reply.addString("nack");
            return false;
        }
        if (fulfill)
            yInfo()<<"I need to fulfill my goals " ;
        else{
            yInfo()<<"I don't need to fulfill my goals anymore " ;
            current_goal->clear();
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
bool GoalManager::updateModule() {
    
    //check goals in goal list
    // if (goal_list.size() != 0){
    //     //send rpc to allostasis  
    //     yDebug()<<"There are "<< goal_list.size() << " goals to fulfill...";
    //     Bottle &bGoal = goal_need_port.prepare();
    //     bGoal.clear();
    //     bGoal.addInt(1);
    //     goal_need_port.write();
    //     yDebug()<< "allostasis was informed...";
    // }else if(current_goal->size()==0){
    //     yDebug()<<"There are "<< goal_list.size() << " goals to fulfill...";
    //     Bottle &bGoal = goal_need_port.prepare();
    //     bGoal.clear();
    //     bGoal.addInt(0);
    //     goal_need_port.write();
    //     yDebug()<< "allostasis was informed...";
    // }
    
    //Check need to fulfill goals
    if (fulfill){
        Value val;  //Likely whould be global and not local
            if (current_goal->size()==0 && goal_list.size() != 0){
                yDebug()<< goal_list.toString();
                val = goal_list.pop();
                yDebug()<< val.toString();
                current_goal = val.asList();
                yDebug()<<current_goal->toString();
                yDebug()<<"new Current Goal";
            }else{
                yDebug()<<"old Current Goal";
            }//select action and send context
            /*
            yDebug()<< current_goal->toString();
            yDebug()<< current_goal->get(0).toString();
            yDebug()<< current_goal->get(1).toString();
            */
            if (current_goal->size()!=0){
                if (current_goal->get(1).asList()->get(0).asString() == "point"){
                    //Send Cpontext
                    Bottle &bToTarget = port_behavior_context.prepare();
                    bToTarget.clear();
                    bToTarget.append(*current_goal->get(1).asList());
                    port_behavior_context.write();
                    yDebug() << "Sending " + bToTarget.toString();
    
                    //Trigger behavior
                    Bottle behCommand;
                    behCommand.clear();
                    behCommand.addString("pointingOrder");
                    behCommand.addList()=*current_goal->get(1).asList();
                    fulfill = (int)(!triggerBehavior(behCommand));
                    current_goal->clear();
    
                }else{
                    yDebug()<< "I couldn't understand the goal, or I don't have a plan to execute it...";
                }
            }else{
                yDebug()<< "I need to fulfill goals, but I have none in my list...";
            }
    }
    
    
    return true;
}


bool GoalManager::triggerBehavior(Bottle cmd)
{
    yDebug()<<"sending behavior...";
    
    Bottle reply;
    reply.clear();
    portToBehavior.write(cmd,reply);
    if (reply.get(0).asString()=="ack")
        current_goal->clear();
    return true;
}