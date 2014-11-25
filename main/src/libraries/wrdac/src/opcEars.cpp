#include <wrdac/clients/opcEars.h>
#include <algorithm>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace wysiwyd::wrdac;

Bottle opcEars::snapshot(Bottle bInput, OPCClient *OPCReal)
{
    Bottle bOutput;
    Bottle bName = *bInput.get(1).asList();
    if (!OPCReal->isConnected())
    {
        bOutput.addString("Error, OPC not connected");
        return bOutput;
    }


    cout << "bName : " << bName.toString() << endl;
    if (!bName.get(1).isString())
    {
        bOutput.addString("Error wrong format of input");
    }
    ostringstream osName;
    map<string, opcSave>::iterator it_Map;
    string sName = bName.get(1).asString().c_str();
    osName << sName << instance;
    sName += osName.str();

    OPCReal->checkout();
    OPCReal->update();
    opcNew.lEntities = OPCReal->EntitiesCacheCopy();
    opcNew.lRelations = OPCReal->getRelations();

    for (list<Entity*>::iterator it_E = opcNew.lEntities.begin(); it_E != opcNew.lEntities.end(); it_E++)
    {   // Check all the entities to find the iCub
        if ( ( (*it_E)->name() == "icub" || (*it_E)->name() == "iCub" ) && ( (*it_E)->entity_type() == EFAA_OPC_ENTITY_AGENT) )
        {
            Agent AgA;
            AgA.fromBottle((*it_E)->asBottle()); // Converstion Entity -> Agent
            opcNew.lDrives.clear();
            opcNew.lEmotions.clear();
            for (map<string, Drive>::iterator it_D = AgA.m_drives.begin(); it_D != AgA.m_drives.end(); it_D++)
            {
                opcNew.lDrives.push_back(it_D->second); // Copy the Drives in opcSave
            }

            for (map<string, double>::iterator it_Em = AgA.m_emotions_intrinsic.begin(); it_Em != AgA.m_emotions_intrinsic.end(); it_Em++)
            {
                pair<string, double> pTemp;
                pTemp.first = it_Em->first;
                pTemp.second = it_Em->second;
                opcNew.lEmotions.push_back(pTemp); // Copy the Emotions in opcSave
            }

        }
    }

    mSave[sName] = opcNew;
    bOutput.addString(sName.c_str());
    bOutput.addString("snapshot done");

    return bOutput;
}


Bottle opcEars::snapshot_string(string sName, OPCClient *OPCReal)
{
    Bottle bOutput;
    opcNew.lEntities = OPCReal->EntitiesCacheCopy();
    opcNew.lRelations = OPCReal->getRelations();

    for (list<Entity*>::iterator it_E = opcNew.lEntities.begin(); it_E != opcNew.lEntities.end(); it_E++)
    {   // Check all the entities to find the iCub
        if ( ( (*it_E)->name() == "icub" || (*it_E)->name() == "iCub" ) && ( (*it_E)->entity_type() == EFAA_OPC_ENTITY_AGENT) )
        {
            Agent AgA;
            AgA.fromBottle((*it_E)->asBottle()); // Converstion Entity -> Agent
            opcNew.lDrives.clear();
            opcNew.lEmotions.clear();
            for (map<string, Drive>::iterator it_D = AgA.m_drives.begin(); it_D != AgA.m_drives.end(); it_D++)
            {
                opcNew.lDrives.push_back(it_D->second); // Copy the Drives in opcSave
            }

            for (map<string, double>::iterator it_Em = AgA.m_emotions_intrinsic.begin(); it_Em != AgA.m_emotions_intrinsic.end(); it_Em++)
            {
                pair<string, double> pTemp;
                pTemp.first = it_Em->first;
                pTemp.second = it_Em->second;
                opcNew.lEmotions.push_back(pTemp); // Copy the Emotions in opcSave
            }

        }
    }

    mSave[sName] = opcNew;
    bOutput.addString(sName.c_str());
    bOutput.addString("snapshot done");

    return bOutput;
}


/*
* Return a bottle with the format for insertion in the DB
* output[0] : entitytype
* output[1] : insert_into_contentopc
* output[2] : insert_into_entitytype_table
* output[3] : insert_into_beliefs if agent
* output[4] : true or false, is there is beliefs or not
*/
Bottle opcEars::insertEntity(Entity *A)
{
    Bottle bOutput,
        bA;
    bA = A->asBottle();

    int opcID;
    opcID = A->opc_id();
    
    ostringstream osEntity,
        osBeliefs, // if the entity is an agent
        osContent;

    bOutput.addString(A->entity_type().c_str());
    osContent <<  " ('entity', " << instance << " , " << opcID << " , '" << A->entity_type().c_str() << "') ";

    bOutput.addString(osContent.str().c_str());

    osEntity << " (" << opcID << " , '" << A->name().c_str() << "' , " << instance ;
  
    
    if (A->entity_type() == "entity")
    {
        osEntity << ") ";
        bOutput.addString(osEntity.str().c_str());
    }

    // If entities are object :
    if (A->entity_type() == EFAA_OPC_ENTITY_OBJECT)
    {
        Object OA;
        OA.fromBottle(bA);
 
         if (OA.m_present)
             osEntity << " , TRUE , '{ ";
        else
            osEntity << " , FALSE , '{ ";

        //  Insert position
        osEntity << OA.m_ego_position[0] << " , " << OA.m_ego_position[1] << " , " << OA.m_ego_position[2] << " }' , '{ " ;

        // Insert orientation
        osEntity << OA.m_ego_orientation[0] << " , " << OA.m_ego_orientation[1] << " , " << OA.m_ego_orientation[2] << " }' , '{ " ;

        // Insert dimension
        osEntity << OA.m_dimensions[0] << " , " << OA.m_dimensions[1] << " , " << OA.m_dimensions[2] << " }' , '{ " ;
     
        //Insert color
        osEntity << OA.m_color[0] << " , " << OA.m_color[1] << " , " << OA.m_color[2] << " }' ,  " ;

        // Insert Saliency
        osEntity << OA.m_saliency << " ) " ;

        bOutput.addString(osEntity.str().c_str());  
    }

    // If entities are agent
    if (A->entity_type() == EFAA_OPC_ENTITY_AGENT)
    {
        Agent AgA;
        AgA.fromBottle(bA);
        bool fBelief = false;
 
        if (AgA.m_present)
             osEntity << " , TRUE , '{ ";
        else
            osEntity << " , FALSE , '{ ";

        //  Insert position
        osEntity << AgA.m_ego_position[0] << " , " << AgA.m_ego_position[1] << " , " << AgA.m_ego_position[2] << " }' , '{ " ;

        // Insert orientation
        osEntity << AgA.m_ego_orientation[0] << " , " << AgA.m_ego_orientation[1] << " , " << AgA.m_ego_orientation[2] << " }' , '{ " ;

        // Insert dimension
        osEntity << AgA.m_dimensions[0] << " , " << AgA.m_dimensions[1] << " , " << AgA.m_dimensions[2] << " }' , '{ " ;
     
        //Insert color
        osEntity << AgA.m_color[0] << " , " << AgA.m_color[1] << " , " << AgA.m_color[2] << " }' ,  " ;

        // Insert Saliency
        osEntity << AgA.m_saliency << " ) " ;


        // Get the beliefs:
        // output format : (instance, idagent, subject, verb, object, time, place, manner )
        // for each beliefs
        for (list<Relation>::iterator it_Beli = AgA.beliefs().begin() ; it_Beli != AgA.beliefs().end() ; it_Beli++)
        {
            if (!fBelief)
            {
                osBeliefs << " ( " << instance << " , " << AgA.opc_id() << " , '" << it_Beli->subject() << "' , '" << it_Beli->verb() << "' , '" << it_Beli->object() << "' , '" << it_Beli->complement_time() << "' , '" << it_Beli->complement_place() << "' , '" << it_Beli->complement_manner() << "' ) "  ;
                fBelief = true;
            }
            else
                osBeliefs << ", ( " << instance << " , " << AgA.opc_id() << " , '" << it_Beli->subject() << "' , '" << it_Beli->verb() << "' , '" << it_Beli->object() << "' , '" << it_Beli->complement_time() << "' , '" << it_Beli->complement_place() << "' , '" << it_Beli->complement_manner() << "' ) "  ;
        }



        bOutput.addString(osEntity.str().c_str());
        bOutput.addString(osBeliefs.str().c_str());
        fBelief ? bOutput.addString("true") : bOutput.addString("false");

    
    }


    // If entities are rt_object :
    if (A->entity_type() == EFAA_OPC_ENTITY_RTOBJECT)
    {
        RTObject RTA;
        RTA.fromBottle(bA);

        if (RTA.m_present)
            osEntity << " , TRUE , '{ ";
        else
            osEntity << " , FALSE , '{ ";

        //  Insert position
        osEntity << RTA.m_ego_position[0] << " , " << RTA.m_ego_position[1] << " , " << RTA.m_ego_position[2] << " }' , '{ " ;

        // Insert orientation
        osEntity << RTA.m_ego_orientation[0] << " , " << RTA.m_ego_orientation[1] << " , " << RTA.m_ego_orientation[2] << " }' , '{ " ;

        // Insert dimension
        osEntity << RTA.m_dimensions[0] << " , " << RTA.m_dimensions[1] << " , " << RTA.m_dimensions[2] << " }' , '{ " ;
     
        //Insert color
        osEntity << RTA.m_color[0] << " , " << RTA.m_color[1] << " , " << RTA.m_color[2] << " }' , '{ " ;

        //Insert rt_position
        osEntity << RTA.m_rt_position[0] << " , " << RTA.m_rt_position[1] << " , " << RTA.m_rt_position[2] << " }' , " ;

        // Insert Saliency
        osEntity << RTA.m_saliency << " ) ";

        bOutput.addString(osEntity.str().c_str());
    }

    // If entities are adjectives :
    if (A->entity_type() == EFAA_OPC_ENTITY_ADJECTIVE)
    {
        Adjective AdjA;
        AdjA.fromBottle(bA);
 
        osEntity << " , '" << AdjA.m_quality.c_str() << "') ";

        bOutput.addString(osEntity.str().c_str());
    }

    // If entities are action :
    if (A->entity_type() == EFAA_OPC_ENTITY_ACTION)
    {
        Action ActA, ActB;
        ActA.fromBottle(bA);
 
        Relation desc = ActA.description();

        osEntity << " , '" << EFAA_OPC_ENTITY_ACTION << "' ) ";

        bOutput.addString(osEntity.str().c_str());
    }

    return bOutput;
}


Bottle opcEars::insertRelation(Relation R)
{
    Bottle bOutput;
    int opcID;
    ostringstream osContent,
        osRelation;
    opcID = R.ID();
     
    osContent << " ('Relation' , " << instance << " , " << opcID << " , 'Relation' ) " ;

    osRelation << " ( " << opcID << " , " << instance << " , '" << R.subject().c_str() << "' , '" << R.verb().c_str() << "' , '" << R.object().c_str() << "' , '" << R.complement_time().c_str() << "' , '" << R.complement_manner().c_str() << "' , '" << R.complement_place().c_str() << "' ) ";

//  bOutput.addString(osContent.str().c_str());
    bOutput.addString(osRelation.str().c_str());

    return bOutput;
}


Bottle opcEars::insertDrives(Drive D)
{
    Bottle bOutput;
    ostringstream osDrives;
    
    osDrives << " ( " << instance << " , '" << D.name.c_str() << "' , " << D.value << " , " << D.homeoStasisMax << " , " << D.homeoStasisMin << " ) ";
    bOutput.addString(osDrives.str().c_str());

    return bOutput;
}


Bottle opcEars::insertEmotion(pair<string, double> Emo)
{
    Bottle bOutput;
    ostringstream osEmotion;

    osEmotion << " ( " << instance << " , '" << Emo.first.c_str() << "' , " << Emo.second << " ) ";

    bOutput.addString(osEmotion.str().c_str());

    return bOutput;
}


Bottle opcEars::insertOPC(string sName)
{
    Bottle bOutput,         // Output of the function
        bEntities,          // Bottle with the 2 string for insertion of the entities
        bRelations,         // Bottle with the insertion of the relations
        bDrives,            // Bottle with the insertion of the drives
        bEmotions,          // Bottle with the insertion of the emotions
        bBeliefs,           // Bottle with the insertion of the beliefs
        bTemp;              // Temporary bottle with an entity (entitytype, content, entity)

    ostringstream osContent,
        osEntity,
        osAction,
        osAgent,
        osObject,
        osRTObject,
        osRelation,
        osEmotion,
        osDrives,
        osBeliefs,
        osAdjective;

    bool fEntity = false,
        fAction = false,
        fAgent = false,
        fObject = false,
        fRTObject = false,
        fAdjct = false,
        fRelation = false,
        fEmotion = false,
        fDrives = false,
        fBeliefs = false,
        fContent = false;

    osContent << "INSERT INTO contentopc( type , instance , opcid , subtype) VALUES ";
    osEntity << "INSERT INTO entities(opcid, name, instance) VALUES ";
    osObject <<   "INSERT INTO object(opcid, name, instance, presence, position, orientation, dimension, color, saliency) VALUES ";
    osRTObject << "INSERT INTO rtobject(opcid, name, instance, presence, position, orientation, dimension, color, rtposition, saliency) VALUES ";
    osAgent << "INSERT INTO agent(opcid, name, instance, presence, position, orientation, dimension, color, saliency) VALUES ";
    osAdjective << "INSERT INTO adjective(opcid, name, instance, quality) VALUES ";
    osAction << "INSERT INTO action(opcid, name, instance, argument) VALUES ";
    osRelation << "INSERT INTO relation( opcid, instance, subject, verb, object, time, manner, place ) VALUES ";
    osEmotion << "INSERT INTO emotions (instance, name, value) VALUES ";
    osBeliefs << "INSERT INTO beliefs (instance, idagent, subject, verb, object, time, place, manner) VALUES ";
    osDrives << "INSERT INTO drives (instance, name, value, homeomax, homeomin) VALUES ";


    opcSave opcTemp = mSave[sName];


    // ---- Entities ---- //
    for (list<Entity*>::iterator it_E = opcTemp.lEntities.begin(); it_E != opcTemp.lEntities.end(); it_E++ )
    {
        bTemp = insertEntity(*it_E);
    
        cout << "bTemp = " << bTemp.toString() << endl ;

        if (!fContent)
            osContent << bTemp.get(1).toString().c_str() ;
        else
            osContent << " , " << bTemp.get(1).toString().c_str() ;

        fContent = true;

        if (bTemp.get(0).toString() == "entity")
        {
            if (fEntity)
                osEntity << " , " ;
            osEntity << bTemp.get(2).toString().c_str();
            fEntity = true;
        }
        else if (bTemp.get(0).toString() == EFAA_OPC_ENTITY_ACTION)
        {
            if (fAction)
                osAction << " , " ;
            osAction << bTemp.get(2).toString().c_str();
            fAction = true;
        }
        else if (bTemp.get(0).toString() == EFAA_OPC_ENTITY_ADJECTIVE)
        {       
            if (fAdjct)
                osAdjective << " , " ;
            osAdjective << bTemp.get(2).toString().c_str();
            fAdjct = true;
        }
        else if (bTemp.get(0).toString() == EFAA_OPC_ENTITY_AGENT)
        {
            if (fAgent)
                osAgent << " , " ;
            osAgent << bTemp.get(2).toString().c_str();
            if (bTemp.get(4).toString() == "true" )
            {
                if (fBeliefs)
                    osBeliefs << " , ";
                osBeliefs << bTemp.get(3).toString().c_str();
                fBeliefs = true;
            }
            fAgent = true;
        }

        else if (bTemp.get(0).toString() == EFAA_OPC_ENTITY_OBJECT)
        {
            if (fObject)
                osObject << " , " ;
            osObject << bTemp.get(2).toString().c_str();
            fObject = true ;
        }

        else if (bTemp.get(0).toString() == EFAA_OPC_ENTITY_RTOBJECT)
        {
            if (fRTObject)
                osRTObject << " , " ;
            osRTObject << bTemp.get(2).toString().c_str();
            fRTObject = true;
        }
    }

    bEntities.addString(osContent.str().c_str());

    // ---- RELATIONS ---- //

    for (list<Relation>::iterator it_R = opcTemp.lRelations.begin(); it_R != opcTemp.lRelations.end(); it_R++ )
    {
        if (!fContent)
            osContent << " ( 'relation' , " << instance << " , " << it_R->ID() << " ,  'relation' ) " ;
        else
            osContent << " , ( 'relation' , " << instance << " , " << it_R->ID() << " ,  'relation' ) " ;
        
bTemp  = insertRelation(*it_R); 
        if (!fRelation)
            osRelation << bTemp.get(0).toString().c_str() ;
        else
            osRelation << " , " << bTemp.get(0).toString().c_str() ;
        fRelation = true;
    }


    // ---- DRIVES ---- //

    for (list<Drive>::iterator it_D = opcTemp.lDrives.begin(); it_D != opcTemp.lDrives.end(); it_D++)
    {
        bTemp = insertDrives(*it_D);
        if (!fDrives)
            osDrives << bTemp.get(0).toString().c_str();
        else
            osDrives << " , " << bTemp.get(0).toString().c_str() ;
        fDrives = true;
    }


    // ---- EMOTIONS ---- //

    for (list<pair < string, double> >::iterator it_Emo = opcTemp.lEmotions.begin(); it_Emo != opcTemp.lEmotions.end(); it_Emo++)
    {
        bTemp = insertEmotion(*it_Emo);
        if (!fEmotion)
            osEmotion << bTemp.get(0).toString().c_str() ;
        else
            osEmotion << " , " << bTemp.get(0).toString().c_str() ;
        fEmotion = true;
    }




    // ---- filing bOutput ---- //

    bOutput.addString(osContent.str().c_str());

    if (fEntity)
        bOutput.addString(osEntity.str().c_str());

    if (fAction)
        bOutput.addString(osAction.str().c_str());

    if (fAgent)
        bOutput.addString(osAgent.str().c_str());

    if (fObject)
        bOutput.addString(osObject.str().c_str());

    if (fRTObject)
        bOutput.addString(osRTObject.str().c_str());

    if (fAdjct)
        bOutput.addString(osAdjective.str().c_str());
    
    if (fEmotion)
        bOutput.addString(osEmotion.str().c_str());

    if (fRelation)
        bOutput.addString(osRelation.str().c_str());

    if (fDrives)
        bOutput.addString(osDrives.str().c_str());


    return bOutput;
}


string opcEars::InttoStr(int input)
{
    ostringstream stOutput;
    stOutput << input;
    return stOutput.str();
}


string opcEars::DoutoStr(double input)
{
    ostringstream stOutput;
    stOutput << input;
    return stOutput.str();
}


Bottle opcEars::getDifferencies(Entity *A, Entity *B)
{

    Bottle bOutput,bTemp,bA,bB;
    bA = A->asBottle();
    bB = B->asBottle();

    // If entities are agents : 
    if (A->entity_type() == EFAA_OPC_ENTITY_AGENT && B->entity_type() == EFAA_OPC_ENTITY_AGENT)
    {
        Agent AgA,AgB;
        AgA.fromBottle(bA);
        AgB.fromBottle(bB);

        bTemp = getDiffAgent(&AgA, &AgB);
    }

    // If entities are rt_object :
     else if (A->entity_type() == EFAA_OPC_ENTITY_RTOBJECT && B->entity_type() == EFAA_OPC_ENTITY_RTOBJECT)
    {
        RTObject RTA, RTB;
        RTA.fromBottle(bA);
        RTB.fromBottle(bB);

        bTemp = getDiffRTObject(&RTA, &RTB);
    }

    // If entities are adjectives :
    else if (A->entity_type() == EFAA_OPC_ENTITY_ADJECTIVE && B->entity_type() == EFAA_OPC_ENTITY_ADJECTIVE)
    {
        Adjective AdjA, AdjB;
        AdjA.fromBottle(bA);
        AdjB.fromBottle(bB);

        bTemp = getDiffAdj(&AdjA, &AdjB);
    }

    // If entities are action :
    else if (A->entity_type() == "action" && B->entity_type() == "action")
    {
        Action ActA, ActB;
        ActA.fromBottle(bA);
        ActB.fromBottle(bB);

        bTemp = getDiffAction(&ActA, &ActB);
    }

    if (bTemp.toString() == "")
    {
        bOutput.addString("none");
        return bOutput;
    }

    bOutput.addString(A->name().c_str());
    bOutput.addList() = bTemp;

    return bOutput;
}


Bottle opcEars::getDiffAgent(Agent *AgA, Agent *AgB)
{

    Bottle bOutput,bTemp,bBeliefsRemoved, bBeliefsAdded, bEmotion;
    bOutput = getDiffObject(AgA, AgB);

    bBeliefsRemoved.addString("Beliefs removed");
    bBeliefsAdded.addString("Beliefs added");

    map<string, double> mAEmotion = AgA->m_emotions_intrinsic;
    map<string, double> mBEmotion = AgB->m_emotions_intrinsic;

    map<string, double>::iterator it_MapA, it_MapB;

    list<Relation> lBeliefA = AgA->beliefs();//*AgA->beliefs();
    list<Relation> lBeliefB = AgB->beliefs();

    // beliefs in A and not in B
    // For each element of A
    for (list<Relation>::iterator it_A = lBeliefA.begin() ; it_A != lBeliefA.end() ; it_A++)
    {
        bool found = false;
        // for each element of B
        for (list<Relation>::iterator it_B = lBeliefB.begin() ; it_B != lBeliefB.end() ; it_B++)
        {
            if (!found)
            {
                if ((it_A->asLightBottle().toString()) == (it_B->asLightBottle().toString()))
                {
                    found = true;
                }
            }
        }
        if (!found)
        {
            bBeliefsRemoved.addList() = it_A->asLightBottle();
        }
    }

    // beliefs in B and not in A
    // For each element of B
    for (list<Relation>::iterator it_B = lBeliefB.begin() ; it_B != lBeliefB.end() ; it_B++)
    {
        bool found = false;
        // for each element of A
        for (list<Relation>::iterator it_A = lBeliefA.begin() ; it_A != lBeliefA.end() ; it_A++)
        {
            if (!found)
            {
                if ((it_A->asLightBottle().toString()) == (it_B->asLightBottle().toString()))
                {
                    found = true;
                }
            }
        }
        if (!found)
        {
            bBeliefsAdded.addList() = it_B->asLightBottle();
        }
    }


    if (bBeliefsAdded.size() !=1)
        bOutput.addList() = bBeliefsAdded;

    if (bBeliefsRemoved.size() !=1)
        bOutput.addList() = bBeliefsRemoved;


    bEmotion.addString("emotions");
    // Emotion
    for (it_MapA = mAEmotion.begin(); it_MapA != mAEmotion.end(); it_MapA++)
    {
        if (mBEmotion[it_MapA->first] != it_MapA->second)
        {
            bTemp.clear();
            bTemp.addString(it_MapA->first.c_str());
            bTemp.addDouble(it_MapA->second-mBEmotion[it_MapA->first]);
            bEmotion.addList() = bTemp;
        }
    }


    if (bEmotion.size()>1)
    {
        bOutput.addList() = bEmotion;
    }

    return bOutput;
}


Bottle opcEars::getDiffObject(Object *AgA, Object *AgB)
{

    Bottle bOutput,bTemp;

    // Colors
    if (!(AgA->m_color == AgB->m_color))
    {
        if (AgA->m_color[0] != AgB->m_color[0])
        {
            bTemp.clear();
            bTemp.addString(EFAA_OPC_OBJECT_GUI_COLOR_R);
            bTemp.addDouble(AgB->m_color[0]-AgA->m_color[0]);
            bOutput.addList() = bTemp;
        }
        if (AgA->m_color[1] != AgB->m_color[1])
        {
            bTemp.clear();
            bTemp.addString(EFAA_OPC_OBJECT_GUI_COLOR_G);
            bTemp.addDouble(AgB->m_color[1]-AgA->m_color[1]);
            bOutput.addList() = bTemp;
        }
        if (AgA->m_color[2] != AgB->m_color[2])
        {
            bTemp.clear();
            bTemp.addString(EFAA_OPC_OBJECT_GUI_COLOR_B);
            bTemp.addDouble(AgB->m_color[2]-AgA->m_color[2]);
            bOutput.addList() = bTemp;
        }
    }


    // Dimensions
    if (!(AgA->m_dimensions == AgB->m_dimensions))
    {
        if (AgA->m_dimensions[0] != AgB->m_dimensions[0])
        {
            bTemp.clear();
            bTemp.addString(EFAA_OPC_OBJECT_RTDIMX_TAG);
            bTemp.addDouble(AgB->m_dimensions[0]-AgA->m_dimensions[0]);
            bOutput.addList() = bTemp;
        }
        if (AgA->m_dimensions[1] != AgB->m_dimensions[1])
        {
            bTemp.clear();
            bTemp.addString(EFAA_OPC_OBJECT_RTDIMY_TAG);
            bTemp.addDouble(AgB->m_dimensions[1]-AgA->m_dimensions[1]);
            bOutput.addList() = bTemp;
        }
        if (AgA->m_dimensions[2] != AgB->m_dimensions[2])
        {
            bTemp.clear();
            bTemp.addString(EFAA_OPC_OBJECT_RTDIMZ_TAG);
            bTemp.addDouble(AgB->m_dimensions[2]-AgA->m_dimensions[2]);
            bOutput.addList() = bTemp;
        }
    }

    //Ego Position
    if (!(AgA->m_ego_position == AgB->m_ego_position))
    {
        if (AgA->m_ego_position[0] != AgB->m_ego_position[0])
        {
            bTemp.clear();
            bTemp.addString(EFAA_OPC_OBJECT_ROBOTPOSX_TAG);
            bTemp.addDouble(AgB->m_ego_position[0]-AgA->m_ego_position[0]);
            bOutput.addList() = bTemp;
        }
        if (AgA->m_ego_position[1] != AgB->m_ego_position[1])
        {
            bTemp.clear();
            bTemp.addString(EFAA_OPC_OBJECT_ROBOTPOSY_TAG);
            bTemp.addDouble(AgB->m_ego_position[1]-AgA->m_ego_position[1]);
            bOutput.addList() = bTemp;
        }
        if (AgA->m_ego_position[2] != AgB->m_ego_position[2])
        {
            bTemp.clear();
            bTemp.addString(EFAA_OPC_OBJECT_ROBOTPOSZ_TAG);
            bTemp.addDouble(AgB->m_ego_position[2]-AgA->m_ego_position[2]);
            bOutput.addList() = bTemp;
        }
    }

    //Ego Orientation
    if (!(AgA->m_ego_orientation == AgB->m_ego_orientation))
    {
        if (AgA->m_ego_orientation[0] != AgB->m_ego_orientation[0])
        {
            bTemp.clear();
            bTemp.addString(EFAA_OPC_OBJECT_ROBOTORX_TAG);
            bTemp.addDouble(AgB->m_ego_orientation[0]-AgA->m_ego_orientation[0]);
            bOutput.addList() = bTemp;
        }
        if (AgA->m_ego_orientation[1] != AgB->m_ego_orientation[1])
        {
            bTemp.clear();
            bTemp.addString(EFAA_OPC_OBJECT_ROBOTORY_TAG);
            bTemp.addDouble(AgB->m_ego_orientation[1]-AgA->m_ego_orientation[1]);
            bOutput.addList() = bTemp;
        }
        if (AgA->m_ego_orientation[2] != AgB->m_ego_orientation[2])
        {
            bTemp.clear();
            bTemp.addString(EFAA_OPC_OBJECT_ROBOTORZ_TAG);
            bTemp.addDouble(AgB->m_ego_orientation[2]-AgA->m_ego_orientation[2]);
            bOutput.addList() = bTemp;
        }
    }

    // Present
    if (AgA->m_present != AgB->m_present)
    {
        bTemp.clear();
        bTemp.addString(EFAA_OPC_OBJECT_PRESENT_TAG);
        bTemp.addInt(AgB->m_present - AgA->m_present);
        bOutput.addList() = bTemp;
    }

    return bOutput;
}


Bottle opcEars::getDiffRTObject(RTObject *AgA, RTObject *AgB)
{

    Bottle bOutput,bTemp;
    bOutput = getDiffObject(AgA, AgB);

    if (!(AgA->m_rt_position == AgB->m_rt_position))
    {
        if (AgA->m_rt_position[0] != AgB->m_rt_position[0])
        {
            bTemp.clear();
            bTemp.addString("rt_position_x");
            bTemp.addDouble(AgB->m_rt_position[0]-AgA->m_rt_position[0]);
            bOutput.addList() = bTemp;
        }
        if (AgA->m_rt_position[1] != AgB->m_rt_position[1])
        {
            bTemp.clear();
            bTemp.addString("rt_position_y");
            bTemp.addDouble(AgB->m_rt_position[1]-AgA->m_rt_position[1]);
            bOutput.addList() = bTemp;
        }
        if (AgA->m_rt_position[2] != AgB->m_rt_position[2])
        {
            bTemp.clear();
            bTemp.addString("rt_position_z");
            bTemp.addDouble(AgB->m_rt_position[2]-AgA->m_rt_position[2]);
            bOutput.addList() = bTemp;
        }
    }

    return bOutput;
}


Bottle opcEars::getDiffAdj(Adjective *AgA, Adjective *AgB)
{
    Bottle bOutput,bTemp;

    if(AgA->m_quality != AgB->m_quality)
    {
        bTemp.addString("qualityType");
        bTemp.addString(AgB->m_quality.c_str());
        bOutput.addList() = bTemp;
    }
    return bOutput;
}


Bottle opcEars::getDiffAction(Action *AgA, Action *AgB)
{
    Bottle bOutput;
    Relation descA = AgA->description();
    Relation descB = AgB->description();
    if (descA.subject() != descB.subject())
    {
        Bottle sub;
        sub.addString("subject");
        sub.addString(descB.subject().c_str());
        bOutput.addList() = sub;
    }

    if (descA.verb() != descB.verb())
    {
        Bottle sub;
        sub.addString("verb");
        sub.addString(descB.subject().c_str());
        bOutput.addList() = sub;
    }

    if (descA.object() != descB.object())
    {
        Bottle sub;
        sub.addString("object");
        sub.addString(descB.subject().c_str());
        bOutput.addList() = sub;;
    }

    if (descA.complement_manner() != descB.complement_manner())
    {
        Bottle sub;
        sub.addString("complement_manner");
        sub.addString(descB.subject().c_str());
        bOutput.addList() = sub;
    }

    if (descA.complement_place() != descB.complement_place())
    {
        Bottle sub;
        sub.addString("complement_place");
        sub.addString(descB.subject().c_str());
        bOutput.addList() = sub;
    }

    if (descA.complement_time() != descB.complement_time())
    {
        Bottle sub;
        sub.addString("complement_time");
        sub.addString(descB.subject().c_str());
        bOutput.addList() = sub;
    }

    return bOutput;
}

