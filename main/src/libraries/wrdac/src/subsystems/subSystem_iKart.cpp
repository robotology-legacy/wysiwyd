#include <yarp/os/all.h>
#include "wrdac/subsystems/subSystem_iKart.h"

bool wysiwyd::wrdac::SubSystem_iKart::connect()
{
    bool success = true;
    if(!yarp::os::Network::isConnected(portCmd.getName(), "/ikart/control:i")) {
        success &= yarp::os::Network::connect(portCmd.getName(), "/ikart/control:i");
    }
    if(!yarp::os::Network::isConnected("/ikart/odometry:o", portOdometry.getName())) {
        success &= yarp::os::Network::connect("/ikart/odometry:o", portOdometry.getName());
    }
    if(!yarp::os::Network::isConnected("/ikart/laser:o", portLaser.getName())) {
        success &= yarp::os::Network::connect("/ikart/laser:o", portLaser.getName());
    }
    if(!yarp::os::Network::isConnected(portRpc.getName(), "/ikart/rpc")) {
        success &= yarp::os::Network::connect(portRpc.getName(), "/ikart/rpc");
    }
    return success;
}

wysiwyd::wrdac::SubSystem_iKart::SubSystem_iKart(const std::string &masterName) :SubSystem(masterName)
{
    portCmd.open(("/" + m_masterName + "/ikart/cmd:o").c_str());
    portRpc.open(("/" + m_masterName + "/ikart/rpc:o").c_str());
    portOdometry.open(("/" + m_masterName + "/ikart/odometry:i").c_str());
    portLaser.open(("/" + m_masterName + "/ikart/laser:i").c_str());
    m_type = SUBSYSTEM_IKART;
}

wysiwyd::wrdac::SubSystem_iKart::~SubSystem_iKart() {}

void wysiwyd::wrdac::SubSystem_iKart::Close()
{
    portCmd.interrupt(); portCmd.close();
    portRpc.interrupt(); portRpc.close();
    portOdometry.interrupt(); portOdometry.close();
    portLaser.interrupt(); portLaser.close();
}

void wysiwyd::wrdac::SubSystem_iKart::resetOdometer()
{
    yarp::os::Bottle b;
    yarp::os::Bottle reply;
    b.addString("reset_odometry");
    portRpc.write(b, reply);
    std::cout << "Reseting odometry: " << reply.toString() << std::endl;
}

void wysiwyd::wrdac::SubSystem_iKart::printOdometryValues()
{
    std::cout<<"Current iKart odometry: "<<odometry_x<<"\t"<<odometry_y<<"\t"<<odometry_orientation<<std::endl;
}

void wysiwyd::wrdac::SubSystem_iKart::refreshOdometryValues()
{
    yarp::os::Bottle* bOdometry = portOdometry.read(true);
    odometry_x = bOdometry->get(0).asDouble();
    odometry_y = bOdometry->get(1).asDouble();
    odometry_orientation = bOdometry->get(2).asDouble();
}

yarp::os::Bottle wysiwyd::wrdac::SubSystem_iKart::getLaserValues()
{
    yarp::os::Bottle* bLaser = portLaser.read(true);
    return (*bLaser);
}

void wysiwyd::wrdac::SubSystem_iKart::turn(double degreeOffset, double speedMS)
{
    refreshOdometryValues();
    double startingOrienation = odometry_orientation;
    double endingOrientation = startingOrienation + degreeOffset;
    std::cout << "Turning...";
    while (fabs(odometry_orientation - endingOrientation) > 2.0)
    {
        yarp::os::Bottle bCmd;
        bCmd.addInt(2);
        bCmd.addDouble(0.0); //heading
        bCmd.addDouble(0.0); //linear speed
        if (odometry_orientation < endingOrientation)
            bCmd.addDouble(speedMS); //angular speed
        else
            bCmd.addDouble(-speedMS); //angular speed
        portCmd.write(bCmd);
        yarp::os::Time::delay(0.02);

        refreshOdometryValues();
        //printOdometryValues();
    }

    //Zero the velocity
    yarp::os::Bottle bCmd;
    bCmd.addInt(2);
    bCmd.addDouble(0.0); //heading
    bCmd.addDouble(0.0); //linear speed
    bCmd.addDouble(0.0); //angular speed
    portCmd.write(bCmd);
    std::cout << "ok" << std::endl;
}

void wysiwyd::wrdac::SubSystem_iKart::goToWayPoint(const yarp::sig::Vector &wp)
{
    refreshOdometryValues();
    printOdometryValues();

    double tolerance = 0.01;
    double distanceToTarget = sqrt(pow(wp[0] - odometry_x, 2.0) + pow(wp[1] - odometry_y, 2.0));
    double angleToTarget = fabs(wp[2] - odometry_orientation);
    double toleranceAngle = 2.0;
    while (distanceToTarget > tolerance || angleToTarget > toleranceAngle)
    {
        //Refresh the heading based on current odometry
        yarp::sig::Vector currentWp(3, 0.0);
        currentWp[0] = odometry_x;
        currentWp[1] = odometry_y;
        currentWp[2] = odometry_orientation;
        double a = currentWp[2] * M_PI / 180.0;
        std::cout << "alpha=" << a << std::endl;
        yarp::sig::Matrix H(4, 4);
        H.eye();
        H(0, 0) = H(1, 1) = cos(a);
        H(0, 1) = sin(a);
        H(1, 0) = -H(0, 1);
        H(0, 3) = currentWp[0];
        H(1, 3) = currentWp[1];

        yarp::sig::Vector localWp(4, 0.0);
        localWp[0] = wp[0];
        localWp[1] = wp[1];
        localWp[3] = 1.0;

        localWp = yarp::math::pinv(H)*localWp;
        std::cout << "H=" << H.toString(3, 3).c_str() << std::endl;
        std::cout << "Going to a waypoint at : " << wp.toString(3, 3).c_str() << std::endl;
        std::cout << "Current position is : " << currentWp.toString(3, 3).c_str() << std::endl;
        std::cout << "Local expression of waypoint : " << localWp.toString(3, 3).c_str() << std::endl;

        double dX = localWp[0];
        double dY = localWp[1];
        double current_heading = M_PI / 2.0 - atan2(dY, dX);
        current_heading = 180.0 * current_heading / M_PI;

        //Calculate angular speed
        double angularSpeed = std::max(-5.0, std::min(5.0, 0.8*(wp[2] - currentWp[2])));
        double linearSpeed = std::min(0.1, 0.8*(distanceToTarget));

        yarp::os::Bottle bCmd;
        bCmd.addInt(2);
        bCmd.addDouble(current_heading); //heading
        bCmd.addDouble(linearSpeed); //linear speed
        bCmd.addDouble(angularSpeed); //angular speed
        portCmd.write(bCmd);
        yarp::os::Time::delay(0.02);

        refreshOdometryValues();
        distanceToTarget = sqrt(pow(wp[0] - odometry_x, 2.0) + pow(wp[1] - odometry_y, 2.0));
        angleToTarget = fabs(wp[2] - currentWp[2]);
        printOdometryValues();
        std::cout << "Speed : " << linearSpeed << "m/s \t" << angularSpeed << "deg/s" << std::endl;
        std::cout << "Distance to target : " << distanceToTarget << std::endl;
    }

    //Zero the velocity
    yarp::os::Bottle bCmd;
    bCmd.addInt(2);
    bCmd.addDouble(0.0); //heading
    bCmd.addDouble(0.0); //linear speed
    bCmd.addDouble(0.0); //angular speed
    portCmd.write(bCmd);
    std::cout << "ok" << std::endl;
}

void wysiwyd::wrdac::SubSystem_iKart::rawCommand(double linearSpeed, double angularSpeed)
{
    std::cout << "Ikart command : linear " << linearSpeed << "\t angular " << angularSpeed << std::endl;
    yarp::os::Bottle bCmd;
    bCmd.clear();
    bCmd.addInt(2);
    bCmd.addDouble(0.0); //heading
    bCmd.addDouble(linearSpeed); //linear speed
    bCmd.addDouble(angularSpeed); //angular speed
    portCmd.write(bCmd);
    yarp::os::Time::delay(0.1);
}
