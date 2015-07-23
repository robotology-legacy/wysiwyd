#include <string>
#include <iostream>
#include <iomanip>


using namespace std;

class Drive
{
public:
    std::string name;
    double value, homeostasisMin, homeostasisMax, decay;
    bool gradient;

    Drive(std::string d_name, double d_value, double d_homeo_min, double d_homeo_max, double d_decay = 0.05, bool d_gradient = true)
    {

        //todo : check the min/max
        name = d_name;
        value = d_value;
        homeostasisMin = d_homeo_min;
        homeostasisMax = d_homeo_max;
        decay = d_decay;
        gradient = d_gradient;
    }

    Drive()
    {
        name = "defaultDrive";
        value = 0.5;
        homeostasisMin = 0.25;
        homeostasisMax = 0.75;
        decay = 0.0;
        gradient = true;
    }

    Drive(const Drive &b)
    {
        name = b.name;
        value = b.value;
        homeostasisMin = b.homeostasisMin;
        homeostasisMax = b.homeostasisMax;
        decay = b.decay;
        gradient = b.gradient;
    }

    void setValue(double d_value)
    {
        this->value=d_value;
    }
    void deltaValue(double d_value)
    {
        this->value += d_value;
    }

    void setHomeostasisMin(double d_homeo_min)
    {
        this->homeostasisMin = d_homeo_min;
    }
    void deltaHomeostasisMin(double d_homeo_min)
    {
        this->homeostasisMin += d_homeo_min;
    }

    void setHomeostasisMax(double d_homeo_max)
    {
        this->homeostasisMax = d_homeo_max;
    }
    void deltaHomeostasisMax(double d_homeo_max)
    {
        this->homeostasisMax += d_homeo_max;
    }
    
    void setDecay(double d_decay)
    {
        this->decay = d_decay;
    }
    void deltaDecay(double d_decay)
    {
        this->decay += d_decay;
    }
    void update()
    {
        this->value -= this->decay;
    }

    double getValue()
    {
        return(value);
    }

    void setName(std::string n)
    {
        this->name = n;
    }

    void setGradient(bool b)
    {
        this->gradient = b;
    }

};

