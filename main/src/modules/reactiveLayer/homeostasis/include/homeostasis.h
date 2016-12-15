#include <string>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <limits> 
#include <time.h>

using namespace std;

class Drive
{
public:
    std::string name;
    double period;
    double value, homeostasisMin, homeostasisMax, decay, valueMin, valueMax, default_value, decay_multiplier;
    bool gradient;
    time_t start_sleep;
    bool is_sleeping;
    double time_to_sleep;

    Drive(std::string d_name, double _period, double d_value=0.5, double d_homeo_min=0.25, double d_homeo_max=0.75, double d_decay = 0.05, double d_value_min=numeric_limits<double>::min(), double d_value_max=numeric_limits<double>::max(), bool d_gradient = false)
    {
        name = d_name;
        period = _period;
        value = d_value;
        homeostasisMin = d_homeo_min;
        homeostasisMax = d_homeo_max;
        default_value = (d_homeo_max + d_homeo_min)/2.;
        decay = d_decay;
        gradient = d_gradient;
        decay_multiplier = 1;
        is_sleeping = false;
        //todo : check the min/max
        double homeoRange =  homeostasisMax - homeostasisMin;
        if (d_value_min == numeric_limits<double>::min() && d_value_max == numeric_limits<double>::max()){
            valueMin = homeostasisMin - 0.5 * homeoRange;
            valueMax = homeostasisMax + 0.5 * homeoRange;
        } else {
            valueMin = d_value_min;
            valueMax = d_value_max;
        }
             

    }

    Drive()
    {
        cout << "Drive created using the default construtor, you should not do this" << endl;
        name = "defaultDrive";
        value = 0.5;
        homeostasisMin = 0.25;
        homeostasisMax = 0.75;
        decay = 0.0;
        gradient = true;
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

    void setDecayMultiplier(double mult)
    {
        this->decay_multiplier = mult;
    }

    // double sigDecay()
    // {
    //     double aux = (1/(1+exp(-this->value)))-0.5;
    //     if (aux<0)
    //         aux=-aux;
    //     //yDebug() << "sig function: "<< aux;// << endl;
    //     return this->decay*(aux-1);

    // }

    void sleep(double t) {
        start_sleep = time(NULL);
        is_sleeping = true;
        time_to_sleep = t;
    }

    void freeze() {
        is_sleeping = true;
        time_to_sleep = 1e10;
    }

    void reset(){
        value = default_value;
    }

    void unfreeze() {
        is_sleeping = false;
    }

    void update()
    {
        if (is_sleeping) {
            if (difftime(time(NULL), start_sleep) > time_to_sleep) {
                is_sleeping = false;
            }
        }
        else if (! ((this->value > valueMax && this->decay<0) || (this->value < valueMin && this->decay>0))) {
            this->value -= (this->decay * this->decay_multiplier * period);           
        }


        //yDebug()<<"real decay: "<<this->sigDecay();//<<endl;
        //this->value -= this->sigDecay();
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

