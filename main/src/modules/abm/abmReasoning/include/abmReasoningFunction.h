#ifndef _ABMREASONINGFUNCTION_H_
#define _ABMREASONINGFUNCTION_H_

#include "wrdac/clients/icubClient.h"
#include <tuple>
#include <time.h>
#include <numeric>

#define ae_machineepsilon 5E-16
#define ae_maxrealnumber  1E300
#define ae_minrealnumber  1E-300


const double PI = 3.141592654;


class abmReasoningFunction
{
public:

    abmReasoningFunction(yarp::os::ResourceFinder &rf);

    static std::string s_realOPC;               // name of the real OPC
    static std::string s_mentalOPC;              // name of the mental OPC

    //table
    static double   X_center;               // X of the center of the table
    static double   Y_center;               // Y of the center of the table
    static double   X_origin;               // X Origin of the reference of the table
    static double   Y_origin;               // Y Origin of the reference of the table
    static double   table_radius;           // Distance from the user to the center of the table

    //mental
    static double threshold_time_sequence;       //threshold of a same sequence
    static double height_location;      // coordonate in Z of a location in the OPC
    static double size_location;                    // size in Z of a location

    static int  color_dream_R;                  // color of the dreamed object in the OPC
    static int  color_dream_G;
    static int  color_dream_B;

    static int  color_loc_R;                    // color of the locations in the OPC
    static int  color_loc_G;
    static int  color_loc_B;

    static int  DIFFERENCE_DATE_IN_SECOND;      // threshold return of second if 2 actions are at different dates

    static double LIFETIME_RELATION;                // life time of a relation about the objects in the OPC

    //Spatialisation
    static unsigned int  THRESHOLD_DETERMINE_INFLUENCE;           // number of tries before deterstd::mine if location
    static double FACTOR_LOCATION;                  // factor of the size of a location : center +/- FACTOR_LOCATION * std dev
    static double THRESHOLD_IS_AT_LOCATION;
    static double THRESHOLD_IS_AT_TEMPORAL_LOCATION;
    static double THRESHOLD_IS_DISPERSION;          // is lower, then the dispersion of a cloud of point is "null".

    // PDDL
    static double THRESHOLD_INTERSECT_SUP;
    static double THRESHOLD_INTERSECT_INF;
    static double THRESHOLD_PRESENCE;
    static double THRESHOLD_ABSENCE;
    static double THRESHOLD_CONFIDENCE_RELATION;

    //TAGS
    static std::string TAG_LOCATION;
    static std::string TAG_IS_AT_LOC;
    static std::string TAG_DEFAULT;
    static std::string TAG_SPEAKER;
    static std::string TAG_ADRESSEE;
    static std::string TAG_SUBJECT;
    static std::string TAG_NONE;
    static std::string TAG_NULL;
    static std::string TAG_ACTION;
    static std::string TAG_SEPARATOR;
    static std::string TAG_SENTENCE;
    static std::string TAG_PREDICATE;
    static std::string TAG_AGENT;
    static std::string TAG_OBJECT;
    static std::string TAG_RECIPIENT;

    //DB
    static std::string TAG_DB_ACTION;
    static std::string TAG_DB_COMPLEX;
    static std::string TAG_DB_BEHAVIOR;
    static std::string TAG_DB_SHARED_PLAN;
    static std::string TAG_DB_ARGUMENT;
    static std::string TAG_DB_NONE;
    static std::string TAG_DB_MANNER;
    static std::string TAG_DB_UNKNOWN;

    //GK
    static std::string TAG_SPEAKER_IS_RECEIVER;
    static std::string TAG_ADRESSEE_IS_SPEAKER;
    static std::string TAG_SPEAKER_IS_AGENT;
    static std::string TAG_AGENT_IS_SPEAKER;
    static std::string TAG_AGENT_IS_RECEIVER;
    static std::string TAG_ADRESSEE_IS_AGENT;
    static int  SIGMA_LEARNING_GRAMMAR;
    static double   THRESHOLD_CONFIDENCE_GRAMMAR;

    //ADJ
    static double THRESHOLD_PVALUE_INFLUENCE_TIMING;






    // FUNCTIONS

    static std::pair<double, double> coordFromString(std::string);

    static bool timeDiff(struct tm TM1, struct tm TM2);
    static struct tm string2Time(std::string sTime);
    static std::string time2string(struct tm Time);
    static int  timeDiffSecondFromString(std::string T1, std::string T2);
    static std::pair<std::string, std::string> ago2string(std::pair<int, std::string> pInput);

    static std::vector<double> getCovMatrix(std::vector<std::pair<double, double> > vXY);
    static std::vector<double> getCovMatrix(std::vector<double> vX, std::vector<double> vY);

    static double getMahalaDist(std::vector<double> vX, std::vector<double> vY, std::pair<double, double> XY);
    static double getMahalaDist(std::vector<std::pair<double, double> > vData, std::pair<double, double> XY);

    static std::tuple<int, int, int> tupleIntFromString(std::string sInput);
    static std::tuple<double, double, double> tupleDoubleFromString(std::string sInput);


    static void studentttest2(/* Real    */ std::vector<double> x,
        /* Real    */ std::vector<double> y,
        double* bothtails,
        double* lefttail,
        double* righttail)
    {
        int i;
        double x0;
        double y0;
        double xmean;
        double ymean;
        double stat;
        double s;
        double p;

        int n = x.size();
        int m = y.size();


        *bothtails = 0;
        *lefttail = 0;
        *righttail = 0;

        if (n <= 0 || m <= 0)
        {
            *bothtails = 1.0;
            *lefttail = 1.0;
            *righttail = 1.0;
            return;
        }

        /*
        * Mean
        */
        xmean = 0;
        x0 = x[0];
        for (i = 0; i <= n - 1; i++)
        {

            xmean = xmean + x[i];
        }
        xmean = xmean / n;


        ymean = 0;

        y0 = y[0];
        for (i = 0; i <= m - 1; i++)
        {
            ymean = ymean + y[i];
        }
        ymean = ymean / m;


        /*
        * S
        */
        s = 0;
        if (n + m > 2)
        {
            for (i = 0; i <= n - 1; i++)
            {
                s = s + (x[i] - xmean)*(x[i] - xmean);
            }
            for (i = 0; i <= m - 1; i++)
            {
                s = s + (y[i] - xmean)*(y[i] - xmean);
            }
            s = sqrt(s*(1. / n + 1. / m) / (n + m - 2));
        }
        if (s == 0)
        {
            if (xmean == ymean)
            {
                *bothtails = 1.0;
            }
            else
            {
                *bothtails = 0.0;
            }
            if (xmean > ymean)
            {
                *lefttail = 1.0;
            }
            else
            {
                *lefttail = 0.0;
            }
            if (xmean <= ymean)
            {
                *righttail = 1.0;
            }
            else
            {
                *righttail = 0.0;
            }
            return;
        }

        /*
        * Statistic
        */
        stat = (xmean - ymean) / s;
        p = studenttdistribution(n + m - 2, stat);
        *bothtails = 2 * std::min(p, 1 - p);
        *lefttail = p;
        *righttail = 1 - p;
    }



    static double studenttdistribution(int k, double t)
    {
        double x;
        double rk;
        double z;
        double f;
        double tz;
        double p;
        double xsqk;
        int j;
        double result;


        if (t == 0)
        {
            result = 0.5;
            return result;
        }
        if (t < -2.0)
        {
            rk = k;
            z = rk / (rk + t*t);
            result = 0.5*incompletebeta(0.5*rk, 0.5, z);
            return result;
        }
        if (t < 0)
        {
            x = -t;
        }
        else
        {
            x = t;
        }
        rk = k;
        z = 1.0 + x*x / rk;
        if (k % 2 != 0)
        {
            xsqk = x / sqrt(rk);
            p = atan(xsqk);
            if (k > 1)
            {
                f = 1.0;
                tz = 1.0;
                j = 3;
                while (j <= k - 2 && (tz / f > ae_machineepsilon))
                {
                    tz = tz*((j - 1) / (z*j));
                    f = f + tz;
                    j = j + 2;
                }
                p = p + f*xsqk / z;
            }
            p = p*2.0 / PI;
        }
        else
        {
            f = 1.0;
            tz = 1.0;
            j = 2;
            while (j <= k - 2 && (tz / f > ae_machineepsilon))
            {
                tz = tz*((j - 1) / (z*j));
                f = f + tz;
                j = j + 2;
            }
            p = f*x / sqrt(z*rk);
        }
        if (t < 0)
        {
            p = -p;
        }
        result = 0.5 + 0.5*p;
        return result;
    }


    static double incompletebeta(double a, double b, double x)
    {
        double t;
        double xc;
        double w;
        double y;
        int flag;
        double sg;
        double big;
        double biginv;
        double maxgam;
        double minlog;
        double maxlog;
        double result;


        big = 4.503599627370496e15;
        biginv = 2.22044604925031308085e-16;
        maxgam = 171.624376956302725;
        minlog = log(ae_minrealnumber);
        maxlog = log(ae_maxrealnumber);

        //    ae_assert(ae_fp_greater(a,0)&&ae_fp_greater(b,0), "Domain error in IncompleteBeta" );
        //    ae_assert(ae_fp_greater_eq(x,0)&&ae_fp_less_eq(x,1), "Domain error in IncompleteBeta" );


        if (x == 0)
        {
            result = 0.;
            return result;
        }
        if (x == 1.)
        {
            result = 1;
            return result;
        }
        flag = 0;
        if ((b*x < 1.0) && (x <= 0.95))
        {
            result = ibetaf_incompletebetaps(a, b, x, maxgam);
            return result;
        }
        w = 1.0 - x;
        if (x > a / (a + b))
        {
            flag = 1;
            t = a;
            a = b;
            b = t;
            xc = x;
            x = w;
        }
        else
        {
            xc = w;
        }
        if (flag == 1 && (b*x <= 1.0) && (x <= 0.95))
        {
            t = ibetaf_incompletebetaps(a, b, x, maxgam);
            if (t <= ae_machineepsilon)
            {
                result = 1.0 - ae_machineepsilon;
            }
            else
            {
                result = 1.0 - t;
            }
            return result;
        }
        y = x*(a + b - 2.0) - (a - 1.0);
        if (y < 0.0)
        {
            w = ibetaf_incompletebetafe(a, b, x, big, biginv);
        }
        else
        {
            w = ibetaf_incompletebetafe2(a, b, x, big, biginv) / xc;
        }
        y = a*log(x);
        t = b*log(xc);
        if ((a + b < maxgam) && (fabs(y) < maxlog) && (fabs(t) < maxlog))
        {
            t = pow(xc, b);
            t = t*pow(x, a);
            t = t / a;
            t = t*w;
            t = t*(gammafunction(a + b) / (gammafunction(a)*gammafunction(b)));
            if (flag == 1)
            {
                if (t <= ae_machineepsilon)
                {
                    result = 1.0 - ae_machineepsilon;
                }
                else
                {
                    result = 1.0 - t;
                }
            }
            else
            {
                result = t;
            }
            return result;
        }
        y = y + t + lngamma(a + b, &sg) - lngamma(a, &sg) - lngamma(b, &sg);
        y = y + log(w / a);
        if (y < minlog)
        {
            t = 0.0;
        }
        else
        {
            t = exp(y);
        }
        if (flag == 1)
        {
            if (t <= ae_machineepsilon)
            {
                t = 1.0 - ae_machineepsilon;
            }
            else
            {
                t = 1.0 - t;
            }
        }
        result = t;
        return result;
    }


    /*************************************************************************
    Power series for incomplete beta integral.
    Use when b*x is small and x not too close to 1.

    Cephes Math Library, Release 2.8:  June, 2000
    Copyright 1984, 1995, 2000 by Stephen L. Moshier
    *************************************************************************/
    static double ibetaf_incompletebetaps(double a,
        double b,
        double x,
        double maxgam)
    {
        double s;
        double t;
        double u;
        double v;
        double n;
        double t1;
        double z;
        double ai;
        double sg;
        double result;


        ai = 1.0 / a;
        u = (1.0 - b)*x;
        v = u / (a + 1.0);
        t1 = v;
        t = u;
        n = 2.0;
        s = 0.0;
        z = ae_machineepsilon*ai;
        while (fabs(v) > z)
        {
            u = (n - b)*x / n;
            t = t*u;
            v = t / (a + n);
            s = s + v;
            n = n + 1.0;
        }
        s = s + t1;
        s = s + ai;
        u = a*log(x);
        if ((a + b < maxgam) && (fabs(u) < log(ae_maxrealnumber)))
        {
            t = gammafunction(a + b) / (gammafunction(a)*gammafunction(b));
            s = s*t*pow(x, a);
        }
        else
        {
            t = lngamma(a + b, &sg) - lngamma(a, &sg) - lngamma(b, &sg) + u + log(s);
            if ((t < log(ae_minrealnumber)))
            {
                s = 0.0;
            }
            else
            {
                s = exp(t);
            }
        }
        result = s;
        return result;
    }

    /*************************************************************************
    Continued fraction expansion #1 for incomplete beta integral

    Cephes Math Library, Release 2.8:  June, 2000
    Copyright 1984, 1995, 2000 by Stephen L. Moshier
    *************************************************************************/
    static double ibetaf_incompletebetafe(double a,
        double b,
        double x,
        double big,
        double biginv)
    {
        double xk;
        double pk;
        double pkm1;
        double pkm2;
        double qk;
        double qkm1;
        double qkm2;
        double k1;
        double k2;
        double k3;
        double k4;
        double k5;
        double k6;
        double k7;
        double k8;
        double r;
        double t;
        double ans;
        double thresh;
        int n;
        double result;


        k1 = a;
        k2 = a + b;
        k3 = a;
        k4 = a + 1.0;
        k5 = 1.0;
        k6 = b - 1.0;
        k7 = k4;
        k8 = a + 2.0;
        pkm2 = 0.0;
        qkm2 = 1.0;
        pkm1 = 1.0;
        qkm1 = 1.0;
        ans = 1.0;
        r = 1.0;
        n = 0;
        thresh = 3.0*ae_machineepsilon;
        do
        {
            xk = -x*k1*k2 / (k3*k4);
            pk = pkm1 + pkm2*xk;
            qk = qkm1 + qkm2*xk;
            pkm2 = pkm1;
            pkm1 = pk;
            qkm2 = qkm1;
            qkm1 = qk;
            xk = x*k5*k6 / (k7*k8);
            pk = pkm1 + pkm2*xk;
            qk = qkm1 + qkm2*xk;
            pkm2 = pkm1;
            pkm1 = pk;
            qkm2 = qkm1;
            qkm1 = qk;
            if (qk != 0)
            {
                r = pk / qk;
            }
            if (r != 0)
            {
                t = fabs((ans - r) / r);
                ans = r;
            }
            else
            {
                t = 1.0;
            }
            if (t < thresh)
            {
                break;
            }
            k1 = k1 + 1.0;
            k2 = k2 + 1.0;
            k3 = k3 + 2.0;
            k4 = k4 + 2.0;
            k5 = k5 + 1.0;
            k6 = k6 - 1.0;
            k7 = k7 + 2.0;
            k8 = k8 + 2.0;
            if ((fabs(qk) + fabs(pk) > big))
            {
                pkm2 = pkm2*biginv;
                pkm1 = pkm1*biginv;
                qkm2 = qkm2*biginv;
                qkm1 = qkm1*biginv;
            }
            if ((fabs(qk) < biginv) || (fabs(pk) < biginv))
            {
                pkm2 = pkm2*big;
                pkm1 = pkm1*big;
                qkm2 = qkm2*big;
                qkm1 = qkm1*big;
            }
            n = n + 1;
        } while (n != 300);
        result = ans;
        return result;
    }


    /*************************************************************************
    Continued fraction expansion #2
    for incomplete beta integral

    Cephes Math Library, Release 2.8:  June, 2000
    Copyright 1984, 1995, 2000 by Stephen L. Moshier
    *************************************************************************/
    static double ibetaf_incompletebetafe2(double a,
        double b,
        double x,
        double big,
        double biginv)
    {
        double xk;
        double pk;
        double pkm1;
        double pkm2;
        double qk;
        double qkm1;
        double qkm2;
        double k1;
        double k2;
        double k3;
        double k4;
        double k5;
        double k6;
        double k7;
        double k8;
        double r;
        double t;
        double ans;
        double z;
        double thresh;
        int n;
        double result;


        k1 = a;
        k2 = b - 1.0;
        k3 = a;
        k4 = a + 1.0;
        k5 = 1.0;
        k6 = a + b;
        k7 = a + 1.0;
        k8 = a + 2.0;
        pkm2 = 0.0;
        qkm2 = 1.0;
        pkm1 = 1.0;
        qkm1 = 1.0;
        z = x / (1.0 - x);
        ans = 1.0;
        r = 1.0;
        n = 0;
        thresh = 3.0*ae_machineepsilon;
        do
        {
            xk = -z*k1*k2 / (k3*k4);
            pk = pkm1 + pkm2*xk;
            qk = qkm1 + qkm2*xk;
            pkm2 = pkm1;
            pkm1 = pk;
            qkm2 = qkm1;
            qkm1 = qk;
            xk = z*k5*k6 / (k7*k8);
            pk = pkm1 + pkm2*xk;
            qk = qkm1 + qkm2*xk;
            pkm2 = pkm1;
            pkm1 = pk;
            qkm2 = qkm1;
            qkm1 = qk;
            if (qk != 0)
            {
                r = pk / qk;
            }
            if (r != 0)
            {
                t = fabs((ans - r) / r);
                ans = r;
            }
            else
            {
                t = 1.0;
            }
            if (t < thresh)
            {
                break;
            }
            k1 = k1 + 1.0;
            k2 = k2 - 1.0;
            k3 = k3 + 2.0;
            k4 = k4 + 2.0;
            k5 = k5 + 1.0;
            k6 = k6 + 1.0;
            k7 = k7 + 2.0;
            k8 = k8 + 2.0;
            if ((fabs(qk) + fabs(pk) > big))
            {
                pkm2 = pkm2*biginv;
                pkm1 = pkm1*biginv;
                qkm2 = qkm2*biginv;
                qkm1 = qkm1*biginv;
            }
            if ((fabs(qk) < biginv) || (fabs(pk) < biginv))
            {
                pkm2 = pkm2*big;
                pkm1 = pkm1*big;
                qkm2 = qkm2*big;
                qkm1 = qkm1*big;
            }
            n = n + 1;
        } while (n != 300);
        result = ans;
        return result;
    }


    /*************************************************************************
    Gamma function

    Input parameters:
    X   -   argument

    Domain:
    0 < X < 171.6
    -170 < X < 0, X is not an integer.

    Relative error:
    arithmetic   domain     # trials      peak         rms
    IEEE    -170,-33      20000       2.3e-15     3.3e-16
    IEEE     -33,  33     20000       9.4e-16     2.2e-16
    IEEE      33, 171.6   20000       2.3e-15     3.2e-16

    Cephes Math Library Release 2.8:  June, 2000
    Original copyright 1984, 1987, 1989, 1992, 2000 by Stephen L. Moshier
    Translated to AlgoPascal by Bochkanov Sergey (2005, 2006, 2007).
    *************************************************************************/
    static double gammafunction(double x)
    {
#ifndef ALGLIB_INTERCEPTS_SPECFUNCS
        double p;
        double pp;
        double q;
        double qq;
        double z;
        int i;
        double sgngam;
        double result;


        sgngam = 1;
        q = fabs(x);
        if (q > 33.0)
        {
            if ((x < 0.0))
            {
                p = (floor(q));
                i = static_cast<int>(p);
                if (i % 2 == 0)
                {
                    sgngam = -1;
                }
                z = q - p;
                if ((z > 0.5))
                {
                    p = p + 1;
                    z = q - p;
                }
                z = q*sin(PI*z);
                z = fabs(z);
                z = PI / (z*gammafunc_gammastirf(q));
            }
            else
            {
                z = gammafunc_gammastirf(x);
            }
            result = sgngam*z;
            return result;
        }
        z = 1;
        while ((x >= 3))
        {
            x = x - 1;
            z = z*x;
        }
        while ((x < 0))
        {
            if ((x > -0.000000001))
            {
                result = z / ((1 + 0.5772156649015329*x)*x);
                return result;
            }
            z = z / x;
            x = x + 1;
        }
        while ((x < 2))
        {
            if ((x < 0.000000001))
            {
                result = z / ((1 + 0.5772156649015329*x)*x);
                return result;
            }
            z = z / x;
            x = x + 1.0;
        }
        if ((x == 2))
        {
            result = z;
            return result;
        }
        x = x - 2.0;
        pp = 1.60119522476751861407E-4;
        pp = 1.19135147006586384913E-3 + x*pp;
        pp = 1.04213797561761569935E-2 + x*pp;
        pp = 4.76367800457137231464E-2 + x*pp;
        pp = 2.07448227648435975150E-1 + x*pp;
        pp = 4.94214826801497100753E-1 + x*pp;
        pp = 9.99999999999999996796E-1 + x*pp;
        qq = -2.31581873324120129819E-5;
        qq = 5.39605580493303397842E-4 + x*qq;
        qq = -4.45641913851797240494E-3 + x*qq;
        qq = 1.18139785222060435552E-2 + x*qq;
        qq = 3.58236398605498653373E-2 + x*qq;
        qq = -2.34591795718243348568E-1 + x*qq;
        qq = 7.14304917030273074085E-2 + x*qq;
        qq = 1.00000000000000000320 + x*qq;
        result = z*pp / qq;
        return result;
#else
        return _ialglib_i_gammafunction(x);
#endif
    }


    static double gammafunc_gammastirf(double x)
    {
        double y;
        double w;
        double v;
        double stir;
        double result;


        w = 1 / x;
        stir = 7.87311395793093628397E-4;
        stir = -2.29549961613378126380E-4 + w*stir;
        stir = -2.68132617805781232825E-3 + w*stir;
        stir = 3.47222221605458667310E-3 + w*stir;
        stir = 8.33333333333482257126E-2 + w*stir;
        w = 1 + w*stir;
        y = exp(x);
        if ((x > 143.01608))
        {
            v = pow(x, 0.5*x - 0.25);
            y = v*(v / y);
        }
        else
        {
            y = pow(x, x - 0.5) / y;
        }
        result = 2.50662827463100050242*y*w;
        return result;
    }



    static double lngamma(double x, double* sgngam)
    {
#ifndef ALGLIB_INTERCEPTS_SPECFUNCS
        double a;
        double b;
        double c;
        double p;
        double q;
        double u;
        double w;
        double z;
        int i;
        double logpi;
        double ls2pi;
        double tmp;
        double result;

        *sgngam = 0;

        *sgngam = 1;
        logpi = 1.14472988584940017414;
        ls2pi = 0.91893853320467274178;
        if ((x < -34.0))
        {
            q = -x;
            w = lngamma(q, &tmp);
            p = (floor(q));
            i = static_cast<int>(p);
            if (i % 2 == 0)
            {
                *sgngam = -1;
            }
            else
            {
                *sgngam = 1;
            }
            z = q - p;
            if ((z > 0.5))
            {
                p = p + 1;
                z = p - q;
            }
            z = q*sin(PI*z);
            result = logpi - log(z) - w;
            return result;
        }
        if ((x < 13))
        {
            z = 1;
            p = 0;
            u = x;
            while ((u >= 3))
            {
                p = p - 1;
                u = x + p;
                z = z*u;
            }
            while ((u < 2))
            {
                z = z / u;
                p = p + 1;
                u = x + p;
            }
            if ((z < 0))
            {
                *sgngam = -1;
                z = -z;
            }
            else
            {
                *sgngam = 1;
            }
            if ((u == 2))
            {
                result = log(z);
                return result;
            }
            p = p - 2;
            x = x + p;
            b = -1378.25152569120859100;
            b = -38801.6315134637840924 + x*b;
            b = -331612.992738871184744 + x*b;
            b = -1162370.97492762307383 + x*b;
            b = -1721737.00820839662146 + x*b;
            b = -853555.664245765465627 + x*b;
            c = 1;
            c = -351.815701436523470549 + x*c;
            c = -17064.2106651881159223 + x*c;
            c = -220528.590553854454839 + x*c;
            c = -1139334.44367982507207 + x*c;
            c = -2532523.07177582951285 + x*c;
            c = -2018891.41433532773231 + x*c;
            p = x*b / c;
            result = log(z) + p;
            return result;
        }
        q = (x - 0.5)*log(x) - x + ls2pi;
        if ((x > 100000000))
        {
            result = q;
            return result;
        }
        p = 1 / (x*x);
        if ((x >= 1000.0))
        {
            q = q + ((7.9365079365079365079365*0.0001*p - 2.7777777777777777777778*0.001)*p + 0.0833333333333333333333) / x;
        }
        else
        {
            a = 8.11614167470508450300*0.0001;
            a = -5.95061904284301438324*0.0001 + p*a;
            a = 7.93650340457716943945*0.0001 + p*a;
            a = -2.77777777730099687205*0.001 + p*a;
            a = 8.33333333333331927722*0.01 + p*a;
            q = q + a / x;
        }
        result = q;
        return result;
#else
        return _ialglib_i_lngamma(x, sgngam);
#endif
    }


};

class matrix3D          // personnal class of 3D matrix (cubic)
    // X is the speaker
    // Y is the addressee
    // Z is the agent
{
protected:
    std::vector<int>     viData;         // data of the matrix


public:

    // Variables : 
    int             iSize;          // size of each size of the 3D matrix (cubic)
    int             iSum;
    std::vector<std::string>  vsLabels;       // label associated to each col/row/deepth

    //Constructor 
    matrix3D() { iSize = 0; iSum = 0; }

    // Functions
    int oneCoord(int x, int y, int z) { return (x + y*iSize + z*iSize*iSize); }       // return the 1D coordinate from a 3D coordinate    

    int get(int x, int y, int z) { return viData[oneCoord(x, y, z)]; }            // get the x y z position in the matrix

    int get(std::string sSpeaker, std::string sAddressee, std::string sAgent)
    {
        addLabel(sSpeaker);
        addLabel(sAddressee);
        addLabel(sAgent);

        int X = -1,
            Y = -1,
            Z = -1;

        for (int i = 0; i < iSize; i++)
        {
            if (vsLabels[i] == sSpeaker) X = i;
            if (vsLabels[i] == sAddressee) Y = i;
            if (vsLabels[i] == sAgent) Z = i;
        }

        // check 
        if (X == -1 || Y == -1 || Z == -1)
        {
            //yInfo() << "\t" << std::endl << "Error in abmReasoning::abmReasoningFunction.h::matrix3D::get(std::string, std::string, std::string) | One of the label is missing" << std::endl;
            return 0;
        }

        return viData[oneCoord(X, Y, Z)];
    }           // get the x y z position in the matrix

    void incr(int x, int y, int z) { viData[oneCoord(x, y, z)]++; }               // increment the x y z position of the matrix of 1

    void addLabel(std::string sLabel)    // add 1 to the x y and z size, and add the label to the list
    {
        //check if label already in the matrix

        bool bFound = false;
        for (std::vector<std::string>::iterator itS = vsLabels.begin(); itS != vsLabels.end(); itS++)
        {
            if (*itS == sLabel)     bFound = true;
        }

        if (bFound)
        {
            //yInfo() << "\t" << std::endl << "Error in abmReasoning::abmReasoningFunction.h::matrix3D::addLabel | Label already existing" << std::endl;
            return;
        }


        vsLabels.push_back(sLabel);

        std::vector<int> matTemp;
        for (int k = 0; k < iSize + 1; k++)
        {
            for (int j = 0; j < iSize + 1; j++)
            {
                for (int i = 0; i < iSize + 1; i++)
                {

                    if (i == iSize || j == iSize || k == iSize)
                    {
                        matTemp.push_back(0);
                    }
                    else
                    {
                        matTemp.push_back(get(i, j, k));
                    }
                }
            }
        }
        viData = matTemp;
        iSize++;
    }

    void incr(std::string sSpeaker, std::string sAddressee, std::string sAgent)
        // increment in the matrix for the use of a pronom with information about the sentence
    {
        // first check if the speaker, receiver and agent are known
        addLabel(sSpeaker);
        addLabel(sAddressee);
        addLabel(sAgent);

        int X = -1,
            Y = -1,
            Z = -1;

        for (int i = 0; i < iSize; i++)
        {
            if (vsLabels[i] == sSpeaker) X = i;
            if (vsLabels[i] == sAddressee) Y = i;
            if (vsLabels[i] == sAgent) Z = i;
        }

        // check 
        if (X == -1 || Y == -1 || Z == -1)
        {
            //yInfo() << "\t" << std::endl << "Error in abmReasoning::abmReasoningFunction.h::matrix3D::incr(std::string, std::string, std::string) | One of the label is missing" << std::endl;
            return;
        }

        incr(X, Y, Z);
        iSum++;
    }

    int getSum()    { return iSum; }
    int getSize()   { return iSize; }

    /* get the sum of the diagonal of the correspondant plan (x, y or z) */
    int sumDiagDouble(std::string W)
    {
        int sum = 0;
        if (W != "x" && W != "y" && W != "z")
        {
            yInfo() << "\t" << "Error in abmReasoning::abmReasoningFunction.h::matrix3D::sumDiag(std::string) | wrong coordinate ('x', 'y' or 'z')";
        }
        for (int b = 0; b < iSize; b++)
        {
            for (int a = 0; a < iSize; a++)
            {
                if (W == "x") sum += get(a, b, b);
                if (W == "y") sum += get(b, a, b);
                if (W == "z") sum += get(b, b, a);
            }
        }
        return sum;
    }


    int sumPlan(std::string W, std::string sLabel)    // W is x y or z and Label is the name
    {
        int iLabel = -1;
        for (int i = 0; i < iSize; i++)
        {
            if (vsLabels[i] == sLabel) iLabel = i;
        }

        int sum = 0;
        if (W != "x" && W != "y" && W != "z")
        {
            yInfo() << "\t" << "Error in abmReasoning::abmReasoningFunction.h::matrix3D::sumPlan(std::string) | wrong coordinate ('x', 'y' or 'z')";
        }
        for (int a = 0; a < iSize; a++)
        {
            for (int b = 0; b < iSize; b++)
            {
                if (W == "x") sum += get(iLabel, a, b);
                if (W == "y") sum += get(a, iLabel, b);
                if (W == "z") sum += get(a, b, iLabel);
            }
        }
        return sum;
    }

    /* For a given X and Y, return the sum of the Z line*/
    int sumLineXY(std::string X, std::string Y)
    {
        int xLabel = -1,
            yLabel = -1;
        for (int i = 0; i < iSize; i++)
        {
            if (vsLabels[i] == X) xLabel = i;
            if (vsLabels[i] == Y) yLabel = i;
        }

        int sum = 0;
        for (int a = 0; a < iSize; a++)
        {
            sum += get(xLabel, yLabel, a);
        }
        return sum;
    }

    /* For a given X and Z, return the sum of the Y line*/
    int sumLineXZ(std::string X, std::string Z)
    {
        int xLabel = -1,
            zLabel = -1;
        for (int i = 0; i < iSize; i++)
        {
            if (vsLabels[i] == X) xLabel = i;
            if (vsLabels[i] == Z) zLabel = i;
        }

        int sum = 0;
        for (int a = 0; a < iSize; a++)
        {
            sum += get(xLabel, a, zLabel);
        }
        return sum;
    }

    /* For a given Y and Z, return the sum of the X line*/
    int sumLineYZ(std::string Y, std::string Z)
    {
        int zLabel = -1,
            yLabel = -1;
        for (int i = 0; i < iSize; i++)
        {
            if (vsLabels[i] == Z) zLabel = i;
            if (vsLabels[i] == Y) yLabel = i;
        }

        int sum = 0;
        for (int a = 0; a < iSize; a++)
        {
            sum += get(a, yLabel, zLabel);
        }
        return sum;
    }
};



//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------      MATRIX 3D NON CUBIC     -----------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------


class matrix3D_nonCubic          // personnal class of 3D matrix
{
public:
    std::vector<int>         viData;         // data of the matrix
    std::vector<std::pair<std::string, int> >      vLabelX;
    std::vector<std::pair<std::string, int> >      vLabelY;
    std::vector<std::pair<std::string, int> >      vLabelZ;

    // Variables : 
    int             iSum;

    //Constructor 
    matrix3D_nonCubic() { iSum = 0; }

    // Functions
    int oneCoord(int x, int y, int z) {
        //        int test = x+y*vLabelX.size()+z*vLabelX.size()*vLabelY.size();
        //        yInfo() << "\t" << "ici " << test << std::endl;
        return (x + y*vLabelX.size() + z*vLabelX.size()*vLabelY.size());
    }       // return the 1D coordinate from a 3D coordinate    

    int get(int x, int y, int z) { return viData[oneCoord(x, y, z)]; }            // get the x y z position in the matrix

    int get(std::string sX, std::string sY, std::string sZ)
    {
        //        addLabelX(sX, true);
        //        addLabelY(sY, true);
        //        addLabelZ(sZ, true);

        int X = -1,
            Y = -1,
            Z = -1;

        for (unsigned int i = 0; i < vLabelX.size(); i++)
        {
            if (vLabelX[i].first == sX) X = i;
        }

        for (unsigned int i = 0; i < vLabelY.size(); i++)
        {
            if (vLabelY[i].first == sY) Y = i;
        }

        for (unsigned int i = 0; i < vLabelZ.size(); i++)
        {
            if (vLabelZ[i].first == sZ) Z = i;
        }

        // check 
        if (X == -1 || Y == -1 || Z == -1)
        {
            //yInfo() << "\t" << std::endl << "Error in abmReasoning::abmReasoning.h::matrix3D::get(std::string, std::string, std::string) | One of the label is missing" << std::endl;
            return 0;
        }

        return viData[oneCoord(X, Y, Z)];
    }           // get the x y z position in the matrix


    void incr(int x, int y, int z) { viData[oneCoord(x, y, z)]++; }               // increment the x y z position of the matrix of 1

    bool addLabelX(std::string sLabel, bool check)    // add 1 to the x size, and add the label to the list; return FALSE if already existing
    {

        //check if label already in the matrix
        for (std::vector<std::pair<std::string, int> >::iterator it = vLabelX.begin(); it != vLabelX.end(); it++)
        {
            if (sLabel == it->first)
            {
                if (!check) it->second++;
                return false;
            }
        }

        std::vector<int>     matrixTemp;

        for (unsigned int i = 0; i < vLabelZ.size(); i++)
        {

            for (unsigned int j = 0; j < vLabelY.size(); j++)
            {

                for (unsigned int k = 0; k < vLabelX.size(); k++)
                {
                    matrixTemp.push_back(get(k, j, i));
                }
                matrixTemp.push_back(0);
            }
        }

        std::pair<std::string, int> pTemp(sLabel, check ? 0 : 1);
        vLabelX.push_back(pTemp);
        viData = matrixTemp;
        return true;
    }

    bool addLabelY(std::string sLabel, bool check)    // add 1 to the y size, and add the label to the list; return FALSE if already existing
    {

        //check if label already in the matrix
        for (std::vector<std::pair<std::string, int> >::iterator it = vLabelY.begin(); it != vLabelY.end(); it++)
        {
            if (sLabel == it->first)
            {
                if (!check)    it->second++;
                return false;
            }
        }

        std::vector<int>     matrixTemp;

        for (unsigned int i = 0; i < vLabelZ.size(); i++)
        {
            for (unsigned int j = 0; j < vLabelY.size(); j++)
            {
                for (unsigned int k = 0; k < vLabelX.size(); k++)
                {
                    matrixTemp.push_back(get(k, j, i));
                }
            }
            for (unsigned int k = 0; k < vLabelX.size(); k++)
            {
                matrixTemp.push_back(0);
            }
        }


        std::pair<std::string, int> pTemp(sLabel, check ? 0 : 1);
        vLabelY.push_back(pTemp);
        viData = matrixTemp;
        return true;
    }

    bool addLabelZ(std::string sLabel, bool check)    // add 1 to the z size, and add the label to the list; return FALSE if already existing
    {

        //check if label already in the matrix
        for (std::vector<std::pair<std::string, int> >::iterator it = vLabelZ.begin(); it != vLabelZ.end(); it++)
        {
            if (sLabel == it->first)
            {
                if (!check)    it->second++;
                return false;
            }
        }

        std::vector<int>     matrixTemp;


        for (unsigned int j = 0; j < vLabelY.size(); j++)
        {
            for (unsigned int k = 0; k < vLabelX.size(); k++)
            {
                viData.push_back(0);
            }
        }


        std::pair<std::string, int> pTemp(sLabel, check ? 0 : 1);
        vLabelZ.push_back(pTemp);
        //viData = matrixTemp;
        return true;
    }

    void incr(std::string sX, std::vector<std::string> vY, std::string sZ)
        // increment in the matrix for the use of a pronom with information about the sentence
    {
        // first check if the speaker, receiver and agent are known
        addLabelX(sX, false);
        addLabelZ(sZ, false);

        int iX = -1,
            iY = -1,
            iZ = -1;

        // search for X
        for (unsigned int i = 0; i < vLabelX.size(); i++)
        {
            if (vLabelX[i].first == sX)
            {
                iX = i;
                //vLabelX[i].second++;
            }
        }


        // search for Z
        for (unsigned int i = 0; i < vLabelZ.size(); i++)
        {
            if (vLabelZ[i].first == sZ)
            {
                iZ = i;
                //vLabelZ[i].second++;
            }
        }

        // check 
        if (iX == -1 || iZ == -1)
        {
            yInfo() << "\t" << "Error in abmReasoning::abmReasoningFunction.h::matrix3D_nonCubic::incr(std::string, std::vector<std::string>, std::string) | One of the label is missing";
            return;
        }


        // search for Y
        for (std::vector<std::string>::iterator itYinput = vY.begin(); itYinput != vY.end(); itYinput++)
        {
            addLabelY(*itYinput, false);
            for (unsigned int i = 0; i < vLabelY.size(); i++)
            {
                if (vLabelY[i].first == *itYinput)
                {
                    iY = i;
                    vLabelY[i].second++;
                }
            }
            incr(iX, iY, iZ);
            iSum++;
        }
    }

    /* For a given X and Y, return the sum of the Z line*/
    int sumLineXY(std::string sX, std::string sY)
    {
        int iX = -1,
            iY = -1;

        // search for X
        for (unsigned int i = 0; i < vLabelX.size(); i++)
        {
            if (vLabelX[i].first == sX) iX = i;
        }

        // search for Y
        for (unsigned int i = 0; i < vLabelY.size(); i++)
        {
            if (vLabelY[i].first == sY) iY = i;
        }

        int sum = 0;
        for (unsigned int a = 0; a < vLabelZ.size(); a++)
        {
            sum += get(iX, iY, a);
        }
        return sum;
    }

    /* For a given X and Y, return the sum of the Z line*/
    int sumLineYZ(std::string sY, std::string sZ)
    {
        int iY = -1,
            iZ = -1;

        // search for X
        for (unsigned int i = 0; i < vLabelZ.size(); i++)
        {
            if (vLabelZ[i].first == sZ) iZ = i;
        }

        // search for Y
        for (unsigned int i = 0; i < vLabelY.size(); i++)
        {
            if (vLabelY[i].first == sY) iY = i;
        }

        int sum = 0;
        for (unsigned int a = 0; a < vLabelX.size(); a++)
        {
            sum += get(a, iY, iZ);
        }
        return sum;
    }

    /* For a given X and Y, return the sum of the Z line*/
    int sumLineXZ(std::string sX, std::string sZ)
    {
        int iX = -1,
            iZ = -1;

        // search for X
        for (unsigned int i = 0; i < vLabelX.size(); i++)
        {
            if (vLabelX[i].first == sX) iX = i;
        }

        // search for Y
        for (unsigned int i = 0; i < vLabelZ.size(); i++)
        {
            if (vLabelZ[i].first == sZ) iZ = i;
        }

        int sum = 0;
        for (unsigned int a = 0; a < vLabelY.size(); a++)
        {
            sum += get(iX, a, iZ);
        }
        return sum;
    }
};


/* SCORE PROP */

class scoreProp
{

public:
    int A;
    int B;
    int C;
    int D;

    /*------------------

    |-----------|-----------|
    |    A      |   B       |
    |-----------|-----------|
    |    C      |   D       |
    |-----------|-----------|

    */

    double chiSquare()
    {

        /*
        input format :

        |   pop1    |   pop2    |
        ------------|-----------|-----------|
        properties  |    A      |   B       |
        ------------|-----------|-----------|
        ~properties |    C      |   D       |
        ------------|-----------|-----------|


        (N*(A*D - B*C)*(A*D - B*C))
        CHI2 = -----------------------------
        (A+B)*(C+D)*(A+C)*(B+D)

        */


        double dPValue;
        int N = A + B + C + D;
        double chi2 = (N*(A*D - B*C)*(A*D - B*C)) / (1.*(A + B)*(C + D)*(A + C)*(B + D));

        dPValue = pvalue(chi2);

        return dPValue;
    }

    double pvalue(double X) {
        /*--------------------------------------------------------
        Adapted From:
        Hill, I. D. and Pike, M. C. Algorithm 299
        Collected Algorithms for the CACM 1967 p. 243
        Updated for rounding errors based on remark in
        ACM TOMS June 1985, page 185
        ---------------------------------------------------------*/

        if (X <= 0.0)  return 1.0;

        return (2.0 * poz(-sqrt(X)));
    }

    double poz(double z) {
        /*-------------------------------------------------------------------------
        POZ  --  probability of standard normal z value

        Adapted from a polynomial approximation in:
        Ibbetson D, Algorithm 209
        Collected Algorithms of the CACM 1963 p. 616

        Note:
        This routine has six digits accuracy, so it is only useful for absolute
        z values < 6.  For z values >=  6.0, poz() returns 0.0.
        --------------------------------------------------------------------------*/
        double Y, X, w, zmax;
        zmax = 6.0;
        if (z == 0.0)
            X = 0.0;
        else {
            Y = 0.5 * fabs(z);
            if (Y >= zmax * 0.5)
                X = 1.0;
            else if (Y < 1.0) {
                w = Y * Y;
                X = ((((((((0.000124818987*w - 0.001075204047)*w + 0.005198775019)*w - 0.019198292004)*w + 0.059054035642)*w
                    - 0.151968751364)*w + 0.319152932694) * w - 0.5319230073)*w + 0.7978845605929999)*Y * 2;
            }
            else {
                Y = Y - 2;
                X = (((((((((((((-0.000045255659*Y + 0.00015252929)*Y - 0.000019538132)*Y - 6.769049860000001E-04)*Y
                    + 0.001390604284)*Y - 0.00079462082)*Y - 0.002034254874)*Y + 0.006549791214)*Y - 0.010557625006)*Y
                    + 0.011630447319)*Y - 9.279453341000001E-03)*Y + 0.005353579108)*Y - 0.002141268741)*Y
                    + 0.000535310849)*Y + 0.999936657524;
            }
        }
        if (z > 0.0)
            return ((X + 1) * 0.5);
        else
            return ((1 - X) * 0.5);

    }

    double  getScore()
    {
        return ((1.*abmReasoningFunction::SIGMA_LEARNING_GRAMMAR + A) / (B + C + D + abmReasoningFunction::SIGMA_LEARNING_GRAMMAR));
    }

    // if !bPositive multiplie the result by -1
    double  getScoreSum(bool bPositive = true)
    {
        if (A < 0 || B < 0 || C < 0 || D < 0)    return 0.;
        if (B + C + D == 0 && A != 0)    return 1.;
        if (A + B == 0 || A + C == 0 || C + D == 0)   return 0.;
        if (B + D == 0)
        {
            return (2 * A / (A + C*1.) - 1);
        }
        if (chiSquare() > abmReasoningFunction::THRESHOLD_CONFIDENCE_GRAMMAR)   return 0.;

        double dFactor;
        bPositive ? dFactor = 1. : dFactor = -1.;
        int iDistrib = 1;
        if ((1.*abmReasoningFunction::SIGMA_LEARNING_GRAMMAR + A) / (C + 1.*abmReasoningFunction::SIGMA_LEARNING_GRAMMAR) - (1.*abmReasoningFunction::SIGMA_LEARNING_GRAMMAR + B) / (D + 1.*abmReasoningFunction::SIGMA_LEARNING_GRAMMAR) < 0) iDistrib = -1;

        return (dFactor*iDistrib*(1 - chiSquare()));
    }

    scoreProp()
    {
        A = 0;
        B = 0;
        C = 0;
        D = 0;
    }

    scoreProp(int a, int b, int c, int d)
    {
        A = a;
        B = b;
        C = c;
        D = d;
    }

    void plus(scoreProp score2)
    {
        A += score2.A;
        B += score2.B;
        C += score2.C;
        D += score2.D;
    }

    void addScore(std::pair<int, int> pInput, bool bFirstCol)
    {
        if (bFirstCol)  {
            A += pInput.first;
            C += pInput.second;
        }
        else    {
            B += pInput.first;
            D += pInput.second;
        }
    }


};





#endif

