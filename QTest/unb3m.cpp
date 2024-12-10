#include "unb3m.h"
// #include <QDebug>
// #include "qmath.h"
#include <Eigen/Eigen>
#include <cmath>

// #include <Eigen/Dense>
using namespace Eigen;
// using namespace Eigen::internal;
// using namespace Eigen::Architecture;
//#define  EXCEN2  6.6943799901413e-03
//#define  MD      28.9644
//#define  MW     18.0152
//#define  K1      77.604
//#define  K2    64.79
//#define  K3     3.776e5
//#define  R      8314.34
//#define  C1    2.2768e-03
//#define  K2PRIM (K2 - K1*(MW/MD))
//#define  RD      (R / MD)
//#define  DTR  1.745329251994329e-02
//#define  DOY2RAD   ((0.31415926535897935601e01)*2/365.25)
//#define  A_HT  2.53e-5
//#define  B_HT 5.49e-3
//#define  C_HT  1.14e-3
//#define  HT_TOPCON  (1 + A_HT/(1 + B_HT/(1 + C_HT)))
#define M_PI 3.141592653590
unb3m::unb3m()
{
}
double unb3m::trop_unb3m(double LATRAD,double HEIGHTM,double DAYOYEAR,double ELEVRAD,double &map)
{
    MatrixXd AVG(5,6);
    AVG=MatrixXd::Zero(5,6);
    AVG<<15.0,  1013.25,  299.65, 75.00,  6.30,  2.77,
               30.0,  1017.25,  294.15, 80.00,  6.05,  3.15,
               45.0,  1015.75,  283.15, 76.00,  5.58,  2.57,
               60.0,  1011.75,  272.15, 77.50,  5.39,  1.81,
               75.0,  1013.00,  263.65, 82.50,  4.53, 1.55;
    MatrixXd AMP(5,6);
    AMP=MatrixXd::Zero(5,6);
    AMP<<15.0,   0.00,   0.00,   0.00,  0.00,  0.00,
               30.0,  -3.75,   7.00,   0.00,  0.25,  0.33,
            45.0,  -2.25,  11.00,  -1.00,  0.32,  0.46,
            60.0,  -1.75,  15.00,  -2.50,  0.81,  0.74,
            75.0,  -0.50,  14.50,   2.50,  0.62,  0.30;
    double  EXCEN2 = 6.6943799901413e-03;
    double   MD     = 28.9644;
    double   MW     = 18.0152;
    double   K1     = 77.604;
    double  K2     = 64.79;
    double  K3     = 3.776e5;
    double   R      = 8314.34;
    double   C1     = 2.2768e-03;
    double  K2PRIM = K2 - K1*(MW/MD);
    double   RD     = R / MD;
    double   DTR = 1.745329251994329e-02;
    double  DOY2RAD=(0.31415926535897935601e01)*2/365.25;

    MatrixXd ABC_AVG(5,4);
    ABC_AVG=MatrixXd::Zero(5,4);
    ABC_AVG<<15.0, 1.2769934e-3, 2.9153695e-3, 62.610505e-3,
                        30.0, 1.2683230e-3, 2.9152299e-3, 62.837393e-3,
                        45.0, 1.2465397e-3, 2.9288445e-3, 63.721774e-3,
                        60.0, 1.2196049e-3, 2.9022565e-3, 63.824265e-3,
                        75.0, 1.2045996e-3, 2.9024912e-3, 64.258455e-3;
    MatrixXd ABC_AMP(5,4);
    ABC_AMP=MatrixXd::Zero(5,4);
    ABC_AMP<<15.0, 0.0,                    0.0,                   0.0,
                         30.0, 1.2709626e-5, 2.1414979e-5, 9.0128400e-5,
                         45.0, 2.6523662e-5, 3.0160779e-5, 4.3497037e-5,
                         60.0, 3.4000452e-5, 7.2562722e-5, 84.795348e-5,
                         75.0, 4.1202191e-5, 11.723375e-5, 170.37206e-5;
    double A_HT = 2.53e-5;
    double B_HT= 5.49e-3;
    double C_HT = 1.14e-3;
    double HT_TOPCON = 1 + A_HT/(1 + B_HT/(1 + C_HT));
    MatrixXd ABC_W2P0(5,4);
    ABC_W2P0=MatrixXd::Zero(5,4);
    ABC_W2P0<<15.0, 5.8021897e-4, 1.4275268e-3, 4.3472961e-2,
                           30.0, 5.6794847e-4, 1.5138625e-3, 4.6729510e-2,
                           45.0, 5.8118019e-4, 1.4572752e-3, 4.3908931e-2,
                           60.0, 5.9727542e-4, 1.5007428e-3, 4.4626982e-2,
                           75.0, 6.1641693e-4, 1.7599082e-3, 5.4736038e-2;
    double LATDEG,TD_O_Y,COSPHS,LAT;
    int P1,P2;
    double M;
    double PAVG,TAVG,EAVG,BETAAVG,LAMBDAAVG;
    double PAMP,TAMP,EAMP,BETAAMP,LAMBDAAMP;
    double P0,T0,E0,BETA,LAMBDA;
    double ES,FW,EP;
    double T,P,E;
    double GEOLAT,DGREF,GM,DEN;
    double TM,HZD,WZD;
    double A_AVG,B_AVG,C_AVG;
    double A_AMP,B_AMP,C_AMP;
    double A,B,C;
    double SINE;
    double ALPHA,GAMMA,TOPCON,HMF;
    double HT_CORR_COEF,HT_CORR;
    double WMF,RTROP;
    LATDEG = LATRAD * 180.0 / M_PI ;
    TD_O_Y = DAYOYEAR;
    if( LATDEG<0)
        TD_O_Y = TD_O_Y + 182.625;
    COSPHS = cos((TD_O_Y - 28) * DOY2RAD);
    LAT = std::abs( LATDEG );
    if (LAT>=75)
    {
        P1 = 5;
        P2 = 5;
        M = 0;
    }
    else if (LAT<=15)
    {
        P1 = 1;
        P2 = 1;
        M = 0;
    }
    else
    {
        P1 = (int)((LAT - 15)/15) + 1;
        P2 = P1 + 1;
        M = (LAT - AVG(P1-1,0) ) / ( AVG(P2-1,0) - AVG(P1-1,0) );
    }
    PAVG = M * ( AVG(P2-1,1) - AVG(P1-1,1) ) + AVG(P1-1,1);
    TAVG = M * ( AVG(P2-1,2) - AVG(P1-1,2) ) + AVG(P1-1,2);
    EAVG = M * ( AVG(P2-1,3) - AVG(P1-1,3) ) + AVG(P1-1,3);
    BETAAVG   = M * ( AVG(P2-1,4) - AVG(P1-1,4) ) + AVG(P1-1,4);
    LAMBDAAVG = M * ( AVG(P2-1,5) - AVG(P1-1,5) ) + AVG(P1-1,5);
    PAMP = M * ( AMP(P2-1,1) - AMP(P1-1,1) ) + AMP(P1-1,1);
    TAMP = M * ( AMP(P2-1,2) - AMP(P1-1,2) ) + AMP(P1-1,2);
    EAMP = M * ( AMP(P2-1,3) - AMP(P1-1,3) ) + AMP(P1-1,3);
    BETAAMP   = M * ( AMP(P2-1,4) - AMP(P1-1,4) ) + AMP(P1-1,4);
    LAMBDAAMP = M * ( AMP(P2-1,5) - AMP(P1-1,5) ) + AMP(P1-1,5);
    P0 = PAVG - PAMP * COSPHS;
    T0 = TAVG - TAMP * COSPHS;
    E0 = EAVG - EAMP * COSPHS;
    BETA = BETAAVG - BETAAMP * COSPHS;
    BETA   = BETA / 1000;
    LAMBDA = LAMBDAAVG - LAMBDAAMP * COSPHS;
    ES = 0.01 * exp(1.2378847e-5 * pow(T0 , 2) - 1.9121316e-2 *T0 + 3.393711047e1 - 6.3431645e3 * pow(T0 , -1));
    FW = 1.00062 + 3.14e-6 * P0 + 5.6e-7 * pow((T0 - 273.15) , 2);
    E0 = (E0 / 1.00e2) * ES * FW;
    EP = 9.80665 / 287.054 / BETA;
    T = T0 - BETA * HEIGHTM;
    P = P0 * pow(( T / T0 ) , EP);
    E = E0 * pow(( T / T0 ) , ( EP * (LAMBDA+1) ));
    GEOLAT = atan((1.0-EXCEN2)*tan(LATRAD));
    DGREF = 1.0 - 2.66e-03*cos(2.0*GEOLAT) - 2.8e-07*HEIGHTM;
    GM    = 9.784 * DGREF;
    DEN   = ( LAMBDA + 1.0 ) * GM;
    TM  = T * (1 - BETA * RD / DEN);
    HZD = C1 / DGREF * P;
    WZD = 1.0e-6 * ( K2PRIM + K3/TM) * RD * E/DEN;
    A_AVG = M * ( ABC_AVG(P2-1,1) - ABC_AVG(P1-1,1) ) + ABC_AVG(P1-1,1);
    B_AVG = M * ( ABC_AVG(P2-1,2) - ABC_AVG(P1-1,2) ) + ABC_AVG(P1-1,2);
    C_AVG = M * ( ABC_AVG(P2-1,3) - ABC_AVG(P1-1,3) ) + ABC_AVG(P1-1,3);
    A_AMP = M * ( ABC_AMP(P2-1,1) - ABC_AMP(P1-1,1) ) + ABC_AMP(P1-1,1);
    B_AMP = M * ( ABC_AMP(P2-1,2) - ABC_AMP(P1-1,2) ) + ABC_AMP(P1-1,2);
    C_AMP = M * ( ABC_AMP(P2-1,3) - ABC_AMP(P1-1,3) ) + ABC_AMP(P1-1,3);
    A = A_AVG - A_AMP * COSPHS;
    B = B_AVG - B_AMP * COSPHS;
    C = C_AVG - C_AMP * COSPHS;
    SINE = sin(ELEVRAD);
    ALPHA  = B/(SINE + C );
    GAMMA  = A/(SINE + ALPHA);
    TOPCON = (1 + A/(1 + B/(1 + C)));
    HMF    = TOPCON / ( SINE + GAMMA );
    ALPHA  = B_HT/( SINE + C_HT );
    GAMMA  = A_HT/( SINE + ALPHA);
    HT_CORR_COEF = 1/SINE - HT_TOPCON/(SINE + GAMMA);
    HT_CORR      = HT_CORR_COEF * HEIGHTM / 1000;
    HMF          = HMF + HT_CORR;
    A = M * ( ABC_W2P0(P2-1,1) - ABC_W2P0(P1-1,1) ) + ABC_W2P0(P1-1,1);
    B = M * ( ABC_W2P0(P2-1,2) - ABC_W2P0(P1-1,2) ) + ABC_W2P0(P1-1,2);
    C = M * ( ABC_W2P0(P2-1,3) - ABC_W2P0(P1-1,3) ) + ABC_W2P0(P1-1,3);
    ALPHA = B/( SINE + C );
    GAMMA = A/( SINE + ALPHA);
    TOPCON = (1 + A/(1 + B/(1 + C)));
    WMF    = TOPCON / ( SINE + GAMMA );
    RTROP=HZD*HMF+WZD*WMF;//Double difference ZWD as unknowns
 //   RTROP=HZD*HMF;
    map=WMF;
    return RTROP;

}
