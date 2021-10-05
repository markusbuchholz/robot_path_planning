//g++ cpp_potential_1.cpp -o t -I/usr/include/python3.8 -lpython3.8
#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>
#include <limits>
#include <iomanip>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int sizeX = 50; // size of robot space
int sizeY = 50; // size of robot space
double sX = 5.0; // robot start          
double sY = 1.0; // robot start
double gX = 14.0; // goal x position [m]
double gY = 29.0; // goal y position [m]
double robot_radius = 10.0; // robot radius [m]
double KP = 10.0;
double ETA = 100.0;

std::vector<double> ox{15.0, 5.0, 10.0, 15.0};  //  obstacle x position list [m]
std::vector<double> oy{25.0, 15.0, 18.0, 15.0}; // obstacle y position list [m]
std::tuple<std::vector<double>, std::vector<double>> obs(ox, oy);
std::vector<int> motionX{1, 0, -1, 0, -1, -1, 1, 1};
std::vector<int> motionY{0, 1, 0, -1, -1, 1, -1, 1};


//-------------------------------------------------------------------------------------------
double computeAttractivePotential(double actual_x, double actual_y, double gx, double gy)
{

    return 0.5 * KP * std::sqrt(std::pow((actual_x - gx), 2) + std::pow((actual_y - gy), 2));
}

//-------------------------------------------------------------------------------------------

double computeRepulsivePotential(double actual_x, double actual_y, double rr, std::tuple<std::vector<double>, std::vector<double>> obs)
{

    std::vector<double> ox = std::get<0>(obs);
    std::vector<double> oy = std::get<1>(obs);

    //find closest obstacle

    double minD = std::numeric_limits<double>::max();
    int posID{-100};

    for (int ii = 0; ii < ox.size(); ii++)
    {

        double actualD = std::sqrt(std::pow((actual_x - ox[ii]), 2) + std::pow((actual_y - oy[ii]), 2));

        if (actualD <= minD)
        {

            minD = actualD;
            posID = ii;
        }
    }

    
    //compute repulsive potential

    double dq = std::sqrt(std::pow((actual_x - ox[posID]), 2) + std::pow((actual_y - oy[posID]), 2));

    if (dq <= rr)
    {

        if (dq <= 0.1)
        {
            dq = 0.1;
        }

        return 0.5 * ETA * std::pow((1.0 / dq - 1.0 / rr), 2);
    }

    else
    {

        return 0.0;
    }

    
}

//-------------------------------------------------------------------------------------------

std::vector<std::vector<double>> computePotentialField()
{ 

    std::vector<std::vector<double>> pmap;

    for (int i = 0; i < sizeX; i++)
    {

        std::vector<double> rowInit(sizeX, 0.0);

        pmap.push_back(rowInit);
    }

    for (int ii = 0; ii < sizeX; ii++)
    {

        for (int jj = 0; jj < sizeY; jj++)
        {

            double uf = computeAttractivePotential(ii, jj, gX, gY) + computeRepulsivePotential(ii, jj, robot_radius, obs);
            //std::cout<<ii<<" , " << jj << " : " <<std::setprecision(3) << uf << std::endl;
            pmap[ii][jj] = uf;
        }
    }

    for (auto &ii : pmap)
    {

        for (auto &jj : ii)
        {

            std::cout << std::setprecision(3) << jj << " ";
        }

        std::cout << "" << std::endl;
    }

    return pmap;
}

//-------------------------------------------------------------------------------------------

std::tuple<std::vector<int>, std::vector<int>> pathPlanner(std::vector<std::vector<double>> pmap)
{
    std::vector<int> pathX;
    std::vector<int> pathY;

    double actualD = std::sqrt(std::pow((sX - gX), 2) + std::pow((sY - gY), 2));
    int iX = sX;
    int iY = sY;

    int diX{0};
    int diY{0};

    while (actualD > 1.0)
    {
        double minPotential = std::numeric_limits<double>::max();

        for (int ii = 0; ii < motionX.size(); ii++)
        {

            if ((iX + motionX[ii] >= 0) && (iY + motionY[ii] >= 0) && (iX + motionX[ii] < sizeY) && (iY + motionY[ii] < sizeY))
            {

                if (pmap[iX + motionX[ii]][iY + motionY[ii]] < minPotential)
                {
                    minPotential = pmap[iX + motionX[ii]][iY + motionY[ii]];
                    diX = ii; //iX + motionX[ii];
                    diY = ii; //iY + motionY[jj];
                }
            }
        }

        iX = iX + motionX[diX];
        iY = iY + motionY[diY];
        pathX.push_back(iX);
        pathY.push_back(iY);
        actualD = std::sqrt(std::pow((iX - gX), 2) + std::pow((iY - gY), 2));

        std::cout << actualD << std::endl;
    }

    return std::make_tuple(pathX, pathY);
}

//-------------------------------------------------------------------------------------------

void drawPlot(std::vector<int> xX, std::vector<int> yY)
{

    
    int n = 360;
    int r = 2;
    int a1 = ox[0];
    int b1 = oy[0];
    int a2 = ox[1];
    int b2 = oy[1];
    int a3 = ox[2];
    int b3 = oy[2];
    int a4 = ox[3];
    int b4 = oy[3];

    std::vector<double> xX1;
    std::vector<double> yY1;
    std::vector<double> xX2;
    std::vector<double> yY2;
    std::vector<double> xX3;
    std::vector<double> yY3;
    std::vector<double> xX4;
    std::vector<double> yY4;


    for (int i = 0; i < n; i++)
    {

        xX1.push_back(r * std::cos(2 * M_PI * i / 360.0) + a1);
        yY1.push_back(r * std::sin(2 * M_PI * i / 360.0) + b1);
        
        xX2.push_back(r * std::cos(2 * M_PI * i / 360.0) + a2);
        yY2.push_back(r * std::sin(2 * M_PI * i / 360.0) + b2);
   
        xX3.push_back(r * std::cos(2 * M_PI * i / 360.0) + a3);
        yY3.push_back(r * std::sin(2 * M_PI * i / 360.0) + b3);
   
        xX4.push_back(r * std::cos(2 * M_PI * i / 360.0) + a4);
        yY4.push_back(r * std::sin(2 * M_PI * i / 360.0) + b4);
   
   
   
    }

    plt::figure_size(600, 600);

    plt::plot(xX1, yY1);
    plt::plot(xX2, yY2);
    plt::plot(xX3, yY3);
    plt::plot(xX4, yY4);

    plt::plot(xX, yY);
    //plt::scatter(xX, yY);
    plt::xlabel("xx");
    plt::ylabel("yy");
    plt::xlim(0, 35);
    plt::ylim(0, 35);

    plt::show();
}

//-------------------------------------------------------------------------------------------

void drawCircle()
{

    int n = 360;
    int r = 2;
    int a = 3;
    int b = 5;

    std::vector<double> xX;
    std::vector<double> yY;

    for (int i = 0; i < n; i++)
    {

        xX.push_back(r * std::cos(2 * M_PI * i / 360.0) + a);
        yY.push_back(r * std::sin(2 * M_PI * i / 360.0) + b);
    }

    plt::plot(xX, yY);
    plt::show();
}

//-------------------------------------------------------------------------------------------

int main()
{

    std::vector<std::vector<double>> pmap = computePotentialField();
    std::tuple<std::vector<int>, std::vector<int>> path = pathPlanner(pmap);

    std::vector<int> pathX = std::get<0>(path);
    std::vector<int> pathY = std::get<1>(path);

    std::cout << "---------------------------------------" << std::endl;
    
    for (int ii = 0; ii < pathX.size(); ii++)
    {

        std::cout << pathX[ii] << " : " << pathY[ii] << std::endl;
    }

    drawPlot(pathX, pathY);
}