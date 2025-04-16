#ifndef __POLYPLUGIN_H__
#define __POLYPLUGIN_H__

#include "polyBase.h"
#include <cmath>


namespace polyPlugin
{
    class Triangle : public polyBase::Polygon
    {
    public:
        Triangle(){}

        void initialize(double sideLength)
        {
            this->sideLength = sideLength;
        }

        double calArea()
        {
            return 0.5 * this->sideLength * getHeight();
        }

    private:
        double sideLength;
        double getHeight()
        {
            double temp = sideLength * sideLength;
            return sqrt(temp - 0.25 * temp);
        }
    };


    class Square : public polyBase::Polygon
    {
    public:
        Square(){}

        void initialize(double sideLength)
        {
            this->sideLength = sideLength;
        }

        double calArea()
        {
            return this->sideLength * this->sideLength;
        }
        
    private:
        double sideLength;
    };

};



#endif