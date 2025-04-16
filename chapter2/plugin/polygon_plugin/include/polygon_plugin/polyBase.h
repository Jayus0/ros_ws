#ifndef __POLYBASE_H__
#define __POLYBASE_H__

namespace polyBase
{
    class Polygon
    {
    public:
        virtual void initialize(double sideLength) = 0;
        virtual double calArea() = 0;
        virtual ~Polygon(){}

    protected:
        Polygon(){}
    };
};



#endif