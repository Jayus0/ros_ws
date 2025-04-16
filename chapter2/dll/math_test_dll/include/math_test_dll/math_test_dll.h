#ifndef __MATH_TEST_DLL_H__
#define __MATH_TEST_DLL_H__

class Calculator
{
public:
    Calculator();
    ~Calculator();

    template<typename T>
    T mAdd(const T &num1, const T &num2)
    {
        return num1 + num2;
    }
    
};


#endif