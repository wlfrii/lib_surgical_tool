#ifndef LIB_SURGICAL_TOOL_PARAM_H_LF
#define LIB_SURGICAL_TOOL_PARAM_H_LF
#include <cstdio>


class SurgicalToolParam
{
public:
    SurgicalToolParam(float L1 = 0, float Lr = 0, float L2 = 0, float Lg = 0,
                    float r = 0, float gamma3 = 0)
        : _L1(L1)
        , _Lr(Lr)
        , _L2(L2)
        , _Lg(Lg)
        , _radius(r)
        , _gamma3(gamma3)
    {}

    SurgicalToolParam& operator= (const SurgicalToolParam &param)
    {
        _L1 = param._L1;
        _Lr = param._Lr;
        _L2 = param._L2;
        _Lg = param._Lg;
        _radius = param._radius;
        _gamma3 = param._gamma3;

        return *this;
    }

    float getL1()       const	{   return _L1;	    }

    float getLr()       const	{   return _Lr;	    }

    float getL2()       const	{   return _L2;	    }

    float getLg()       const	{   return _Lg;	    }

    float getRadius()   const	{   return _radius;    }

    float getGamma3()   const	{   return _gamma3;    }

    bool isEmpty() const {
        float zero = 1e-2;
        return _L1 < zero && _L1 > -zero && 
               _Lr < zero && _Lr > -zero &&
               _L2 < zero && _L2 > -zero;
    }

    char* info() const {
        static char info[128];
        sprintf(info, "L1:%f,Lr:%f,L2:%f,Lg:%f,radius:%f,gamma3:%f",
                _L1, _Lr, _L2, _Lg, _radius, _gamma3);
        return info;
    }

private:
    float _L1;		// would also used to represent LS1
    float _Lr;
    float _L2;
    float _Lg;
    float _radius;
    float _gamma3;
};

#endif // LIB_SURGICAL_TOOL_PARAM_H_LF
