// FOS.h
#ifndef FOS_H
#define FOS_H

class FOS {
public:
    FOS(float tau, float Ts)
    {
        setParams(tau, Ts);
        y1 = 0.0f;
        u1 = 0.0f;
    }

    void setParams(float tau, float Ts)
    {
        float denom = 2.0f * tau + Ts;
        a1 = (2.0f * tau - Ts) / denom;
        b0 = Ts / denom;
        b1 = b0;
    }

    float update(float u)
    {
        float y = a1 * y1 + b0 * u + b1 * u1;
        y1 = y;
        u1 = u;
        return y;
    }

private:
    float a1, b0, b1;
    float y1, u1;
};
#endif // FOS_H