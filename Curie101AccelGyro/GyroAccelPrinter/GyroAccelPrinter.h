#pragma once
#include <math.h>

// from Intel CPUT library (CPUTMath.h)
struct float3
{
    union
    {
        struct
        {
            float x;
            float y;
            float z;
        };
        float f[3];
    };

    /***************************************\
    |   Constructors                        |
    \***************************************/
    float3() {}
    explicit float3(float f) : x(f), y(f), z(f) { }
    explicit float3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) { }
    explicit float3(float* f) : x(f[0]), y(f[1]), z(f[2]) { }
    float3(const float3 &v) : x(v.x), y(v.y), z(v.z) { }
    const float3 &operator=(const float3 &v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }

    /***************************************\
    |   Basic math operations               |
    \***************************************/
    float3 operator+(const float3 &r) const
    {
        return float3(x+r.x, y+r.y, z+r.z);
    }
    const float3 &operator+=(const float3 &r)
    {
        x += r.x;
        y += r.y;
        z += r.z;
        return *this;
    }
    float3 operator-(const float3 &r) const
    {
        return float3(x-r.x, y-r.y, z-r.z);
    }
    const float3 &operator-=(const float3 &r)
    {
        x -= r.x;
        y -= r.y;
        z -= r.z;
        return *this;
    }
    float3 operator*(const float3 &r) const
    {
        return float3(x*r.x, y*r.y, z*r.z);
    }
    const float3 &operator*=(const float3 &r)
    {
        x *= r.x;
        y *= r.y;
        z *= r.z;
        return *this;
    }
    float3 operator/(const float3 &r) const
    {
        return float3(x/r.x, y/r.y, z/r.z);
    }
    const float3 &operator/=(const float3 &r)
    {
        x /= r.x;
        y /= r.y;
        z /= r.z;
        return *this;
    }

    /***************************************\
    |   Basic math operations with scalars  |
    \***************************************/
    float3 operator+(float f) const
    {
        return float3(x+f, y+f, z+f);
    }
    const float3 &operator+=(float f)
    {
        x += f;
        y += f;
        z += f;
        return *this;
    }
    float3 operator-(float f) const
    {
        return float3(x-f, y-f, z-f);
    }
    const float3 &operator-=(float f)
    {
        x -= f;
        y -= f;
        z -= f;
        return *this;
    }
    float3 operator*(float f) const
    {
        return float3(x*f, y*f, z*f);
    }
    const float3 &operator*=(float f)
    {
        x *= f;
        y *= f;
        z *= f;
        return *this;
    }
    float3 operator/(float f) const
    {
        return float3(x/f, y/f, z/f);
    }
    const float3 &operator/=(float f)
    {
        x /= f;
        y /= f;
        z /= f;
        return *this;
    }

    /***************************************\
    |   Other math                          |
    \***************************************/
    // Equality
    bool operator==(const float3 &r) const
    {
        return x==r.x &&y == r.y &&z == r.z;
    }
    bool operator!=(const float3 &r) const
    {
        return !(*this == r);
    }

    // Hadd
    float hadd(void) const
    {
        return x + y + z;
    }

    // Length
    float lengthSq(void) const
    {
        return x*x + y*y + z*z;
    }
    float length(void) const
    {
        return sqrtf(lengthSq());
    }
    float3 normalize(void)
    {
        return (*this /= length());
    }
};


void updateValues(int opcode);

void calibrate();

static void eventCallback(void);

bool isZeroMotion(unsigned long time);

float convertRawAcceleration(int aRaw);

float convertRawGyro(int gRaw);