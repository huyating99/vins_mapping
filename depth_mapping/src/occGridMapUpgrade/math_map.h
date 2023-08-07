//
// Created by lao-tang on 2023/3/27.
//

#ifndef BETA_MATH_H
#define BETA_MATH_H

#include <cstdint>
#include <iostream>
#include <cmath>

namespace mapping{
    // Clamps 'value' to be in the range ['min', 'max'].
    using int8 = int8_t;
    using int16 = int16_t;
    using int32 = int32_t;
    using int64 = int64_t;
    using uint8 = uint8_t;
    using uint16 = uint16_t;
    using uint32 = uint32_t;
    using uint64 = uint64_t;
    // std::lround(x)表示取最接近的整数
    inline int RoundToInt(const float x) { return std::lround(x); }

    inline int RoundToInt(const double x) { return std::lround(x); }

    inline int64_t RoundToInt64(const float x) { return std::lround(x); }

    inline int64_t RoundToInt64(const double x) { return std::lround(x); }

    template<typename T ,typename Q>
    void ICHECK_LT(T a,Q b){
        if(a>=b){
            std::cout<<"CHECK_LT error"<<std::endl;
            throw ;
        }
    }

    template<typename T,typename Q>
    void ICHECK_LE(T a,Q b){
        if(a>b){
            std::cout<<"CHECK_LE error"<<std::endl;
            throw ;
        }
    }
    template<typename T,typename Q>
    void ICHECK_GT(T a,Q b){
        if(a<= b){
            std::cout<<"CHECK_GT"<<std::endl;
            throw ;
        }
    }


    inline void ICHECK(bool condition){
        if(condition==false){
            std::cout<<"condition=false"<<std::endl;
            throw;
        }
    }

    template<typename T,typename Q>
    void ICHECK_GE(T a,Q b){
        if(a< b){
            std::cout<<"CHECK_Ge"<<std::endl;
            throw ;
        }
    }

    template<typename T,typename Q>
    void ICHECK_EQ(T a,Q b){
        if(a!= b){
            std::cout<<"DCHECK_EQ"<<std::endl;
            throw ;
        }
    }

    template<typename T,typename Q>
    void ICHECK_NE(T a,Q b){
        if(a == b){
            std::cout<<"DCHECK_NE"<<std::endl;
            throw ;
        }
    }

    template <typename T>
    T Clamp(const T value, const T min, const T max) {
        if (value > max) {
            return max;
        }
        if (value < min) {
            return min;
        }
        return value;
    }

// Calculates 'base'^'exponent'.
    template <typename T>
    constexpr T Power(T base, int exponent) {
        return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
    }

// Calculates a^2.
    template <typename T>
    constexpr T Pow2(T a) {
        return Power(a, 2);
    }

// Converts from degrees to radians.
    constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
    constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }

// Bring the 'difference' between two angles into [-pi; pi].
    template <typename T>
    T NormalizeAngleDifference(T difference) {
        const T kPi = T(M_PI);
        while (difference > kPi) difference -= 2. * kPi;
        while (difference < -kPi) difference += 2. * kPi;
        return difference;
    }


    template <typename T>
    inline void QuaternionProduct(const double* const z, const T* const w,
                                  T* const zw) {
        zw[0] = z[0] * w[0] - z[1] * w[1] - z[2] * w[2] - z[3] * w[3];
        zw[1] = z[0] * w[1] + z[1] * w[0] + z[2] * w[3] - z[3] * w[2];
        zw[2] = z[0] * w[2] - z[1] * w[3] + z[2] * w[0] + z[3] * w[1];
        zw[3] = z[0] * w[3] + z[1] * w[2] - z[2] * w[1] + z[3] * w[0];
    }
}

#endif //BETA_MATH_H
