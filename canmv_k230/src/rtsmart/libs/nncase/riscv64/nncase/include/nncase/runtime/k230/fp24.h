/* Copyright 2020 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
#include <bit>
#include <cmath>
#include <compare>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>

namespace nncase
{
struct fp24
{
    fp24()
        : value(ZERO_VALUE) { }

    static fp24 truncate_to_fp24(const float v)
    {
        fp24 output;
        if (float_isnan(v))
        {
            output.value = NAN_VALUE;
            return output;
        }

        FP32 f;
        f.f = v;

        const uint32_t *p = reinterpret_cast<const uint32_t *>(&f.u);
        output.value = ((*p) >> 8);
        return output;
    }

    explicit fp24(const float v)
    {
        value = round_to_fp24(v).value;
    }

    explicit fp24(const double val)
        : fp24(static_cast<float>(val)) { }

    explicit fp24(const unsigned short val)
        : fp24(static_cast<float>(val)) { }

    explicit fp24(const unsigned int val)
        : fp24(static_cast<float>(val)) { }

    explicit fp24(const int val)
        : fp24(static_cast<float>(val)) { }

    explicit fp24(const long val)
        : fp24(static_cast<float>(val)) { }

    explicit fp24(const long long val)
        : fp24(static_cast<float>(val)) { }

    template <class T>
    explicit fp24(const T &val)
        : fp24(static_cast<float>(val)) { }

    explicit operator float() const
    {
        FP32 result;
        result.f = 0;

        uint32_t *q = reinterpret_cast<uint32_t *>(&result.u);

        *q = value << 8;

        return result.f;
    }

    explicit operator bool() const
    {
        return static_cast<bool>(float(*this));
    }

    explicit operator short() const
    {
        return static_cast<short>(float(*this));
    }

    explicit operator int() const
    {
        return static_cast<int>(float(*this));
    }

    explicit operator long() const
    {
        return static_cast<long>(float(*this));
    }

    explicit operator char() const
    {
        return static_cast<char>(float(*this));
    }

    explicit operator signed char() const
    {
        return static_cast<signed char>(float(*this));
    }

    explicit operator unsigned char() const
    {
        return static_cast<unsigned char>(float(*this));
    }

    explicit operator unsigned short() const
    {
        return static_cast<unsigned short>(float(*this));
    }

    explicit operator unsigned int() const
    {
        return static_cast<unsigned int>(float(*this));
    }

    explicit operator unsigned long() const
    {
        return static_cast<unsigned long>(float(*this));
    }

    explicit operator unsigned long long() const
    {
        return static_cast<unsigned long long>(float(*this));
    }

    explicit operator long long() const
    {
        return static_cast<long long>(float(*this));
    }

    explicit operator double() const
    {
        return static_cast<double>(float(*this));
    }

    union FP32
    {
        unsigned int u;
        float f;
    };

    // Converts a float point to fp24, with round-nearest-to-even as rounding
    // method.
    static fp24 round_to_fp24(float v)
    {
        uint32_t input;
        FP32 f;
        f.f = v;
        input = f.u;
        fp24 output;

        if (float_isnan(v))
        {
            // If the value is a NaN, squash it to a qNaN with msb of fraction set,
            // this makes sure after truncation we don't end up with an inf.
            //
            // qNaN magic: All exponent bits set + most significant bit of fraction
            // set.
            output.value = 0x7fc000;
        }
        else
        {
            // Least significant bit of resulting bfloat.
            uint32_t lsb = (input >> 8) & 1;
            uint32_t rounding_bias = 0x7f + lsb;
            input += rounding_bias;
            output.value = static_cast<uint32_t>(input >> 8);
        }

        return output;
    }

    static fp24 epsilon()
    {
        fp24 x;
        x.value = 0x3c0000; // 0x1.0p-7
        return x;
    }

    static fp24 highest()
    {
        fp24 x;
        x.value = 0x7F7FFF; // 0x1.FFFEp127
        return x;
    }

    static fp24 lowest()
    {
        fp24 x;
        x.value = 0xFF7FFF; // -0x1.FFFEp127
        return x;
    }

    static fp24 min_positive_normal()
    {
        fp24 x;
        x.value = 0x008000; // 0x1p-126
        return x;
    }

    bool IsZero() const { return (value & 0x7FFFFF) == ZERO_VALUE; }

    uint32_t value;

    // A value that represents "not a number".
    static const uint32_t NAN_VALUE = 0x7FC000;

private:
    // A value that represents "zero".
    static const uint32_t ZERO_VALUE = 0;

    static bool float_isnan(const float &x)
    {
        return std::isnan(x);
    }
};

inline std::ostream &operator<<(std::ostream &os, const fp24 &dt)
{
    os << static_cast<float>(dt);
    return os;
}

inline int32_t component(uint32_t v, uint8_t s)
{
    return (1 - 2 * s) * (int32_t)v;
}

inline fp24 operator+(fp24 a, fp24 b)
{
    return fp24(static_cast<float>(a) + static_cast<float>(b));
    // return AddTwoFp24(a, b);
    // return AddTwoFp24Simp(a, b);
}
inline fp24 operator+(fp24 a, int b)
{
    return fp24(static_cast<float>(a) + static_cast<float>(b));
}
inline fp24 operator+(int a, fp24 b)
{
    return fp24(static_cast<float>(a) + static_cast<float>(b));
}

inline fp24 operator-(fp24 a, fp24 b)
{
    return fp24(static_cast<float>(a) - static_cast<float>(b));
}

inline fp24 operator*(fp24 a, fp24 b)
{
    return fp24(static_cast<float>(a) * static_cast<float>(b));
}

inline fp24 operator/(fp24 a, fp24 b)
{
    return fp24(static_cast<float>(a) / static_cast<float>(b));
}

inline fp24 operator-(fp24 a)
{
    a.value ^= 0x800000;
    return a;
}

inline bool operator<(fp24 a, fp24 b)
{
    return static_cast<float>(a) < static_cast<float>(b);
}

inline bool operator<=(fp24 a, fp24 b)
{
    return static_cast<float>(a) <= static_cast<float>(b);
}

inline bool operator==(fp24 a, fp24 b)
{
    return static_cast<float>(a) == static_cast<float>(b);
}

inline bool operator!=(fp24 a, fp24 b)
{
    return static_cast<float>(a) != static_cast<float>(b);
}

inline bool operator>(fp24 a, fp24 b)
{
    return static_cast<float>(a) > static_cast<float>(b);
}

inline bool operator>=(fp24 a, fp24 b)
{
    return static_cast<float>(a) >= static_cast<float>(b);
}

inline fp24 &operator+=(fp24 &a, fp24 b)
{
    a = a + b;
    return a;
}

inline fp24 &operator-=(fp24 &a, fp24 b)
{
    a = a - b;
    return a;
}

inline fp24 operator++(fp24 &a)
{
    a += fp24(1);
    return a;
}

inline fp24 operator--(fp24 &a)
{
    a -= fp24(1);
    return a;
}

inline fp24 operator++(fp24 &a, int)
{
    fp24 original_value = a;
    ++a;
    return original_value;
}

inline fp24 operator--(fp24 &a, int)
{
    fp24 original_value = a;
    --a;
    return original_value;
}

inline fp24 &operator*=(fp24 &a, fp24 b)
{
    a = a * b;
    return a;
}

inline fp24 &operator/=(fp24 &a, fp24 b)
{
    a = a / b;
    return a;
}
} // namespace nncase

namespace std
{
template <>
struct hash<nncase::fp24>
{
    size_t operator()(const nncase::fp24 &v) const
    {
        return hash<float>()(static_cast<float>(v));
    }
};

using nncase::fp24;
inline bool isinf(const fp24 &a) { return std::isinf(float(a)); }
inline bool isnan(const fp24 &a) { return std::isnan(float(a)); }
inline bool isfinite(const fp24 &a) { return std::isfinite(float(a)); }
inline fp24 abs(const fp24 &a) { return fp24(std::abs(float(a))); }
inline fp24 exp(const fp24 &a) { return fp24(std::exp(float(a))); }
inline fp24 log(const fp24 &a) { return fp24(std::log(float(a))); }
inline fp24 log10(const fp24 &a)
{
    return fp24(std::log10(float(a)));
}
inline fp24 sqrt(const fp24 &a)
{
    return fp24(std::sqrt(float(a)));
}
inline fp24 pow(const fp24 &a, const fp24 &b)
{
    return fp24(std::pow(float(a), float(b)));
}
inline fp24 sin(const fp24 &a) { return fp24(std::sin(float(a))); }
inline fp24 cos(const fp24 &a) { return fp24(std::cos(float(a))); }
inline fp24 tan(const fp24 &a) { return fp24(std::tan(float(a))); }
inline fp24 tanh(const fp24 &a)
{
    return fp24(std::tanh(float(a)));
}
inline fp24 floor(const fp24 &a)
{
    return fp24(std::floor(float(a)));
}
inline fp24 ceil(const fp24 &a)
{
    return fp24(std::ceil(float(a)));
}
inline fp24 round(const fp24 &a)
{
    // return fp24(std::round(float(a)));
    return fp24(std::nearbyint(float(a)));
}
} // namespace std
