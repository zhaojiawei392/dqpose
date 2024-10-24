/** 
 *     This file is part of dqpose.
 *  
 *     dqpose is free software: you can redistribute it and/or modify 
 *     it under the terms of the GNU General Public License as published 
 *     by the Free Software Foundation, either version 3 of the License, 
 *     or (at your option) any later version.
 *  
 *     dqpose is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *     See the GNU General Public License for more details.
 *  
 *     You should have received a copy of the GNU General Public License
 *     along with dqpose. If not, see <https://www.gnu.org/licenses/>.
 */

/**
 *     \file include/dqpose/quat.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2024-2025
 *
 *     \brief A header file defining Quaternion operations
 * 
 *     This file provides the necessary classes and functions to represent 
 *     and manipulate Quaternions.
 * 
 *     \cite https://github.com/zhaojiawei392/dqpose.git
 */

#pragma once
#include <iostream>
#include <iomanip>
#include <cmath>
#include <array>

namespace dqpose
{

template <typename T>
constexpr inline T square(const T& x) {
    return x * x;
}

constexpr int PRINT_PRECISION = 12;

// Forward declarations
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class Quat;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class PureQuat;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class UnitQuat;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class UnitPureQuat;

template<typename qScalar, typename>
class Quat {
public:
protected:
    std::array<qScalar, 4> _data;
    constexpr inline qScalar& _w() noexcept {return _data[0];};
    constexpr inline qScalar& _x() noexcept {return _data[1];};
    constexpr inline qScalar& _y() noexcept {return _data[2];};
    constexpr inline qScalar& _z() noexcept {return _data[3];};
public:
    // Default Constructor
    constexpr explicit Quat() noexcept
        : _data{ 0, 0, 0, 0 } {

    }
    // Array Constructor
    constexpr explicit Quat(const std::array<qScalar, 4>& arr4) noexcept
        : _data( arr4 ) {

    }
    // Scalar Constructor
    constexpr explicit Quat(const qScalar w, const qScalar x=0, const qScalar y=0, const qScalar z=0) noexcept
        : _data{ w, x, y, z } {

    }
    // Copy Constructors 
    template<typename Scalar>
    constexpr Quat(const Quat<Scalar>& other) noexcept
        : _data{ static_cast<qScalar>(other.w()), static_cast<qScalar>(other.x()), static_cast<qScalar>(other.y()), static_cast<qScalar>(other.z()) } {

    }
    // Copy Assignment
    template<typename Scalar>
    constexpr inline Quat& operator=(const Quat<Scalar>& other) noexcept {
        _w() = static_cast<qScalar>(other.w());
        _x() = static_cast<qScalar>(other.x());
        _y() = static_cast<qScalar>(other.y());
        _z() = static_cast<qScalar>(other.z());
        return *this;
    }
    // operator+=
    template<typename Scalar>
    constexpr inline Quat& operator+=(const Quat<Scalar>& other) noexcept {
        _w() += static_cast<qScalar>(other.w());  
        _x() += static_cast<qScalar>(other.x()); 
        _y() += static_cast<qScalar>(other.y()); 
        _z() += static_cast<qScalar>(other.z());
        return *this; 
    }
    // operator-=
    template<typename Scalar>
    constexpr inline Quat& operator-=(const Quat<Scalar>& other) noexcept {
        _w() -= static_cast<qScalar>(other.w());  
        _x() -= static_cast<qScalar>(other.x()); 
        _y() -= static_cast<qScalar>(other.y()); 
        _z() -= static_cast<qScalar>(other.z());
        return *this; 
    }
    // operator*=
    template<typename Scalar>
    constexpr inline Quat& operator*=(const Quat<Scalar>& other) noexcept {
        const qScalar other_w = static_cast<qScalar>(other.w());
        const qScalar other_x = static_cast<qScalar>(other.x());
        const qScalar other_y = static_cast<qScalar>(other.y());
        const qScalar other_z = static_cast<qScalar>(other.z());
        _w() = w()*other_w - x()*other_x - y()*other_y - z()*other_z;  
        _x() = x()*other_w + w()*other_x - z()*other_y + y()*other_z; 
        _y() = y()*other_w + z()*other_x + w()*other_y - x()*other_z; 
        _z() = z()*other_w - y()*other_x + x()*other_y + w()*other_z; 
        return *this;
    }
    // operator*=
    constexpr inline Quat& operator*=(const qScalar scalar) noexcept {
        _w() *= scalar;  
        _x() *= scalar; 
        _y() *= scalar; 
        _z() *= scalar; 
        return *this;
    }
    // normalize
    constexpr inline Quat& normalize() {
        const qScalar norm = this->norm();
        if (norm == 0) {
            throw std::runtime_error("Error: Quat& normalize() Cannot normalize a 0 Quaternion.");
        }
        this->operator*=( 1 / norm );
        return *this;
    }
    // purify
    constexpr inline Quat& purify() noexcept {
        _w() = 0;
        return *this;
    }
    // operator+
    template<typename Scalar>
    constexpr inline Quat operator+(const Quat<Scalar>& other) const noexcept {
        const qScalar w_ = w() + static_cast<qScalar>(other.w());
        const qScalar x_ = x() + static_cast<qScalar>(other.x()); 
        const qScalar y_ = y() + static_cast<qScalar>(other.y()); 
        const qScalar z_ = z() + static_cast<qScalar>(other.z()); 
        return Quat(w_, x_, y_, z_);
    }
    // operator-
    template<typename Scalar>
    constexpr inline Quat operator-(const Quat<Scalar>& other) const noexcept {
        const qScalar w_ = w() - static_cast<qScalar>(other.w());
        const qScalar x_ = x() - static_cast<qScalar>(other.x());
        const qScalar y_ = y() - static_cast<qScalar>(other.y());
        const qScalar z_ = z() - static_cast<qScalar>(other.z());
        return Quat(w_, x_, y_, z_);
    }
    // operator*
    template<typename Scalar>
    constexpr inline Quat operator*(const Quat<Scalar>& other) const noexcept {
        const qScalar other_w = static_cast<qScalar>(other.w());
        const qScalar other_x = static_cast<qScalar>(other.x());
        const qScalar other_y = static_cast<qScalar>(other.y());
        const qScalar other_z = static_cast<qScalar>(other.z());
        const qScalar w_ = w()*other_w - x()*other_x - y()*other_y - z()*other_z;  
        const qScalar x_ = x()*other_w + w()*other_x - z()*other_y + y()*other_z; 
        const qScalar y_ = y()*other_w + z()*other_x + w()*other_y - x()*other_z; 
        const qScalar z_ = z()*other_w - y()*other_x + x()*other_y + w()*other_z; 
        return Quat(w_, x_, y_, z_);
    }
    // operator*
    constexpr inline Quat operator*(const qScalar scalar) const noexcept {
        const qScalar w_ = w() * scalar;  
        const qScalar x_ = x() * scalar; 
        const qScalar y_ = y() * scalar; 
        const qScalar z_ = z() * scalar; 
        return Quat(w_, x_, y_, z_);
    }
    // -operator
    constexpr inline Quat operator-() const noexcept { return Quat(-w(), -x(), -y(), -z()); }
    // operator==
    template<typename Scalar>
    constexpr inline bool operator==(const Quat<Scalar>& other) const noexcept { return _data == other._data; }
    // operator!=
    template<typename Scalar>
    constexpr inline bool operator!=(const Quat<Scalar>& other) const noexcept { return _data != other._data; }
    // dot
    template<typename Scalar>
    constexpr inline qScalar dot(const Quat<Scalar>& other) const noexcept {
        return    w() * static_cast<qScalar>(other.w()) 
                + x() * static_cast<qScalar>(other.x()) 
                + y() * static_cast<qScalar>(other.y()) 
                + z() * static_cast<qScalar>(other.z());
    }
    // norm
    constexpr inline qScalar norm() const noexcept {
        return std::sqrt( square( w() ) + square( x() ) + square( y() ) + square( z() ));
    }
    // copied
    constexpr inline Quat copied() const noexcept {
        return *this;
    }
    // normalized
    constexpr inline UnitQuat<qScalar> normalized() const {
        return UnitQuat<qScalar>(*this); 
    }
    // purified
    constexpr inline PureQuat<qScalar> purified() const noexcept {
        return PureQuat<qScalar>(*this); 
    }
    // conj
    constexpr inline Quat conj() const noexcept {
        const qScalar w_ = w();  
        const qScalar x_ = - x(); 
        const qScalar y_ = - y(); 
        const qScalar z_ = - z(); 
        return Quat(w_, x_, y_, z_);  
    }
    // inv
    constexpr inline Quat inv() const noexcept {
        const qScalar norm2 = square(norm());
        const qScalar w_ = w() / norm2;  
        const qScalar x_ = - x() / norm2; 
        const qScalar y_ = - y() / norm2; 
        const qScalar z_ = - z() / norm2; 
        return Quat(w_, x_, y_, z_);  
    }
    // log
    constexpr inline Quat log() const noexcept {
        const qScalar vec3_norm = std::sqrt( square( x() ) + square( y() ) + square( z() ));
        if (vec3_norm == 0) {
            return Quat(std::log(w()));
        }
        const qScalar this_norm = norm();
        const qScalar this_theta = acos(w() / this_norm);
        const qScalar w_ = std::log(this_norm);
        const qScalar x_ = this_theta * x() / vec3_norm;
        const qScalar y_ = this_theta * y() / vec3_norm;
        const qScalar z_ = this_theta * z() / vec3_norm;
        return Quat(w_, x_, y_, z_);  
    }
    // exp
    constexpr inline Quat exp() const noexcept {        
        const qScalar vec3_norm = std::sqrt( square( x() ) + square( y() ) + square( z() ));
        const qScalar exp_ = std::exp(w());        
        if (vec3_norm == 0) {
            return Quat(exp_);
        }
        const qScalar cos_ = cos(vec3_norm);
        const qScalar sin_ = sin(vec3_norm);
        const qScalar w_ = exp_ * cos_;
        const qScalar x_ = exp_ * sin_ * x() / vec3_norm;
        const qScalar y_ = exp_ * sin_ * y() / vec3_norm;
        const qScalar z_ = exp_ * sin_ * z() / vec3_norm;
        return Quat(w_, x_, y_, z_); 
    }
    // pow
    constexpr inline Quat pow(const qScalar index) const noexcept{
        return (this->log() * index).exp();
    }
    // hamiplus
    constexpr inline std::array<std::array<qScalar, 4>, 4> hamiplus() const noexcept {
        return std::array<std::array<qScalar, 4>, 4> { { w(), -x(), -y(), -z() },
                                                    { x(),  w(), -z(),  y() },
                                                    { y(),  z(),  w(), -x() },
                                                    { z(), -y(),  x(),  w() } };
    }
    // haminus
    constexpr inline std::array<std::array<qScalar, 4>, 4> haminus() const noexcept {
        return std::array<std::array<qScalar, 4>, 4> { { w(), -x(), -y(), -z() },
                                                    { x(),  w(),  z(), -y() },
                                                    { y(), -z(),  w(),  x() },
                                                    { z(),  y(), -x(),  w() } };            
    }
    // Query const
    constexpr inline qScalar w() const noexcept {return _data[0];}
    constexpr inline qScalar x() const noexcept {return _data[1];}
    constexpr inline qScalar y() const noexcept {return _data[2];}
    constexpr inline qScalar z() const noexcept {return _data[3];}
    // to_string
    constexpr inline std::string to_string() const {    
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION) << w() << " + " << x() << " î + " << y() << " ĵ + " << z() << " k̂";
        return oss.str();
    };
    // data
    constexpr inline const qScalar* data() const noexcept { return _data.data(); }
    constexpr inline std::array<qScalar, 4> array() const noexcept { return _data; }
    constexpr inline std::array<qScalar, 4> vrep_array() const noexcept { return std::array<qScalar, 4>{x(), y(), z(), w()}; }
    // Defaults

    virtual ~Quat()=default;    
            Quat(const Quat&)=default;
            Quat(Quat&&)=default;
    Quat& operator=(const Quat&)=default;
    Quat& operator=(Quat&&)=default;
};

template<typename qScalar, typename>
class PureQuat: public Quat<qScalar>
{ 
public:
    // Default Constructor
    constexpr explicit PureQuat() noexcept
        : Quat<qScalar>( ) {

    }
    // Array Constructor
    constexpr explicit PureQuat(const std::array<qScalar, 3> arr3) noexcept
        : Quat<qScalar>( 0, arr3[0], arr3[1], arr3[2] ) {

    }
    // Scalar Constructor
    constexpr explicit PureQuat(const qScalar x, const qScalar y=0, const qScalar z=0) noexcept
        : Quat<qScalar>( 0, x, y, z ) {

    }
    // Quat Constructor
    template<typename Scalar>
    constexpr PureQuat(const Quat<Scalar>& other) noexcept
        : Quat<qScalar>( other ) {
        this->_w() = 0;
    }
    // Quat Assignment
    template<typename Scalar>
    constexpr inline PureQuat& operator=(const Quat<Scalar>& other) noexcept {
        this->_w() = 0;
        this->_x() = static_cast<qScalar>(other.x());
        this->_y() = static_cast<qScalar>(other.y());
        this->_z() = static_cast<qScalar>(other.z());
        return *this;
    }
    // operator+=
    template<typename Scalar>
    constexpr inline PureQuat& operator+=(const PureQuat<Scalar>& other) noexcept {
        this->_w() = 0;  
        this->_x() += static_cast<qScalar>(other.x()); 
        this->_y() += static_cast<qScalar>(other.y()); 
        this->_z() += static_cast<qScalar>(other.z());
        return *this; 
    }  
    // operator-=  
    template<typename Scalar>
    constexpr inline PureQuat& operator-=(const PureQuat& other) noexcept {
        this->_w() = 0;  
        this->_x() -= static_cast<qScalar>(other.x()); 
        this->_y() -= static_cast<qScalar>(other.y()); 
        this->_z() -= static_cast<qScalar>(other.z());
        return *this; 
    }
    // operator*=
    constexpr inline PureQuat& operator*=(const qScalar scalar) noexcept {
        this->_w() = 0;  
        this->_x() *= scalar; 
        this->_y() *= scalar; 
        this->_z() *= scalar; 
        return *this;
    }
    // normalized
    constexpr inline UnitPureQuat<qScalar> normalized() const {
        return UnitPureQuat<qScalar>(*this); 
    }
    // data
    constexpr inline const qScalar* data() const noexcept { return this->_data.data()+1; }
    // array
    constexpr inline std::array<qScalar, 3> array() const noexcept { return std::array<qScalar, 3>{ this->x(), this->y(), this->z()}; }
    // Delete unsafe mutable operators    
    template<typename Scalar>
    constexpr inline Quat<qScalar>& operator+=(const Quat<Scalar>& other) noexcept =delete;
    template<typename Scalar>
    constexpr inline Quat<qScalar>& operator-=(const Quat<Scalar>& other) noexcept =delete;
    template<typename Scalar>
    constexpr inline Quat<qScalar>& operator*=(const Quat<Scalar>& other) noexcept =delete;
    // Defaults
        virtual ~PureQuat()=default;
                PureQuat(const PureQuat&)=default;
                PureQuat(PureQuat&&)=default;
    PureQuat& operator=(const PureQuat&)=default;
    PureQuat& operator=(PureQuat&&)=default;
};

template<typename qScalar, typename>
class UnitQuat : public Quat<qScalar>
{
public:
    // Default Constructor 
    constexpr explicit UnitQuat() noexcept
        : Quat<qScalar>( 1 ) {
        
    }
    // Array Constructor
    constexpr explicit UnitQuat(const std::array<qScalar, 4>& arr4) noexcept
        : Quat<qScalar>( arr4 ) {
        this->normalize();
    }
    // Scalar Constructor 
    constexpr explicit UnitQuat(const qScalar w, const qScalar x=0, const qScalar y=0, const qScalar z=0) noexcept
        : Quat<qScalar>( w, x, y, z ) {
        this->normalize();
    }
    // Quat Constructor
    template<typename Scalar>
    constexpr UnitQuat(const Quat<Scalar>& other) noexcept
        : Quat<qScalar>(other) {
        this->normalize();
    }
    // Quat Assignment 
    template<typename Scalar>
    constexpr inline UnitQuat& operator=(const Quat<Scalar>& other) noexcept {
        this->_w() = static_cast<qScalar>(other.w());
        this->_x() = static_cast<qScalar>(other.x());
        this->_y() = static_cast<qScalar>(other.y());
        this->_z() = static_cast<qScalar>(other.z());
        this->normalize();
        return *this;
    }
    // operator*=
    template<typename Scalar>
    constexpr inline UnitQuat& operator*=(const UnitQuat<Scalar>& other) noexcept {
        const qScalar other_w = static_cast<qScalar>(other.w());
        const qScalar other_x = static_cast<qScalar>(other.x());
        const qScalar other_y = static_cast<qScalar>(other.y());
        const qScalar other_z = static_cast<qScalar>(other.z());
        this->_w() = this->w()*other_w - this->x()*other_x - this->y()*other_y - this->z()*other_z; 
        this->_x() = this->x()*other_w + this->w()*other_x - this->z()*other_y + this->y()*other_z; 
        this->_y() = this->y()*other_w + this->z()*other_x + this->w()*other_y - this->x()*other_z; 
        this->_z() = this->z()*other_w - this->y()*other_x + this->x()*other_y + this->w()*other_z; 
        this->normalize();
        return *this;
    }
    // purified
    constexpr inline UnitPureQuat<qScalar> purified() const noexcept { return UnitPureQuat<qScalar>(*this); }
    // data
    constexpr inline const qScalar* data() const noexcept { return this->_data.data(); }
    // Delete unsafe mutable operators
    template<typename Scalar>
    constexpr inline Quat<qScalar>& operator+=(const Quat<Scalar>& other) noexcept =delete;
    template<typename Scalar>
    constexpr inline Quat<qScalar>& operator-=(const Quat<Scalar>& other) noexcept =delete;
    template<typename Scalar>
    constexpr inline Quat<qScalar>& operator*=(const Quat<Scalar>& other) noexcept =delete;
    constexpr inline Quat<qScalar>& operator*=(const qScalar scalar) noexcept =delete;   
    // Defaults
        virtual ~UnitQuat()=default;
                UnitQuat(const UnitQuat&)=default;
                UnitQuat(UnitQuat&&)=default;
    UnitQuat& operator=(const UnitQuat&)=default;
    UnitQuat& operator=(UnitQuat&&)=default;
};

template<typename qScalar, typename>
class UnitPureQuat : public Quat<qScalar>
{
public:
    // Default Constructor
    constexpr explicit UnitPureQuat() noexcept 
        : Quat<qScalar>( 0, 1, 0, 0 ) {

    }
    // Array Constructor
    constexpr explicit UnitPureQuat(const std::array<qScalar, 3> arr3) noexcept
        : Quat<qScalar>( 0, arr3[0], arr3[1], arr3[2] ) {
        this->normalize();
    }
    // Scalar Constructor
    constexpr explicit UnitPureQuat(const qScalar x, const qScalar y=0, const qScalar z=0) noexcept
        : Quat<qScalar>( 0, x, y, z ) {
        this->normalize();
    }
    // Quat Constructor
    template<typename Scalar>
    constexpr UnitPureQuat(const Quat<Scalar>& other) noexcept
        : Quat<qScalar>( other ) {
        this->_w() = 0;
        this->normalize();
    }
    // Quat Assignment
    constexpr inline UnitPureQuat& operator=(const Quat<qScalar>& other) noexcept {
        this->_w() = 0;
        this->_x() = static_cast<qScalar>(other.x());
        this->_y() = static_cast<qScalar>(other.y());
        this->_z() = static_cast<qScalar>(other.z());
        this->normalize();
        return *this;
    } 
    // data
    constexpr inline const qScalar* data() const noexcept { return this->_data.data()+1; }
    // array
    constexpr inline std::array<qScalar, 3> array() const noexcept { return std::array<qScalar, 3>{ this->x(), this->y(), this->z() }; }
    // Delete unsafe mutable operators
    template<typename Scalar>
    constexpr inline Quat<qScalar>& operator+=(const Quat<Scalar>& ) noexcept =delete;
    template<typename Scalar>
    constexpr inline Quat<qScalar>& operator-=(const Quat<Scalar>& ) noexcept =delete;
    template<typename Scalar>
    constexpr inline Quat<qScalar>& operator*=(const Quat<Scalar>& ) noexcept =delete;
    constexpr inline Quat<qScalar>& operator*=(const qScalar& ) noexcept =delete;
    // Default
            virtual ~UnitPureQuat()=default;
                    UnitPureQuat(const UnitPureQuat&)=default;
                    UnitPureQuat(UnitPureQuat&&)=default;
    UnitPureQuat& operator=(const UnitPureQuat&)=default;
    UnitPureQuat& operator=(UnitPureQuat&&)=default;
};
// operator<<
template<typename Scalar>
inline std::ostream& operator<<(std::ostream& os, const Quat<Scalar>& quat) {
    os << quat.to_string();  
    return os;
}
// operator*
template<typename Scalar1, typename Scalar2> 
inline std::enable_if_t<std::is_arithmetic_v<Scalar1>, Quat<Scalar2>>
operator*(const Scalar1 scalar, const Quat<Scalar2>& quat) noexcept {return quat * scalar;}

using Quatf = Quat<float>;
using UnitQuatf = UnitQuat<float>;
using PureQuatf = PureQuat<float>;
using UnitPureQuatf = UnitPureQuat<float>;
using Quatd = Quat<double>;
using UnitQuatd = UnitQuat<double>;
using PureQuatd = PureQuat<double>;
using UnitPureQuatd = UnitPureQuat<double>;
using Quatld = Quat<long double>;
using UnitQuatld = UnitQuat<long double>;
using PureQuatld = PureQuat<long double>;
using UnitPureQuatld = UnitPureQuat<long double>;

}