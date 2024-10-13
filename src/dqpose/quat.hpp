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
 *     \file src/dqpose/quat.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2024-2025
 *
 *     \brief A header file defining Quaternion operations
 * 
 *     This file provides the necessary classes and functions to represent 
 *     and manipulate Quaternions.
 * 
 *     \cite https://github.com/zhaojiawei392
 */

#pragma once
#include <iostream>
#include <iomanip>
#include <cmath>
#include <array>
#include <cstdint>

namespace dqpose
{

template <typename T>
inline T square(const T& x) {
    return x * x;
}

constexpr int PRINT_PRECISION = 12;
constexpr double FLOAT_ERROR_THRESHOLD = 0.0001;
constexpr bool VERBOSE = false;

// Forward declaration
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
using Arr3 = std::array<qScalar, 3>;
using Arr4 = std::array<qScalar, 4>;
using Mat44 = std::array<std::array<qScalar, 4>, 4>;
protected:
    Arr4 _data;
#define _W_ _data[0]
#define _X_ _data[1]
#define _Y_ _data[2]
#define _Z_ _data[3]
public:
    // Constructor
    explicit Quat(const qScalar w, const qScalar x=0, const qScalar y=0, const qScalar z=0) noexcept
    :_data{ w, x, y, z } {

    }
    // Copy Constructors 
    template<typename Scalar>
    explicit Quat(const Quat<Scalar>& other) noexcept
    :_data{ static_cast<qScalar>(other.w()), static_cast<qScalar>(other.x()), static_cast<qScalar>(other.y()), static_cast<qScalar>(other.z()) } {

    }
    // Copy Assignment
    template<typename Scalar>
    inline Quat& operator=(const Quat<Scalar>& other) noexcept {
        _W_ = static_cast<qScalar>(other.w());
        _X_ = static_cast<qScalar>(other.x());
        _Y_ = static_cast<qScalar>(other.y());
        _Z_ = static_cast<qScalar>(other.z());
        return *this;
    }
    // operator+=
    template<typename Scalar>
    inline Quat& operator+=(const Quat<Scalar>& other) noexcept {
        _W_ += static_cast<qScalar>(other.w());  
        _X_ += static_cast<qScalar>(other.x()); 
        _Y_ += static_cast<qScalar>(other.y()); 
        _Z_ += static_cast<qScalar>(other.z());
        return *this; 
    }
    // operator-=
    template<typename Scalar>
    inline Quat& operator-=(const Quat<Scalar>& other) noexcept {
        _W_ -= static_cast<qScalar>(other.w());  
        _X_ -= static_cast<qScalar>(other.x()); 
        _Y_ -= static_cast<qScalar>(other.y()); 
        _Z_ -= static_cast<qScalar>(other.z());
        return *this; 
    }
    // operator*=
    template<typename Scalar>
    inline Quat& operator*=(const Quat<Scalar>& other) noexcept {
        const qScalar other_w = static_cast<qScalar>(other.w());
        const qScalar other_x = static_cast<qScalar>(other.x());
        const qScalar other_y = static_cast<qScalar>(other.y());
        const qScalar other_z = static_cast<qScalar>(other.z());
        _W_ = w()*other_w - x()*other_x - y()*other_y - z()*other_z;  
        _X_ = x()*other_w + w()*other_x - z()*other_y + y()*other_z; 
        _Y_ = y()*other_w + z()*other_x + w()*other_y - x()*other_z; 
        _Z_ = z()*other_w - y()*other_x + x()*other_y + w()*other_z; 
        return *this;
    }
    // operator*=
    template<typename Scalar> 
    inline std::enable_if_t<std::is_arithmetic_v<Scalar>, Quat&>
    operator*=(const Scalar scalar) noexcept {
        const qScalar qscalar = static_cast<qScalar>(scalar);  
        _W_ *= qscalar;  
        _X_ *= qscalar; 
        _Y_ *= qscalar; 
        _Z_ *= qscalar; 
        return *this;
    }
    // normalize
    inline Quat& normalize() {
        const qScalar norm = this->norm();
        if (norm == 0) {
            throw std::runtime_error("Error: Cannot normalize a 0 Quaternion.");
        }
        this->operator*=( 1 / norm );
        return *this;
    }
    // operator+
    template<typename Scalar>
    inline Quat operator+(const Quat<Scalar>& other) const noexcept {
        const qScalar w_ = w() + static_cast<qScalar>(other.w());
        const qScalar x_ = x() + static_cast<qScalar>(other.x()); 
        const qScalar y_ = y() + static_cast<qScalar>(other.y()); 
        const qScalar z_ = z() + static_cast<qScalar>(other.z()); 
        return Quat(w_, x_, y_, z_);
    }
    // operator-
    template<typename Scalar>
    inline Quat operator-(const Quat<Scalar>& other) const noexcept {
        const qScalar w_ = w() - static_cast<qScalar>(other.w());
        const qScalar x_ = x() - static_cast<qScalar>(other.x());
        const qScalar y_ = y() - static_cast<qScalar>(other.y());
        const qScalar z_ = z() - static_cast<qScalar>(other.z());
        return Quat(w_, x_, y_, z_);
    }
    // operator*
    template<typename Scalar>
    inline Quat operator*(const Quat<Scalar>& other) const noexcept {
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
    template<typename Scalar>
    inline std::enable_if_t<std::is_arithmetic_v<Scalar>, Quat>
    operator*(const Scalar scalar) const noexcept {
        const qScalar qscalar = static_cast<qScalar>(scalar);  
        const qScalar w_ = w() * qscalar;  
        const qScalar x_ = x() * qscalar; 
        const qScalar y_ = y() * qscalar; 
        const qScalar z_ = z() * qscalar; 
        return Quat(w_, x_, y_, z_);
    }
    // -operator
    inline Quat operator-() const noexcept {
        const qScalar w_ = - w();  
        const qScalar x_ = - x(); 
        const qScalar y_ = - y(); 
        const qScalar z_ = - z(); 
        return Quat(w_, x_, y_, z_);
    }
    // operator==
    template<typename Scalar>
    inline bool operator==(const Quat<Scalar>& other) const noexcept{
        return _data == other._data;
    }
    // operator!=
    template<typename Scalar>
    inline bool operator!=(const Quat<Scalar>& other) const noexcept {
        return _data != other._data;
    }
    // dot
    template<typename Scalar>
    inline qScalar dot(const Quat<Scalar>& other) const noexcept {
        return    w() * static_cast<qScalar>(other.w()) 
                + x() * static_cast<qScalar>(other.x()) 
                + y() * static_cast<qScalar>(other.y()) 
                + z() * static_cast<qScalar>(other.z());
    }
    // rotation_axis
    inline Arr3 rotation_axis() const noexcept {
        if (rotation_angle() == 0){
            const Arr3 res{0, 0, 1};
            return res;
        }
        const qScalar vec3_norm = std::sqrt( square( x() ) + square( y() ) + square( z() ) );
        const qScalar x_ = x() / vec3_norm;
        const qScalar y_ = y() / vec3_norm;
        const qScalar z_ = z() / vec3_norm;
        const Arr3 res{x_, y_, z_};
        return res;
    }
    // rotation_angle
    inline qScalar rotation_angle() const noexcept {
        return 2 * acos(w() / norm());
    }
    // norm
    inline qScalar norm() const noexcept {
        return std::sqrt( square( w() ) + square( x() ) + square( y() ) + square( z() ));
    }
    // pow
    template<typename Scalar>
    inline std::enable_if_t<std::is_arithmetic_v<Scalar>, Quat>
    pow(const Scalar index) const noexcept{
        const qScalar init_norm = norm();
        const qScalar init_rotate_angle = 2 * acos(w() / init_norm);
        if (init_rotate_angle == 0){
            const qScalar w_ = std::pow(w(), index);
            return Quat(w);
        }
        const qScalar res_norm = std::pow(init_norm, index);
        const qScalar res_rotate_angle = init_rotate_angle * index;
        const qScalar vec3_norm = std::sqrt( square( x() ) + square( y() ) + square( z() ));

        const qScalar cos_ = cos(res_rotate_angle);
        const qScalar sin_ = sin(res_rotate_angle);

        const qScalar w_ = cos_ * res_norm;
        const qScalar x_ = sin_ * x() / vec3_norm * res_norm;
        const qScalar y_ = sin_ * y() / vec3_norm * res_norm;
        const qScalar z_ = sin_ * z() / vec3_norm * res_norm;
        return Quat(w_, x_, y_, z_);   
    }
    // conj
    inline Quat conj() const noexcept {
        const qScalar w_ = w();  
        const qScalar x_ = - x(); 
        const qScalar y_ = - y(); 
        const qScalar z_ = - z(); 
        return Quat(w_, x_, y_, z_);  
    }
    // inv
    inline Quat inv() const noexcept {
        const qScalar norm2 = square(norm());
        const qScalar w_ = w() / norm2;  
        const qScalar x_ = - x() / norm2; 
        const qScalar y_ = - y() / norm2; 
        const qScalar z_ = - z() / norm2; 
        return Quat(w_, x_, y_, z_);  
    }
    // log
    inline Quat log() const noexcept {
        const qScalar init_norm = norm();
        const qScalar init_rotate_angle = 2 * acos(w() / init_norm);
        if (init_rotate_angle == 0){
            const qScalar w_ = std::log(w());
            return Quat(w);
        }
        const qScalar vec3_norm = std::sqrt( square( x() ) + square( y() ) + square( z() ));

        const qScalar w_ = std::log(init_norm);
        const qScalar x_ = 0.5 * init_rotate_angle * x() / vec3_norm;
        const qScalar y_ = 0.5 * init_rotate_angle * y() / vec3_norm;
        const qScalar z_ = 0.5 * init_rotate_angle * z() / vec3_norm;
        return Quat(w_, x_, y_, z_);  
    }
    // exp
    inline Quat exp() const noexcept {        
        const qScalar init_norm = norm();
        const qScalar init_rotate_angle = 2 * acos(w() / norm);
        if (init_rotate_angle == 0){
            const qScalar w_ = std::exp(w());
            return Quat(w);
        }
        const qScalar vec3_norm = std::sqrt( square( x() ) + square( y() ) + square( z() ));
        
        const qScalar exp_ = std::exp(w());
        const qScalar cos_ = cos(vec3_norm);
        const qScalar sin_ = sin(vec3_norm);

        const qScalar w_ = exp_ * cos_;
        const qScalar x_ = exp_ * sin_ * x() / vec3_norm;
        const qScalar y_ = exp_ * sin_ * y() / vec3_norm;
        const qScalar z_ = exp_ * sin_ * z() / vec3_norm;
        return Quat(w_, x_, y_, z_); 
    }
    // normalized
    inline Quat normalized() const {
        const qScalar norm = this->norm();
        if (norm == 0) {
            throw std::runtime_error("Error: Cannot normalize a 0 Quaternion.");
        }
        return *this * (1 / norm); 
    }
    // hamiplus
    inline Mat44 hamiplus() const noexcept {
        return Mat44 { { w(), -x(), -y(), -z() },
                      { x(),  w(), -z(),  y() },
                      { y(),  z(),  w(), -x() },
                      { z(), -y(),  x(),  w() } };
    }
    // haminus
    inline Mat44 haminus() const noexcept {
        return Mat44 { { w(), -x(), -y(), -z() },
                      { x(),  w(),  z(), -y() },
                      { y(), -z(),  w(),  x() },
                      { z(),  y(), -x(),  w() } };            
    }
    // Query const
    inline qScalar w() const noexcept {return _W_;}
    inline qScalar x() const noexcept {return _X_;}
    inline qScalar y() const noexcept {return _Y_;}
    inline qScalar z() const noexcept {return _Z_;}
    // to_string
    inline std::string to_string() const {    
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION) << w() << " + " << x() << " î + " << y() << " ĵ + " << z() << " k̂";
        return oss.str();
    };
    // data
    inline qScalar* data() noexcept {return _data.data();}
    inline const qScalar* data() const noexcept {return _data.data();}
    inline Arr4 vrep_array() const noexcept { return Arr4{x(), y(), z(), w()}; }
    // Defaults

    virtual ~Quat()=default;
            Quat()=default;        
            Quat(const Quat&)=default;
            Quat(Quat&&)=default;
    Quat& operator=(const Quat&)=default;
    Quat& operator=(Quat&&)=default;
};

template<typename qScalar, typename>
class PureQuat: virtual public Quat<qScalar>
{ 
protected:
    // Unsafe assigment qualification check
    void _real_should_be_zero() {
        const qScalar real = std::abs(this->w());
        if (real > FLOAT_ERROR_THRESHOLD) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(PRINT_PRECISION)
                << "Error: " << "Trying to assign a unqualified Quaternion: " << *this << " to a Pure Quaternion.\n";
            throw std::runtime_error(oss.str());
        } 
        if (VERBOSE){
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(PRINT_PRECISION)
                << "Verbose: " << "Assign a Quaternion: " << *this << " to a Pure Quaternion. Real is set to 0.\n";
            std::cout << oss.str();
        }
        this->_W_ = 0;
    }
public:
    // Scalars Constructor
    explicit PureQuat(const qScalar x, const qScalar y, const qScalar z) noexcept
    : Quat<qScalar>( 0, x, y, z ) {

    }
    // Copy Constructor
    template<typename Scalar>
    explicit PureQuat(const PureQuat<Scalar>& other) noexcept 
    : Quat<qScalar>( other ) {
        this->_W_ = 0;
    }
    // Quat Constructor
    template<typename Scalar>
    explicit PureQuat(const Quat<Scalar>& other) noexcept
    : Quat<qScalar>( other ) {
        this->_W_ = 0;
    }
    // Copy Assignment
    template<typename Scalar>
    inline PureQuat& operator=(const PureQuat<Scalar>& other) noexcept {
        this->operator=(other);
    }
    // Unsafe Quat Assignment
    template<typename Scalar>
    inline PureQuat& operator=(const Quat<Scalar>& quat) {
        this->operator=(quat);
        _real_should_be_zero();
    }
    // operator+=
    template<typename Scalar>
    inline PureQuat& operator+=(const PureQuat<Scalar>& other) noexcept {
        this->_W_ = 0;  
        this->_X_ += static_cast<qScalar>(other.x()); 
        this->_Y_ += static_cast<qScalar>(other.y()); 
        this->_Z_ += static_cast<qScalar>(other.z());
        return *this; 
    }  
    // operator-=  
    template<typename Scalar>
    inline PureQuat& operator-=(const PureQuat& other) noexcept {
        this->_W_ = 0;  
        this->_X_ -= static_cast<qScalar>(other.x()); 
        this->_Y_ -= static_cast<qScalar>(other.y()); 
        this->_Z_ -= static_cast<qScalar>(other.z());
        return *this; 
    }
    // operator*=
    template<typename Scalar> 
    inline std::enable_if_t<std::is_arithmetic_v<Scalar>, PureQuat&>
    operator*=(const Scalar scalar) noexcept {
        const qScalar qscalar = static_cast<qScalar>(scalar);
        this->_W_ = 0;  
        this->_X_ *= qscalar; 
        this->_Y_ *= qscalar; 
        this->_Z_ *= qscalar; 
        return *this;
    }
    // operator+
    template<typename Scalar> 
    inline PureQuat operator+(const PureQuat<Scalar>& other) const noexcept {
        const qScalar x_ = this->x() + static_cast<qScalar>(other.x());
        const qScalar y_ = this->y() + static_cast<qScalar>(other.y());
        const qScalar z_ = this->z() + static_cast<qScalar>(other.z());
        return PureQuat(x_, y_, z_);
    }
    // operator-
    template<typename Scalar> 
    inline PureQuat operator-(const PureQuat<Scalar>& other) const noexcept {
        const qScalar x_ = this->x() - static_cast<qScalar>(other.x());
        const qScalar y_ = this->y() - static_cast<qScalar>(other.y());
        const qScalar z_ = this->z() - static_cast<qScalar>(other.z());
        return PureQuat(x_, y_, z_);
    }
    // operator*
    template<typename Scalar>
    inline std::enable_if_t<std::is_arithmetic_v<Scalar>, PureQuat>
    operator*(const Scalar scalar) const noexcept {
        const qScalar qscalar = static_cast<qScalar>(scalar);
        const qScalar x_ = this->x() * qscalar;
        const qScalar y_ = this->y() * qscalar;
        const qScalar z_ = this->z() * qscalar;
        return PureQuat(x_, y_, z_);
    }
    // data
    inline qScalar* data() noexcept { return this->_data.data()+1; }
    inline const qScalar* data() const noexcept { return this->_data.data()+1; }
    // Delete unsafe mutable operators
    Quat<qScalar>& operator+=(const Quat<qScalar>& other) noexcept =delete;
    Quat<qScalar>& operator-=(const Quat<qScalar>& other) noexcept =delete;
    Quat<qScalar>& operator*=(const Quat<qScalar>& other) noexcept =delete;
    template<typename Scalar> 
    std::enable_if_t<std::is_arithmetic_v<Scalar>, Quat<qScalar>&>
    operator*=(const Scalar scalar) noexcept =delete;
    // Defaults
        virtual ~PureQuat()=default;
                PureQuat()=default;
                PureQuat(const PureQuat&)=default;
                PureQuat(PureQuat&&)=default;
    PureQuat& operator=(const PureQuat&)=default;
    PureQuat& operator=(PureQuat&&)=default;
};

template<typename qScalar, typename>
class UnitQuat : virtual public Quat<qScalar>
{
protected:
    // Unsafe assignment qualification check
    void _norm_should_be_one(){
        const qScalar norm_err = std::abs(this->norm() - 1);    
        if (norm_err > FLOAT_ERROR_THRESHOLD) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(PRINT_PRECISION)
                << "Error: " << "Trying to assign a unqualified Quaternion: " << *this << " with a norm of " << std::to_string(this->norm()) << " to a Unit Quaternion.\n";
            throw std::runtime_error(oss.str());
        }
        if (VERBOSE){
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(PRINT_PRECISION)
                << "Verbose: " << "Assign a Quaternion: " << *this << " with a norm of " << std::to_string(this->norm()) << " to a Unit Quaternion. Normalizing is performed.\n";
            std::cout << oss.str();
        }
        this->normalize();
    }
public:
    // Scalars Constructor 
    explicit UnitQuat(const qScalar w, const qScalar x=0, const qScalar y=0, const qScalar z=0) noexcept
    : Quat<qScalar>( w, x, y, z ) {
        this->normalize();
    }
    // Copy Constructor 
    template<typename Scalar>
    explicit UnitQuat(const UnitQuat<Scalar>& other) noexcept
    : Quat<qScalar>(other) {
        this->normalize();
    }
    // Quat Constructor
    template<typename Scalar>
    explicit UnitQuat(const Quat<Scalar>& quat) noexcept
    : Quat<qScalar>(quat) {
        this->normalize();
    }
    // Copy Assignment 
    template<typename Scalar>
    inline UnitQuat& operator=(const UnitQuat<Scalar>& other) noexcept {
        this->operator=(other);
        this->normalize();
    }
    // Unsafe Quat Assignment 
    template<typename Scalar>
    inline UnitQuat& operator=(const Quat<Scalar>& quat) {
        this->operator=(quat);
        _norm_should_be_one();
    }
    // operator*=
    template<typename Scalar>
    inline UnitQuat& operator*=(const UnitQuat<Scalar>& other) noexcept {
        const qScalar other_w = static_cast<qScalar>(other.w());
        const qScalar other_x = static_cast<qScalar>(other.x());
        const qScalar other_y = static_cast<qScalar>(other.y());
        const qScalar other_z = static_cast<qScalar>(other.z());
        this->_W_ = this->w()*other_w - this->x()*other_x - this->y()*other_y - this->z()*other_z; 
        this->_X_ = this->x()*other_w + this->w()*other_x - this->z()*other_y + this->y()*other_z; 
        this->_Y_ = this->y()*other_w + this->z()*other_x + this->w()*other_y - this->x()*other_z; 
        this->_Z_ = this->z()*other_w - this->y()*other_x + this->x()*other_y + this->w()*other_z; 
        this->normalize();
        return *this;
    }
    // operator*
    template<typename Scalar>
    inline UnitQuat& operator*(const UnitQuat<Scalar>& other) const noexcept {
        const qScalar other_w = static_cast<qScalar>(other.w());
        const qScalar other_x = static_cast<qScalar>(other.x());
        const qScalar other_y = static_cast<qScalar>(other.y());
        const qScalar other_z = static_cast<qScalar>(other.z());
        const qScalar w_ = this->w()*other_w - this->x()*other_x - this->y()*other_y - this->z()*other_z;   
        const qScalar x_ = this->x()*other_w + this->w()*other_x - this->z()*other_y + this->y()*other_z;
        const qScalar y_ = this->y()*other_w + this->z()*other_x + this->w()*other_y - this->x()*other_z; 
        const qScalar z_ = this->z()*other_w - this->y()*other_x + this->x()*other_y + this->w()*other_z; 
        return UnitQuat(w_, x_, y_, z_);
    }
    // data
    inline qScalar* data() noexcept { return this->_data.data(); }
    inline const qScalar* data() const noexcept { return this->_data.data(); }
    // Delete unsafe mutable operators
    Quat<qScalar>& operator+=(const Quat<qScalar>& other) noexcept =delete;
    Quat<qScalar>& operator-=(const Quat<qScalar>& other) noexcept =delete;
    Quat<qScalar>& operator*=(const Quat<qScalar>& other) noexcept =delete;
    template<typename Scalar> 
    std::enable_if_t<std::is_arithmetic_v<Scalar>, Quat<qScalar>&>
    operator*=(const Scalar scalar) noexcept =delete;    
    // Defaults
        virtual ~UnitQuat()=default;
                UnitQuat()=default;
                UnitQuat(const UnitQuat&)=default;
                UnitQuat(UnitQuat&&)=default;
    UnitQuat& operator=(const UnitQuat&)=default;
    UnitQuat& operator=(UnitQuat&&)=default;
};

template<typename qScalar, typename>
class UnitPureQuat : public UnitQuat<qScalar>, public PureQuat<qScalar>
{
public:
    // Scalars Constructor
    explicit UnitPureQuat(const qScalar x, const qScalar y, const qScalar z) noexcept
    : Quat<qScalar>(0, x, y, z), UnitQuat<qScalar>(0, x, y, z), PureQuat<qScalar>(x, y, z){
        this->normalize();
    }
    // Copy Constructor
    template<typename Scalar>
    explicit UnitPureQuat(const UnitPureQuat<Scalar>& other) noexcept
    : Quat<qScalar>( other ) {
        this->_W_ = 0;
        this->normalize();
    }
    // Quat Constructor
    template<typename Scalar>
    explicit UnitPureQuat(const Quat<Scalar>& other) noexcept
    : Quat<qScalar>(other), UnitQuat<qScalar>(other), PureQuat<qScalar>(other) {
        this->_W_ = 0;
        this->normalize();
    }
    // Copy Assignment
    template<typename Scalar>
    inline UnitPureQuat& operator=(const UnitPureQuat<Scalar>& other) noexcept {
        Quat<qScalar>::operator=(other);
        this->_W_ = 0;
        this->normalize();
    }
    // Unsafe Quat Assignment
    inline UnitPureQuat& operator=(const Quat<qScalar>& other) {
        Quat<qScalar>::operator=(other);
        this->_real_should_be_zero();
        this->_norm_should_be_one();
    } 
    // data
    inline qScalar* data() noexcept { return this->_data.data()+1; }
    inline const qScalar* data() const noexcept { return this->_data.data()+1; }
    // Delete unsafe mutable operators
    Quat<qScalar>& operator+=(const Quat<qScalar>& other) noexcept =delete;
    Quat<qScalar>& operator-=(const Quat<qScalar>& other) noexcept =delete;
    Quat<qScalar>& operator*=(const Quat<qScalar>& other) noexcept =delete;
    template<typename Scalar> 
    std::enable_if_t<std::is_arithmetic_v<Scalar>, Quat<qScalar>&>
    operator*=(const Scalar scalar) noexcept =delete;
    // Default
            virtual ~UnitPureQuat()=default;
                    UnitPureQuat()=default;
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

#undef _W_
#undef _X_
#undef _Y_
#undef _Z_

using Quati = Quat<int>;
using UnitQuati = UnitQuat<int>;
using PureQuati = PureQuat<int>;
using UnitPureQuati = UnitPureQuat<int>;
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