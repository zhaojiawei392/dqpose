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
 *     \file src/dqpose/dualquat.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2024-2025
 *
 *     \brief A header file defining Dual Quaternion operations
 * 
 *     This file provides the necessary classes and functions to represent 
 *     and manipulate Dual Quaternions.
 * 
 *     \cite https://github.com/zhaojiawei392
 */

#pragma once
#include "quat.hpp"

// Forward declaration
template<typename qScalar>
class DualQuat;
template<typename qScalar>
class PureDualQuat;
template<typename qScalar>
class UnitDualQuat;
template<typename qScalar>
class UnitPureDualQuat;

namespace dqpose 
{
    
template<typename qScalar>
class DualQuat{
using Arr3 = std::array<qScalar, 3>;
using Arr4 = std::array<qScalar, 4>;
using Mat44 = std::array<std::array<qScalar, 4>, 4>;
using Mat88 = std::array<std::array<qScalar, 8>, 8>;
protected:
    std::array<Quat<qScalar>, 2> _data;
#define _REAL_ _data[0]
#define _DUAL_ _data[1]
public:
    // Constructors 0
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit DualQuat(const Quat<qScalar>& real, const Quat<qScalar>& dual=Quat<qScalar>())
    : _data{ real, dual } {

    }
    // Constructors 1
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit DualQuat(const qScalar h0, const qScalar h1=0, const qScalar h2=0, const qScalar h3=0, const qScalar h4=0, const qScalar h5=0, const qScalar h6=0, const qScalar h7=0)
    : _data{ Quat<qScalar>(h0, h1, h2, h3), Quat<qScalar>(h4, h5, h6, h7) } {

    }
    // Assignment
    inline DualQuat& operator=(const Quat<qScalar>& other) noexcept {
        _REAL_ = other;
    }
    // operator+=    
    inline DualQuat& operator+=(const DualQuat& other) noexcept {
        _REAL_ += other.real();
        _DUAL_ += other.dual(); 
        return *this;
    }
    // operator-= 
    inline DualQuat& operator-=(const DualQuat& other) noexcept {
        _REAL_ -= other.real();
        _DUAL_ -= other.dual();   
        return *this;
    }
    // operator*= 
    inline DualQuat& operator*=(const DualQuat& other) noexcept {
        _DUAL_ = _REAL_ * other.dual() + _DUAL_ * other.real();
        _REAL_ *= other.real();
        return *this;
    }
    // operator*= 
    template<typename Scalar_>
    inline std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuat&> 
    operator*=(const Scalar_ scalar) noexcept {
        _REAL_ *= scalar;
        _DUAL_ *= scalar;
        return *this;
    }
    // normalize
    inline DualQuat& normalize() noexcept {
        const qScalar norm = _REAL_.norm();
        _REAL_ *= ( 1 / norm );
        _DUAL_ *= ( 1 / norm );
        return *this;
    }
    // operator+    
    inline DualQuat operator+(const DualQuat& other) const noexcept {
        return DualQuat( _REAL_ + other.real(), _DUAL_ + other.dual() );
    }
    // operator-
    inline DualQuat operator-(const DualQuat& other) const noexcept {
        return DualQuat( _REAL_ - other.real(), _DUAL_ - other.dual() );
    }
    // operator*  
    inline DualQuat operator*(const DualQuat& other) const noexcept {
        return DualQuat( _REAL_ * other.real(), _REAL_ * other.dual() + _DUAL_ * other.real());
    }
    // operator*  
    template<typename Scalar_>
    inline std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuat> 
    operator*(const Scalar_ scalar) const noexcept {
        return DualQuat( _REAL_ * scalar, _DUAL_ * scalar );
    }
    // -operator  
    inline DualQuat operator-() const noexcept {
        return DualQuat( -_REAL_, -_DUAL_ );
    }
    // operator== 
    inline bool operator==(const DualQuat& other) const noexcept {
        return _REAL_ == other.real() && _DUAL_ == other.dual();
    }
    // operator!= 
    inline bool operator!=(const DualQuat& other) const noexcept {
        return _REAL_ != other.real() || _DUAL_ != other.dual();
    }
    // norm
    inline DualQuat norm() const noexcept {
        const qScalar real_norm = _REAL_.norm();
        if (real_norm == 0) 
            return DualQuat(0);
        const qScalar res_dual_norm = _REAL_.dot(_DUAL_) / real_norm;
        return DualQuat(Quat<qScalar>(real_norm), Quat<qScalar>(res_dual_norm));
    }
    // conj
    inline DualQuat conj() const noexcept {
        return DualQuat(_REAL_.conj(), _DUAL_.conj());
    }
    // normalized
    inline DualQuat normalized() const noexcept {
        return *this * ( 1 / _REAL_.norm() );
    }
    // hamiplus
    inline Mat88 hamiplus() const noexcept {
        const Mat44 real_hami = _REAL_.hamiplus();
        const Mat44 dual_hami = _DUAL_.hamiplus();
        // Initialize a zero container
        Mat88 res{}; 
        for (int i=0; i<4; ++i) {
            // fill the first 4x4 block
            std::copy(real_hami[i].begin(), real_hami[i].end(), res[i].begin());
            // the Second 4x4 block is [0]
            // fill the third 4x4 block
            std::copy(dual_hami[i].begin(), dual_hami[i].end(), res[i+4].begin());
            std::copy(real_hami[i].begin(), real_hami[i].end(), res[i+4].begin()+4);
        }
        return res;
    }
    // haminus
    inline Mat88 haminus() const noexcept {        
        const Mat44 real_hami = _REAL_.haminus();
        const Mat44 dual_hami = _DUAL_.haminus();
        // Initialize a zero container
        Mat88 res{};
        for (int i=0; i<4; ++i) {
            // fill the first 4x4 block
            std::copy(real_hami[i].begin(), real_hami[i].end(), res[i].begin());
            // the Second 4x4 block is [0]
            // fill the third 4x4 block
            std::copy(dual_hami[i].begin(), dual_hami[i].end(), res[i+4].begin());
            // fill the fourth 4x4 block
            std::copy(real_hami[i].begin(), real_hami[i].end(), res[i+4].begin()+4);
        }
        return res;
    }
    // query real
    inline Quat<qScalar> real() const noexcept {
        return _REAL_;
    }
    // query dual
    inline Quat<qScalar> dual() const noexcept {
        return _DUAL_;
    }
    // data
    inline qScalar* data() noexcept { return _data.data()[0].data(); }
    inline const qScalar* data() const noexcept { return _data.data()[0].data(); }
    // to_string
    inline std::string to_string() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION) <<  _REAL_ << " + " << " ϵ ( " << _DUAL_ << " )";
        return oss.str();
    }
    // Friend operator<<
    template<typename fScalar_>
    inline friend std::ostream& operator<<(std::ostream& os, const DualQuat<fScalar_>& dq) {
        os << dq.to_string();  
        return os;
    }
    // Friend operator*
    template<typename Scalar_, typename fScalar_>
    inline friend std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuat<fScalar_>>
    operator*(const Scalar_ scalar, const DualQuat<fScalar_>& dq) noexcept {return dq * scalar;}
    // Friend operator*
    template<typename fScalar_>
    inline friend DualQuat<fScalar_> operator*(const Quat<fScalar_>& quat, const DualQuat<fScalar_>& dq) noexcept {
        return DualQuat( quat * dq._REAL_, quat * dq._DUAL_ );
    }
    // Friend operator*
    template<typename fScalar_>
    inline friend DualQuat<fScalar_> operator*(const DualQuat<fScalar_>& dq, const Quat<fScalar_>& quat) noexcept {
        return DualQuat( dq._REAL_ * quat, dq._DUAL_ * quat );
    }
    // Default
            virtual ~DualQuat()=default;
                    DualQuat()=default;
                    DualQuat(const DualQuat& dq)=default;
                    DualQuat(DualQuat&& dq)=default;
    DualQuat& operator=(const DualQuat& dq)=default;
    DualQuat& operator=(DualQuat&& dq)=default;

private:
    // not inplemented
    DualQuat inv() const noexcept {}
    DualQuat log() const noexcept {}
    DualQuat exp() const noexcept {}
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuat> 
    pow(const Scalar_ index) const noexcept {}
};


template<typename qScalar>
class UnitDualQuat: public DualQuat<qScalar>{
using Arr3 = Arr<qScalar, 3>;
using Arr4 = Arr<qScalar, 4>;
using Arr8 = Arr<qScalar, 8>;
using Mat3 = Mat<qScalar, 3,3>;
using Mat44 = Mat<qScalar, 4,4>;
using Mat88 = Mat<qScalar, 8,8>;
public:

    // Constructors and Assignments

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitDualQuat()
    : DualQuat<qScalar>(1){

    }    
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitDualQuat(const Quat<qScalar>& primary, const Quat<qScalar>& dual)
    : DualQuat<qScalar>(primary, dual){
        __norm_should_be_one("explicit UnitDualQuat(const Quat&, const Quat&)", this->real());
    }
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitDualQuat(const DualQuat<qScalar>& dq)
    : DualQuat<qScalar>(dq) {
        __norm_should_be_one("explicit UnitDualQuat(const DualQuat&)", this->real());
    }
    inline UnitDualQuat& operator=(const DualQuat<qScalar>& dq) {
        DualQuat<qScalar>::operator=(dq);
        __norm_should_be_one("UnitDualQuat& operator=(const DualQuat&)", this->real());
    }

    inline UnitDualQuat& operator*=(const UnitDualQuat& other) noexcept {
        this->_REAL_ *= other.real();
        this->_DUAL_ = this->_REAL_ * other.dual() + this->_DUAL_ * other.real();
        __norm_should_be_one("UnitDualQuat& operator*=(const UnitDualQuat&)", this->real());
        return *this;
    }

    inline UnitDualQuat operator*(const UnitDualQuat& other) const noexcept {
        const Quat<qScalar> primary_ = this->_REAL_ * other.real();
        const Quat<qScalar> dual_ = this->_REAL_ * other.dual() + this->_DUAL_ * other.real();
        return UnitDualQuat(primary_, dual_);
    }

    // Delete
    DualQuat& operator+=(const DualQuat& other) noexcept =delete;
    DualQuat& operator-=(const DualQuat& other) noexcept =delete;
    DualQuat& operator*=(const DualQuat& other) noexcept =delete;
    template<typename Scalar_>
    std::enable_if_t<std::is_arithmetic_v<Scalar_>, DualQuat&> 
    operator*=(const Scalar_ scalar) noexcept =delete;
    // Default
                virtual ~UnitDualQuat()=default;
                        UnitDualQuat(const UnitDualQuat& dq)=default;
                        UnitDualQuat(UnitDualQuat&& dq)=default;
    UnitDualQuat& operator=(const UnitDualQuat& dq)=default;
    UnitDualQuat& operator=(UnitDualQuat&& dq)=default;
};

}