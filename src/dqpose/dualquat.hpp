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
 *     \cite https://github.com/zhaojiawei392/dqpose.git
 */

#pragma once
#include "quat.hpp"

namespace dqpose 
{

// Forward declarations
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class DualQuat;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class PureDualQuat;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class UnitDualQuat;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class UnitPureDualQuat;
   
template<typename qScalar, typename>
class DualQuat{
public:
using Arr4 = std::array<qScalar, 4>;
using Arr8 = std::array<qScalar, 8>;
using Mat44 = std::array<std::array<qScalar, 4>, 4>;
using Mat88 = std::array<std::array<qScalar, 8>, 8>;
protected:
    std::array<Quat<qScalar>, 2> _data;
#define _REAL_ _data[0]
#define _DUAL_ _data[1]
public:
    // Quat Constructor
    explicit DualQuat(const Quat<qScalar>& real, const Quat<qScalar>& dual=Quat<qScalar>(0)) noexcept
    : _data{ real, dual } {

    }
    // Scalar Constructor
    explicit DualQuat(const qScalar h0, const qScalar h1=0, const qScalar h2=0, const qScalar h3=0, 
                      const qScalar h4=0, const qScalar h5=0, const qScalar h6=0, const qScalar h7=0) noexcept
    : _data{ Quat<qScalar>(h0, h1, h2, h3), Quat<qScalar>(h4, h5, h6, h7) } {

    }
    // Copy Constructor
    template<typename Scalar>
    explicit DualQuat(const DualQuat<Scalar>& other) noexcept
    : _data( static_cast<Quat<qScalar>>(other.real()), static_cast<Quat<qScalar>>(other.dual()) ) {

    }
    // Copy Assignment
    template<typename Scalar>
    inline DualQuat& operator=(const DualQuat<Scalar>& other) noexcept {
        _REAL_ = other.real();
        _DUAL_ = other.dual();
        return *this;
    }
    // operator+=    
    template<typename Scalar>
    inline DualQuat& operator+=(const DualQuat<Scalar>& other) noexcept {
        _REAL_ += other.real();
        _DUAL_ += other.dual(); 
        return *this;
    }
    // operator-= 
    template<typename Scalar>
    inline DualQuat& operator-=(const DualQuat<Scalar>& other) noexcept {
        _REAL_ -= other.real();
        _DUAL_ -= other.dual();   
        return *this;
    }
    // operator*= 
    template<typename Scalar>
    inline DualQuat& operator*=(const DualQuat<Scalar>& other) noexcept {
        _DUAL_ = _REAL_ * other.dual() + _DUAL_ * other.real();
        _REAL_ *= other.real();
        return *this;
    }
    // operator*= 
    template<typename Scalar>
    inline std::enable_if_t<std::is_arithmetic_v<Scalar>, DualQuat&> 
    operator*=(const Scalar scalar) noexcept {
        _REAL_ *= scalar;
        _DUAL_ *= scalar;
        return *this;
    }
    // normalize
    inline DualQuat& normalize() {
        const qScalar norm = _REAL_.norm();
        if (norm == 0) {
            throw std::runtime_error("Error: DualQuat& normalize() Cannot normalize a 0 Dual Quaternion.");
        }
        _REAL_ *= ( 1 / norm );
        _DUAL_ *= ( 1 / norm );
        return *this;
    }
    // operator+    
    template<typename Scalar>
    inline DualQuat operator+(const DualQuat<Scalar>& other) const noexcept {
        return DualQuat( _REAL_ + other.real(), _DUAL_ + other.dual() );
    }
    // operator-
    template<typename Scalar>
    inline DualQuat operator-(const DualQuat<Scalar>& other) const noexcept {
        return DualQuat( _REAL_ - other.real(), _DUAL_ - other.dual() );
    }
    // operator*  
    template<typename Scalar>
    inline DualQuat operator*(const DualQuat<Scalar>& other) const noexcept {
        const Quat<qScalar> dual_ = _REAL_ * other.dual() + _DUAL_ * other.real();
        const Quat<qScalar> real_ = _REAL_ * other.real();
        return DualQuat( real_, dual_ );
    }
    // operator*  
    template<typename Scalar>
    inline DualQuat operator*(const Quat<Scalar>& quat) const noexcept {
        return DualQuat( _REAL_ * quat, _DUAL_ * quat );
    }
    // operator*  
    template<typename Scalar>
    inline std::enable_if_t<std::is_arithmetic_v<Scalar>, DualQuat> 
    operator*(const Scalar scalar) const noexcept {
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
    // normalized
    inline DualQuat normalized() const {
        const qScalar norm = _REAL_.norm();
        if (norm == 0) {
            throw std::runtime_error("Error: DualQuat normalized() Cannot normalize a 0 Dual Quaternion.");
        }
        return *this * ( 1 / _REAL_.norm() );
    }
    // conj
    inline DualQuat conj() const noexcept {
        return DualQuat(_REAL_.conj(), _DUAL_.conj());
    }
    // inv
    inline DualQuat inv() const noexcept {
        const Quat<qScalar> real_ = _REAL_.inv();
        const Quat<qScalar> dual_ = - real_ * _DUAL_ * real_;
        return DualQuat( real_, dual_ );
    }
    // log
    inline DualQuat log() const noexcept {
        const Quat<qScalar> real_ = _REAL_.log();
        const Quat<qScalar> dual_ = _REAL_.inv() * _DUAL_;
        return DualQuat( real_, dual_ );
    }
    // exp
    inline DualQuat exp() const noexcept {
        const Quat<qScalar> real_ = _REAL_.exp();
        const Quat<qScalar> dual_ = real_ * _REAL_.inv() * _DUAL_;
        return DualQuat( real_, dual_ );
    }
    // pow
    template<typename Scalar>
    inline std::enable_if_t<std::is_arithmetic_v<Scalar>, DualQuat> 
    pow(const Scalar index) const noexcept {
        return (this->log() * index).exp();
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
    // query 
    inline Quat<qScalar> real() const noexcept { return _REAL_; }
    inline Quat<qScalar> dual() const noexcept { return _DUAL_; }
    // data
    inline qScalar* data() noexcept { return _data.data()[0].data(); }
    inline const qScalar* data() const noexcept { return _data.data()[0].data(); }
    inline Arr8 array() const noexcept { 
        const Arr4 res1 = _REAL_.array();
        const Arr4 res2 = _DUAL_.array();
        Arr8 res;
        std::copy(res1.begin(), res1.end(), res.begin());
        std::copy(res2.begin(), res2.end(), res.begin() + 4);
        return res; 
    }
    // to_string
    inline std::string to_string() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION) <<  _REAL_ << " + " << " ϵ ( " << _DUAL_ << " )";
        return oss.str();
    }
    // Default
            virtual ~DualQuat()=default;
                    DualQuat()=default;
                    DualQuat(const DualQuat& dq)=default;
                    DualQuat(DualQuat&& dq)=default;
    DualQuat& operator=(const DualQuat& dq)=default;
    DualQuat& operator=(DualQuat&& dq)=default;
};

template<typename qScalar, typename>
class PureDualQuat: public DualQuat<qScalar>{
public:

    // Constructors and Assignments

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit PureDualQuat()
    : DualQuat<qScalar>(1){

    }    
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit PureDualQuat(const Quat<qScalar>& primary, const Quat<qScalar>& dual)
    : DualQuat<qScalar>(primary, dual){
        __norm_should_be_one("explicit PureDualQuat(const Quat&, const Quat&)", this->real());
    }
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit PureDualQuat(const DualQuat<qScalar>& dq)
    : DualQuat<qScalar>(dq) {
        __norm_should_be_one("explicit PureDualQuat(const DualQuat&)", this->real());
    }
    inline PureDualQuat& operator=(const DualQuat<qScalar>& dq) {
        DualQuat<qScalar>::operator=(dq);
        __norm_should_be_one("PureDualQuat& operator=(const DualQuat&)", this->real());
    }

    inline PureDualQuat& operator*=(const PureDualQuat& other) noexcept {
        this->_REAL_ *= other.real();
        this->_DUAL_ = this->_REAL_ * other.dual() + this->_DUAL_ * other.real();
        __norm_should_be_one("PureDualQuat& operator*=(const PureDualQuat&)", this->real());
        return *this;
    }

    inline PureDualQuat operator*(const PureDualQuat& other) const noexcept {
        const Quat<qScalar> primary_ = this->_REAL_ * other.real();
        const Quat<qScalar> dual_ = this->_REAL_ * other.dual() + this->_DUAL_ * other.real();
        return PureDualQuat(primary_, dual_);
    }

    // Delete
    DualQuat& operator+=(const DualQuat& other) noexcept =delete;
    DualQuat& operator-=(const DualQuat& other) noexcept =delete;
    DualQuat& operator*=(const DualQuat& other) noexcept =delete;
    template<typename Scalar>
    std::enable_if_t<std::is_arithmetic_v<Scalar>, DualQuat&> 
    operator*=(const Scalar scalar) noexcept =delete;
    // Default
                virtual ~PureDualQuat()=default;
                        PureDualQuat(const PureDualQuat& dq)=default;
                        PureDualQuat(PureDualQuat&& dq)=default;
    PureDualQuat& operator=(const PureDualQuat& dq)=default;
    PureDualQuat& operator=(PureDualQuat&& dq)=default;
};

template<typename qScalar, typename>
class UnitDualQuat: public DualQuat<qScalar>{
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
    template<typename Scalar>
    std::enable_if_t<std::is_arithmetic_v<Scalar>, DualQuat&> 
    operator*=(const Scalar scalar) noexcept =delete;
    // Default
                virtual ~UnitDualQuat()=default;
                        UnitDualQuat(const UnitDualQuat& dq)=default;
                        UnitDualQuat(UnitDualQuat&& dq)=default;
    UnitDualQuat& operator=(const UnitDualQuat& dq)=default;
    UnitDualQuat& operator=(UnitDualQuat&& dq)=default;
};

template<typename qScalar, typename>
class UnitPureDualQuat: public DualQuat<qScalar>{
public:

    // Constructors and Assignments

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitPureDualQuat()
    : DualQuat<qScalar>(1){

    }    
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitPureDualQuat(const Quat<qScalar>& primary, const Quat<qScalar>& dual)
    : DualQuat<qScalar>(primary, dual){
        __norm_should_be_one("explicit UnitPureDualQuat(const Quat&, const Quat&)", this->real());
    }
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitPureDualQuat(const DualQuat<qScalar>& dq)
    : DualQuat<qScalar>(dq) {
        __norm_should_be_one("explicit UnitPureDualQuat(const DualQuat&)", this->real());
    }
    inline UnitPureDualQuat& operator=(const DualQuat<qScalar>& dq) {
        DualQuat<qScalar>::operator=(dq);
        __norm_should_be_one("UnitPureDualQuat& operator=(const DualQuat&)", this->real());
    }

    inline UnitPureDualQuat& operator*=(const UnitPureDualQuat& other) noexcept {
        this->_REAL_ *= other.real();
        this->_DUAL_ = this->_REAL_ * other.dual() + this->_DUAL_ * other.real();
        __norm_should_be_one("UnitPureDualQuat& operator*=(const UnitPureDualQuat&)", this->real());
        return *this;
    }

    inline UnitPureDualQuat operator*(const UnitPureDualQuat& other) const noexcept {
        const Quat<qScalar> primary_ = this->_REAL_ * other.real();
        const Quat<qScalar> dual_ = this->_REAL_ * other.dual() + this->_DUAL_ * other.real();
        return UnitPureDualQuat(primary_, dual_);
    }

    // Delete
    DualQuat& operator+=(const DualQuat& other) noexcept =delete;
    DualQuat& operator-=(const DualQuat& other) noexcept =delete;
    DualQuat& operator*=(const DualQuat& other) noexcept =delete;
    template<typename Scalar>
    std::enable_if_t<std::is_arithmetic_v<Scalar>, DualQuat&> 
    operator*=(const Scalar scalar) noexcept =delete;
    // Default
                virtual ~UnitPureDualQuat()=default;
                        UnitPureDualQuat(const UnitPureDualQuat& dq)=default;
                        UnitPureDualQuat(UnitPureDualQuat&& dq)=default;
    UnitPureDualQuat& operator=(const UnitPureDualQuat& dq)=default;
    UnitPureDualQuat& operator=(UnitPureDualQuat&& dq)=default;
};

// operator<<
template<typename Scalar>
inline std::ostream& operator<<(std::ostream& os, const DualQuat<Scalar>& dq) {
    os << dq.to_string();  
    return os;
}
// operator*
template<typename Scalar1, typename Scalar2>
inline std::enable_if_t<std::is_arithmetic_v<Scalar1>, DualQuat<Scalar2>>
 operator*(const Scalar1 scalar, const DualQuat<Scalar2>& dq) noexcept {return dq * scalar;}
// operator*
template<typename Scalar1, typename Scalar2>
inline DualQuat<Scalar1> operator*(const Quat<Scalar1>& quat, const DualQuat<Scalar2>& dq) noexcept {
    return DualQuat( quat * dq._REAL_, quat * dq._DUAL_ );
}

#undef _REAL_
#undef _DUAL_

}