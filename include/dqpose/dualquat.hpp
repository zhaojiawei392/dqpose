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
 *     \file include/dqpose/dualquat.hpp
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
    inline Quat<qScalar>& real() {return _data[0];}
    inline Quat<qScalar>& dual() {return _data[1];}
public:
    // Default Constructor
    explicit DualQuat() noexcept
    : _data{ Quat<qScalar>(), Quat<qScalar>() } {

    }
    // Scalar Constructor
    explicit DualQuat(const qScalar w1, const qScalar x1=0, const qScalar y1=0, const qScalar z1=0, 
                      const qScalar w2=0, const qScalar x2=0, const qScalar y2=0, const qScalar z2=0) noexcept
    : _data{ Quat<qScalar>(w1, x1, y1, z1), Quat<qScalar>(w2, x2, y2, z2) } {

    }
    // Real Constructor
    template<typename Scalar>
    explicit DualQuat(const Quat<Scalar>& real) noexcept
    : _data{ Quat<qScalar>(real), Quat<qScalar>(0) } {

    }
    // Real-Dual Constructor
    template<typename Scalar1, typename Scalar2>
    explicit DualQuat(const Quat<Scalar1>& real, const Quat<Scalar2>& dual) noexcept
    : _data{ Quat<qScalar>(real), Quat<qScalar>(dual) } {

    }
    // Copy Constructor
    template<typename Scalar>
    explicit DualQuat(const DualQuat<Scalar>& other) noexcept
    : _data{ Quat<qScalar>(other.real()), Quat<qScalar>(other.dual()) } {

    }
    // Copy Assignment
    template<typename Scalar>
    inline DualQuat& operator=(const DualQuat<Scalar>& other) noexcept {
        real() = other.real();
        dual() = other.dual();
        return *this;
    }
    // operator+=    
    template<typename Scalar>
    inline DualQuat& operator+=(const DualQuat<Scalar>& other) noexcept {
        real() += other.real();
        dual() += other.dual(); 
        return *this;
    }
    // operator-= 
    template<typename Scalar>
    inline DualQuat& operator-=(const DualQuat<Scalar>& other) noexcept {
        real() -= other.real();
        dual() -= other.dual();   
        return *this;
    }
    // operator*= 
    template<typename Scalar>
    inline DualQuat& operator*=(const DualQuat<Scalar>& other) noexcept {
        dual() = real() * other.dual() + dual() * other.real();
        real() *= other.real();
        return *this;
    }
    // operator*= 
    inline DualQuat& operator*=(const qScalar scalar) noexcept {
        real() *= scalar;
        dual() *= scalar;
        return *this;
    }
    // normalize
    inline DualQuat& normalize() {
        const qScalar norm = real().norm();
        if (norm == 0) {
            throw std::runtime_error("Error: DualQuat& normalize() Cannot normalize a 0 Dual Quaternion.");
        }
        real() *= ( 1 / norm );
        dual() *= ( 1 / norm );
        return *this;
    }
    // purifiy
    inline DualQuat& purify() noexcept {
        real().purify();
        dual().purify();
        return *this;
    }
    // operator+    
    template<typename Scalar>
    inline DualQuat operator+(const DualQuat<Scalar>& other) const noexcept {
        return DualQuat( real() + other.real(), dual() + other.dual() );
    }
    // operator-
    template<typename Scalar>
    inline DualQuat operator-(const DualQuat<Scalar>& other) const noexcept {
        return DualQuat( real() - other.real(), dual() - other.dual() );
    }
    // operator*  
    template<typename Scalar>
    inline DualQuat operator*(const DualQuat<Scalar>& other) const noexcept {
        const Quat<qScalar>& dual_ = real() * other.dual() + dual() * other.real();
        const Quat<qScalar>& real_ = real() * other.real();
        return DualQuat( real_, dual_ );
    }
    // operator*  
    template<typename Scalar>
    inline DualQuat operator*(const Quat<Scalar>& quat) const noexcept {
        return DualQuat( real() * quat, dual() * quat );
    }
    // operator*  
    inline DualQuat operator*(const qScalar scalar) const noexcept {
        return DualQuat( real() * scalar, dual() * scalar );
    }
    // -operator  
    inline DualQuat operator-() const noexcept {
        return DualQuat( -real(), -dual() );
    }
    // operator== 
    inline bool operator==(const DualQuat& other) const noexcept {
        return real() == other.real() && dual() == other.dual();
    }
    // operator!= 
    inline bool operator!=(const DualQuat& other) const noexcept {
        return real() != other.real() || dual() != other.dual();
    }
    // norm
    inline DualQuat norm() const noexcept {
        const qScalar real_norm = real().norm();
        if (real_norm == 0) 
            return DualQuat(0);
        const qScalar res_dual_norm = real().dot(dual()) / real_norm;
        return DualQuat(Quat<qScalar>(real_norm), Quat<qScalar>(res_dual_norm));
    }
    // copied
    inline DualQuat copied() const noexcept {
        return *this;
    }
    // normalized
    inline DualQuat normalized() const {
        const qScalar norm = real().norm();
        if (norm == 0) {
            throw std::runtime_error("Error: DualQuat normalized() Cannot normalize a 0 Dual Quaternion.");
        }
        return *this * ( 1 / real().norm() );
    }
    // purified
    inline DualQuat purified() const noexcept {
        return copied().purify();
    }
    // conj
    inline DualQuat conj() const noexcept {
        return DualQuat(real().conj(), dual().conj());
    }
    // inv
    inline DualQuat inv() const noexcept {
        const Quat<qScalar>& real_ = real().inv();
        const Quat<qScalar>& dual_ = - real_ * dual() * real_;
        return DualQuat( real_, dual_ );
    }
    // log
    inline DualQuat log() const noexcept {
        const Quat<qScalar>& real_ = real().log();
        const Quat<qScalar>& dual_ = real().inv() * dual();
        return DualQuat( real_, dual_ );
    }
    // exp
    inline DualQuat exp() const noexcept {
        const Quat<qScalar>& real_ = real().exp();
        const Quat<qScalar>& dual_ = real_ * real().inv() * dual();
        return DualQuat( real_, dual_ );
    }
    // pow
    inline DualQuat pow(const qScalar index) const noexcept {
        return (this->log() * index).exp();
    }
    // hamiplus
    inline Mat88 hamiplus() const noexcept {
        const Mat44 real_hami = real().hamiplus();
        const Mat44 dual_hami = dual().hamiplus();
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
        const Mat44 real_hami = real().haminus();
        const Mat44 dual_hami = dual().haminus();
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
    inline Quat<qScalar> real() const noexcept { return _data[0]; }
    inline Quat<qScalar> dual() const noexcept { return _data[1]; }
    // data
    inline const qScalar* data() const noexcept { return _data.data()[0].data(); }
    inline Arr8 array() const noexcept { 
        Arr8 res;
        std::copy(data(), data()+8, res.begin());
        return res; 
    }
    // to_string
    inline std::string to_string() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION) <<  real() << " + " << " ϵ ( " << dual() << " )";
        return oss.str();
    }
    // Default
            virtual ~DualQuat()=default;
                    DualQuat(const DualQuat& dq)=default;
                    DualQuat(DualQuat&& dq)=default;
    DualQuat& operator=(const DualQuat& dq)=default;
    DualQuat& operator=(DualQuat&& dq)=default;
};

template<typename qScalar, typename>
class PureDualQuat: public DualQuat<qScalar>{
public:
    // Default Constructor
    explicit PureDualQuat() noexcept
    : DualQuat<qScalar>( ) {

    }
    // Scalar Constructor
    explicit PureDualQuat(const qScalar x1, const qScalar y1=0, const qScalar z1=0, 
                          const qScalar x2=0, const qScalar y2=0, const qScalar z2=0) noexcept
    : DualQuat<qScalar>( 0, x1, y1, z1, 0, x2, y2, z2 ) {

    }
    // Real Constructor
    template<typename Scalar>
    explicit PureDualQuat(const Quat<Scalar>& real) noexcept
    : DualQuat<qScalar>{ real } {

    }
    // Real-Dual Constructor
    template<typename Scalar1, typename Scalar2>
    explicit PureDualQuat(const PureQuat<Scalar1>& real, const PureQuat<Scalar2>& dual) noexcept
    : DualQuat<qScalar>( real, dual ) {

    }
    // DualQuat Constructor
    template <typename Scalar>
    explicit PureDualQuat(const DualQuat<Scalar>& other) noexcept
    : DualQuat<qScalar>( other )  {
        this->purify();
    }
    // DualQuat Assignment
    template<typename Scalar>
    inline PureDualQuat& operator=(const DualQuat<Scalar>& other) noexcept {
        this->real() = other.real();
        this->dual() = other.dual();
        this->purify();
        return *this;
    }
    // operator+=    
    template<typename Scalar>
    inline PureDualQuat& operator+=(const PureDualQuat<Scalar>& other) noexcept {
        this->real() += other.real();
        this->dual() += other.dual();
        this->purify();
        return *this;
    }
    // operator-= 
    template<typename Scalar>
    inline PureDualQuat& operator-=(const PureDualQuat<Scalar>& other) noexcept {
        this->real() -= other.real();
        this->dual() -= other.dual(); 
        this->purify();
        return *this;
    }
    // operator*= 
    inline PureDualQuat& operator*=(const qScalar scalar) noexcept {
        this->real() *= scalar;
        this->dual() *= scalar;
        this->purify();
        return *this;
    }
    // operator+
    template<typename Scalar>
    inline PureDualQuat operator+(const PureDualQuat<Scalar>& other) const noexcept {
        return PureDualQuat( this->real() + other.real(), this->dual() + other.dual() );
    } 
    // operator-
    template<typename Scalar>
    inline PureDualQuat operator-(const PureDualQuat<Scalar>& other) const noexcept {
        return PureDualQuat( this->real() - other.real(), this->dual() - other.dual() );
    } 
    // operator*
    inline PureDualQuat operator*(const qScalar scalar) noexcept {
        this->purify();
        return PureDualQuat( this->real() * scalar, this->dual() * scalar );
    }

    // Delete 
    template<typename Scalar>
    inline DualQuat<qScalar>& operator+=(const DualQuat<Scalar>& other) noexcept =delete;
    template<typename Scalar>
    inline DualQuat<qScalar>& operator-=(const DualQuat<Scalar>& other) noexcept =delete;
    template<typename Scalar>
    inline DualQuat<qScalar>& operator*=(const DualQuat<Scalar>& other) noexcept =delete;
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
    // Default Constructor
    explicit UnitDualQuat() noexcept
    : DualQuat<qScalar>( 1 ) {
    }
    // Scalar Constructor
    explicit UnitDualQuat(const qScalar w1, const qScalar x1=0, const qScalar y1=0, const qScalar z1=0, 
                          const qScalar w2=0, const qScalar x2=0, const qScalar y2=0, const qScalar z2=0) noexcept
    : DualQuat<qScalar>( w1, x1, y1, z1, w2, x2, y2, z2 ) {
        this->normalize();
    }
    // Real Constructor
    template<typename Scalar>
    explicit UnitDualQuat(const Quat<Scalar>& real) noexcept
    : DualQuat<qScalar>{ real } {

    }
    // Real-Dual Constructor
    template<typename Scalar1, typename Scalar2>
    explicit UnitDualQuat(const Quat<Scalar1>& real, const Quat<Scalar2>& dual) noexcept
    : DualQuat<qScalar>( real, dual ) {
        this->normalize();
    }
    // DualQuat Constructor
    template <typename Scalar>
    explicit UnitDualQuat(const DualQuat<Scalar>& other) noexcept
    : DualQuat<qScalar>( other ) {
        this->normalize();
    }
    // DualQuat Assignment
    template<typename Scalar>
    inline UnitDualQuat& operator=(const DualQuat<Scalar>& other) noexcept {
        this->real() = other.real();
        this->dual() = other.dual();
        this->normalize();
        return *this;
    }
    // operator*=
    template<typename Scalar>
    inline UnitDualQuat& operator*=(const UnitDualQuat<Scalar>& other) noexcept {
        this->dual() = this->real() * other.dual() + this->dual() * other.real();
        this->real() *= other.real();
        this->normalize();
        return *this;
    } 
    // operator*
    template<typename Scalar>
    inline UnitDualQuat operator*(const UnitDualQuat<Scalar>& other) const noexcept {
        const Quat<qScalar>& dual_ = this->real() * other.dual() + this->dual() * other.real();
        const Quat<qScalar>& real_ = this->real() * other.real();
        return UnitDualQuat( real_, dual_ );
    } 
    // Delete 
    template<typename Scalar>
    inline DualQuat<qScalar>& operator+=(const DualQuat<Scalar>& other) noexcept =delete;
    template<typename Scalar>
    inline DualQuat<qScalar>& operator-=(const DualQuat<Scalar>& other) noexcept =delete;
    template<typename Scalar>
    inline DualQuat<qScalar>& operator*=(const DualQuat<Scalar>& other) noexcept =delete;
    inline DualQuat<qScalar>& operator*=(const qScalar scalar) noexcept =delete;
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
    // Scalar Constructor
    explicit UnitPureDualQuat(const qScalar x1, const qScalar y1=0, const qScalar z1=0, 
                              const qScalar x2=0, const qScalar y2=0, const qScalar z2=0) noexcept
    : DualQuat<qScalar>( 0, x1, y1, z1, 0, x2, y2, z2 ) {
        this->normalize();
    }
    // Real Constructor
    template<typename Scalar>
    explicit UnitPureDualQuat(const Quat<Scalar>& real) noexcept
    : DualQuat<qScalar>{ real } {

    }
    // Real-Dual Constructor
    template<typename Scalar1, typename Scalar2>
    explicit UnitPureDualQuat(const PureQuat<Scalar1>& real, const PureQuat<Scalar2>& dual) noexcept
    : DualQuat<qScalar>( real, dual ) {
        this->normalize();
    }
    // DualQuat Constructor
    template <typename Scalar>
    explicit UnitPureDualQuat(const DualQuat<Scalar>& other) noexcept
    : DualQuat<qScalar>( other ) {
        this->purify();
        this->normalize();
    }
    // DualQuat Assignment
    template<typename Scalar>
    inline UnitPureDualQuat& operator=(const DualQuat<Scalar>& other) noexcept {
        this->real() = other.real();
        this->dual() = other.dual();
        this->purify();
        this->normalize();
        return *this;
    }
    // Delete 
    template<typename Scalar>
    inline DualQuat<qScalar>& operator+=(const DualQuat<Scalar>& other) noexcept =delete;
    template<typename Scalar>
    inline DualQuat<qScalar>& operator-=(const DualQuat<Scalar>& other) noexcept =delete;
    template<typename Scalar>
    inline DualQuat<qScalar>& operator*=(const DualQuat<Scalar>& other) noexcept =delete;
    inline DualQuat<qScalar>& operator*=(const qScalar scalar) noexcept =delete;
                    UnitPureDualQuat()=delete;
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
    return DualQuat( quat * dq.real(), quat * dq.dual() );
}

using DualQuatf = DualQuat<float>;
using UnitDualQuatf = UnitDualQuat<float>;
using PureDualQuatf = PureDualQuat<float>;
using UnitPureDualQuatf = UnitPureDualQuat<float>;
using DualQuatd = DualQuat<double>;
using UnitDualQuatd = UnitDualQuat<double>;
using PureDualQuatd = PureDualQuat<double>;
using UnitPureDualQuatd = UnitPureDualQuat<double>;
using DualQuatld = DualQuat<long double>;
using UnitDualQuatld = UnitDualQuat<long double>;
using PureDualQuatld = PureDualQuat<long double>;
using UnitPureDualQuatld = UnitPureDualQuat<long double>;
}