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
 *     \file src/dqpose/pose.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2024-2025
 *
 *     \brief A header file defining 3-D poses
 * 
 *     This file provides the necessary classes and functions to represent 
 *     and manipulate Dual-Quaternion-based Rotation, Translation, Pose.
 * 
 *     \cite https://github.com/zhaojiawei392
 */

#pragma once

#include "quat.hpp"
#include "dualquat.hpp"

namespace dqpose
{

template<typename qScalar>
class Rotation;
template<typename qScalar>
class Translation;
template<typename qScalar>
class Pose;
template<typename qScalar>
class UnitAxis;

template<typename qScalar>
class Rotation {
using Arr3 = Arr<qScalar, 3>;
using Arr4 = Arr<qScalar, 4>;
using Arr8 = Arr<qScalar, 8>;
using Mat3 = Mat<qScalar, 3,3>;
using Mat4 = Mat<qScalar, 4,4>;
using Mat8 = Mat<qScalar, 8,8>;
protected:
    UnitQuaternion<qScalar> _quat; 
public:
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Rotation()
    : _quat() {// default UnitQuaternion is set to 1

    }

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Rotation(const Quaternion<qScalar>& quat)
    : _quat(quat) {

    }

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Rotation(const UnitAxis& rotate_axis, const qScalar rotate_angle) {
        const qScalar w_ = cos(0.5 * rotate_angle);
        const qScalar sin_ = sin(0.5 * rotate_angle);
        const qScalar x_ = rotate_axis.x() * sin_;
        const qScalar y_ = rotate_axis.y() * sin_;
        const qScalar z_ = rotate_axis.z() * sin_;

        const UnitQuaternion<qScalar> uquat(w_, x_, y_, z_);
        return Rotation(uquat);
    }

    inline Rotation& operator*=(const Rotation& other) noexcept {
        _quat *= other._quat;
        return *this;
    }

    inline Rotation operator*(const Rotation& other) const noexcept {
        return Rotation(_quat * other._quat);
    }

    static Rotation gap_between(const UnitAxis& uaxis1, const UnitAxis& uaxis2) {
        if ()
    }    

    inline UnitQuaternion<qScalar> quaternion() const noexcept { return _quat; }

    inline Rotation conj() const noexcept {
        return Rotation(_quat.conj());
    }

    // Default
        virtual ~Rotation()=default;
                Rotation(const Rotation&)=default;
                Rotation(Rotation&&)=default;
    Rotation& operator=(const Rotation&)=default;
    Rotation& operator=(Rotation&&)=default;
};

template<typename qScalar>
class Translation {
using Arr3 = Arr<qScalar, 3>;
using Arr4 = Arr<qScalar, 4>;
using Arr8 = Arr<qScalar, 8>;
using Mat3 = Mat<qScalar, 3,3>;
using Mat4 = Mat<qScalar, 4,4>;
using Mat8 = Mat<qScalar, 8,8>;
protected:
    PureQuaternion<qScalar> _quat; 
public:

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Translation(const qScalar x, const qScalar y, const qScalar z)
    : _quat(x, y, z) {

    }

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Translation(const Quaternion<qScalar>& quat)
    : _quat(quat) {

    }

    inline Translation& active_rotate(const Rotation& rotation) {
        const Quaternion<qScalar>& rotation_quat = rotation.quaternion();
        _quat = rotation_quat * _quat * rotation_quat.conj();
        return &this;
    }    
    
    inline Translation& passive_rotate(const Rotation& rotation) {
        const Quaternion<qScalar>& rotation_quat = rotation.quaternion();
        _quat = rotation_quat.conj() * _quat * rotation_quat;
        return &this;
    }

    inline Translation active_rotated(const Rotation& rotation) const {
        const Quaternion<qScalar>& rotation_quat = rotation.quaternion();
        const Quaternion<qScalar>& res = rotation_quat * _quat * rotation_quat.conj();
        return Translation(res);
    }    
    
    inline Translation passive_rotated(const Rotation& rotation) const {
        cosnt Quaternion<qScalar>& rotation_quat = rotation.quaternion();
        const Quaternion<qScalar>& res = rotation_quat.conj() * _quat * rotation_quat;
        return Translation(res);
    }

    inline Translation& operator+=(const Translation& other) noexcept {
        _quat += other._quat;
        return *this;
    }

    inline Translation& operator-=(const Translation& other) noexcept {
        _quat -= other._quat;
        return *this;
    }

    inline Translation operator+(const Translation& other) const noexcept {
        return Translation(_quat + other._quat);
    }

    inline Translation operator-(const Translation& other) const noexcept  {
        return Translation(_quat - other._quat);
    }

    inline PureQuaternion<qScalar> quaternion() const noexcept { return _quat; }

    inline Translation conj() const noexcept {
        return Translation(_quat.conj());
    }
    // Default
        virtual ~Translation()=default;
                Translation()=default; // {0,0,0}
                Translation(const Translation&)=default;
                Translation(Translation&&)=default;
    Translation& operator=(const Translation&)=default;
    Translation& operator=(Translation&&)=default;
};   

template<typename qScalar>
class UnitAxis {
using Arr3 = Arr<qScalar, 3>;
using Arr4 = Arr<qScalar, 4>;
using Arr8 = Arr<qScalar, 8>;
using Mat3 = Mat<qScalar, 3,3>;
using Mat4 = Mat<qScalar, 4,4>;
using Mat8 = Mat<qScalar, 8,8>;
protected:
    UnitPureQuaternion<qScalar> _quat; 
public:

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitAxis(const qScalar x, const qScalar y, const qScalar z)
    : _quat(x, y, z) {

    }

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitAxis(const Quaternion<qScalar>& quat)
    : _quat(quat) {

    }

    inline UnitAxis& active_rotate(const Rotation& rotation) {
        const Quaternion<qScalar>& rotation_quat = rotation.quaternion();
        _quat = rotation_quat * _quat * rotation_quat.conj();
        return &this;
    }    
    
    inline UnitAxis& passive_rotate(const Rotation& rotation) {
        const Quaternion<qScalar>& rotation_quat = rotation.quaternion();
        _quat = rotation_quat.conj() * _quat * rotation_quat;
        return &this;
    }

    inline UnitAxis active_rotated(const Rotation& rotation) const {
        const Quaternion<qScalar>& rotation_quat = rotation.quaternion();
        const Quaternion<qScalar>& res = rotation_quat * _quat * rotation_quat.conj();
        return UnitAxis(res);
    }    
    
    inline UnitAxis passive_rotated(const Rotation& rotation) const {
        cosnt Quaternion<qScalar>& rotation_quat = rotation.quaternion();
        const Quaternion<qScalar>& res = rotation_quat.conj() * _quat * rotation_quat;
        return UnitAxis(res);
    }

    inline UnitPureQuaternion<qScalar> quaternion() const noexcept { return _quat; }

    inline UnitAxis conj() const noexcept {
        return UnitAxis(_quat.conj());
    }
    // Default
        virtual ~UnitAxis()=default;
                UnitAxis()=default; // {0,0,0}
                UnitAxis(const UnitAxis&)=default;
                UnitAxis(UnitAxis&&)=default;
    UnitAxis& operator=(const UnitAxis&)=default;
    UnitAxis& operator=(UnitAxis&&)=default;
};    

template<typename qScalar>
class Pose {

using Arr3 = Arr<qScalar, 3>;
using Arr4 = Arr<qScalar, 4>;
using Arr8 = Arr<qScalar, 8>;
using Mat3 = Mat<qScalar, 3,3>;
using Mat4 = Mat<qScalar, 4,4>;
using Mat8 = Mat<qScalar, 8,8>;
protecte:
    UnitDualQuaternion<qScalar> _dq;
public:
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Pose() 
    : _dq() {

    }
    explicit Pose(const UnitDualQuaternion& udq) 
    : _dq(udq) {

    }

    inline Pose& operator*=(const Pose& other) noexcept {
        _dq *= other._dq;
    }

    inline Pose operator*(const Pose& other) noexcept {
        return Pose(_dq * other._dq);
    }

    template<typename First_, typename... Args_>
    static Pose build_from(const First_& first, const Args_&... args){
        return Pose(build_from(first) * build_from(args...));
    }  
    static Pose build_from(const Rotation<qScalar_>& rotation){
        return Pose(rotation);
    }
    static Pose build_from(const Translation<qScalar_>& translation){
        return Pose(Rotation<qScalar_>(1), translation / 2);
    }
    static Pose build_from(const Pose& pose){
        return pose;
    }

};


}  // namespace dqpose
