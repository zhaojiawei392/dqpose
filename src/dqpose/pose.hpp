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
 *     \brief A header file defining 3-D pose operations
 * 
 *     This file provides the necessary classes and functions to represent 
 *     and manipulate Dual-Quat-based Rotation, Translation, Pose.
 * 
 *     \cite https://github.com/zhaojiawei392/dqpose.git
 */

#pragma once

#include "quat.hpp"
#include "dualquat.hpp"

namespace dqpose
{

template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class Rotation;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class Translation;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class UnitAxis;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class Pose;

template<typename qScalar, typename>
class Rotation : public UnitQuat<qScalar> 
{
public:
    explicit Rotation() noexcept
        : UnitQuat<qScalar>(1) {

    }
    template<typename Scalar>
    explicit Rotation(const UnitAxis<Scalar>& rotate_axis, const qScalar rotate_angle) noexcept
        : UnitQuat<qScalar>(1) {
        this->w() = cos(0.5 * rotate_angle);
        const qScalar sin_ = sin(0.5 * rotate_angle);
        this->x() = rotate_axis.x() * sin_;
        this->y() = rotate_axis.y() * sin_;
        this->z() = rotate_axis.z() * sin_;
    }

    // Default
        virtual ~Rotation()=default;
                Rotation(const Rotation&)=default;
                Rotation(Rotation&&)=default;
    Rotation& operator=(const Rotation&)=default;
    Rotation& operator=(Rotation&&)=default;
};

template<typename qScalar, typename>
class Translation : public PureQuat<qScalar> {
public:
    explicit Translation() noexcept
        : PureQuat<qScalar>(0) {

    }
    explicit Translation(const qScalar x, const qScalar y=0, const qScalar z=0)
        : PureQuat<qScalar>(x, y, z) {

    }
    template<typename Scalar>
    explicit Translation(const Quat<Scalar>& quat) 
        : PureQuat<qScalar>(quat) {

    }

    template<typename Scalar>
    inline Translation& active_rotate(const Rotation<Scalar>& rotation) noexcept {
        PureQuat<qScalar>::operator=(rotation * *this * rotation.conj());
        return *this;
    }    
    
    template<typename Scalar>
    inline Translation& passive_rotate(const Rotation<Scalar>& rotation) noexcept {
        PureQuat<qScalar>::operator=(rotation.conj() * *this * rotation);
        return *this;
    }

    template<typename Scalar>
    inline Translation active_rotated(const Rotation<Scalar>& rotation) const noexcept {
        return Translation(rotation * *this * rotation.conj());
    }    
    
    template<typename Scalar>
    inline Translation passive_rotated(const Rotation<Scalar>& rotation) const noexcept {
        return Translation(rotation.conj() * *this * rotation);
    }

    // Default
        virtual ~Translation()=default;
                Translation(const Translation&)=default;
                Translation(Translation&&)=default;
    Translation& operator=(const Translation&)=default;
    Translation& operator=(Translation&&)=default;
};   

template<typename qScalar, typename>
class UnitAxis {
protected:
    UnitPureQuat<qScalar> _quat; 
public:

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitAxis(const qScalar x, const qScalar y, const qScalar z)
    : _quat(x, y, z) {

    }

    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit UnitAxis(const Quat<qScalar>& quat)
    : _quat(quat) {

    }

    inline UnitAxis& active_rotate(const Rotation& rotation) {
    }    
    
    inline UnitAxis& passive_rotate(const Rotation& rotation) {
    }

    inline UnitAxis active_rotated(const Rotation& rotation) const {
    }    
    
    inline UnitAxis passive_rotated(const Rotation& rotation) const {
    }

    // Default
        virtual ~UnitAxis()=default;
                UnitAxis()=default; // {0,0,0}
                UnitAxis(const UnitAxis&)=default;
                UnitAxis(UnitAxis&&)=default;
    UnitAxis& operator=(const UnitAxis&)=default;
    UnitAxis& operator=(UnitAxis&&)=default;
};    

template<typename qScalar, typename>
class Pose {
protecte:
    UnitDualQuat<qScalar> _dq;
public:
    template <typename T = qScalar, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit Pose() 
    : _dq() {

    }
    explicit Pose(const UnitDualQuat& udq) 
    : _dq(udq) {

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
