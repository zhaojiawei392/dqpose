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
 *     \file include/dqpose/pose.hpp
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
#include <cstdint>

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
    // Default Constructor 
    constexpr explicit Rotation() noexcept
        : UnitQuat<qScalar>() {

    }
    // Scalar Constructor
    constexpr explicit Rotation(const qScalar w, const qScalar x=0, const qScalar y=0, const qScalar z=0) noexcept
        : UnitQuat<qScalar>( w, x, y, z ) {

    }
    // Axis-Angle Constructor 
    template<typename Scalar>
    constexpr explicit Rotation(const UnitAxis<Scalar>& rotate_axis, const qScalar rotate_angle) noexcept
        : UnitQuat<qScalar>(1) {
        this->_w() = cos(0.5 * rotate_angle);
        const qScalar sin_ = sin(0.5 * rotate_angle);
        this->_x() = rotate_axis.x() * sin_;
        this->_y() = rotate_axis.y() * sin_;
        this->_z() = rotate_axis.z() * sin_;
    }
    // Quat Constructor
    template<typename Scalar>
    constexpr Rotation(const Quat<Scalar>& other) 
        : UnitQuat<qScalar>(other) {

    }
    // Copy Constructor 
    template<typename Scalar>
    constexpr Rotation(const Rotation<Scalar>& other) noexcept
        : UnitQuat<qScalar>(other) {
    }
    // Copy Assignment 
    template<typename Scalar>
    constexpr inline Rotation& operator=(const Rotation<Scalar>& other) noexcept {
        UnitQuat<qScalar>::operator=(other);
        return *this;
    }
    // rotation_axis
    constexpr inline UnitAxis<qScalar> rotation_axis() const noexcept {
        const qScalar vec3_norm = std::sqrt( square( this->x() ) + square( this->y() ) + square( this->z() ) );
        if (vec3_norm == 0){
            return UnitAxis<qScalar>(0,0,1);
        }
        const qScalar x_ = this->x() / vec3_norm;
        const qScalar y_ = this->y() / vec3_norm;
        const qScalar z_ = this->z() / vec3_norm;
        return UnitAxis<qScalar>(x_, y_, z_);
    }
    // rotation_angle
    constexpr inline qScalar rotation_angle() const noexcept {
        return 2 * acos(this->w() / this->norm());
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
    // Default Constructor
    constexpr explicit Translation() noexcept
        : PureQuat<qScalar>( ) {

    }
    // Scalar Constructor
    constexpr explicit Translation(const qScalar x, const qScalar y=0, const qScalar z=0)
        : PureQuat<qScalar>(x, y, z) {

    }
    // Quat Constructor
    template<typename Scalar>
    constexpr Translation(const Quat<Scalar>& other) 
        : PureQuat<qScalar>(other) {

    }
    // Copy Constructor 
    template<typename Scalar>
    constexpr Translation(const Translation<Scalar>& other) noexcept
        : PureQuat<qScalar>(other) {
    }
    // Copy Assignment 
    template<typename Scalar>
    constexpr inline Translation& operator=(const Translation<Scalar>& other) noexcept {
        PureQuat<qScalar>::operator=(other);
        return *this;
    }
    // active_rotate 
    template<typename Scalar>
    constexpr inline Translation& active_rotate(const Rotation<Scalar>& rotation) noexcept {
        PureQuat<qScalar>::operator=(rotation * *this * rotation.conj());
        return *this;
    }
    // passive_rotate 
    template<typename Scalar>
    constexpr inline Translation& passive_rotate(const Rotation<Scalar>& rotation) noexcept {
        PureQuat<qScalar>::operator=(rotation.conj() * *this * rotation);
        return *this;
    }
    // active_rotated
    template<typename Scalar>
    constexpr inline Translation active_rotated(const Rotation<Scalar>& rotation) const noexcept {
        return Translation(rotation * *this * rotation.conj());
    }    
    // passive_rotated
    template<typename Scalar>
    constexpr inline Translation passive_rotated(const Rotation<Scalar>& rotation) const noexcept {
        return Translation(rotation.conj() * *this * rotation);
    }
    // perpendicular
    template<typename Scalar>
    constexpr inline UnitAxis<qScalar> perpendicular(const Translation<Scalar>& other) const noexcept {
        const qScalar axis_x = this->y() * other.z() - this->z() * other.y();
        const qScalar axis_y = this->z() * other.x() - this->x() * other.z();
        const qScalar axis_z = this->x() * other.y() - this->y() * other.x();
        return UnitAxis(axis_x, axis_y, axis_z);
    }
    // angle
    template<typename Scalar>
    constexpr inline qScalar angle(const Translation<Scalar>& other) const noexcept {
        return std::acos(this->normalized().dot(other.normalized()));
    }
    // Default
        virtual ~Translation()=default;
                Translation(const Translation&)=default;
                Translation(Translation&&)=default;
    Translation& operator=(const Translation&)=default;
    Translation& operator=(Translation&&)=default;
};   

template<typename qScalar, typename>
class UnitAxis : public UnitPureQuat<qScalar> {
public:
    // Scalar Constructor
    constexpr explicit UnitAxis(const qScalar x, const qScalar y, const qScalar z)
        : UnitPureQuat<qScalar>(x,y,z) {

    }
    // Quat Constructor
    template<typename Scalar>
    constexpr UnitAxis(const Quat<Scalar>& other)
        : UnitPureQuat<qScalar>(other) {

    }
    // Copy Constructor 
    template<typename Scalar>
    constexpr UnitAxis(const UnitAxis<Scalar>& other) noexcept
        : UnitPureQuat<qScalar>(other) {
    }
    // Copy Assignment 
    template<typename Scalar>
    constexpr inline UnitAxis& operator=(const UnitAxis<Scalar>& other) noexcept {
        UnitPureQuat<qScalar>::operator=(other);
        return *this;
    }
    // active_rotate 
    template<typename Scalar>
    constexpr inline UnitAxis& active_rotate(const Rotation<Scalar>& rotation) noexcept {
        UnitPureQuat<qScalar>::operator=(rotation * *this * rotation.conj());
        return *this;
    }
    // passive_rotate 
    template<typename Scalar>
    constexpr inline UnitAxis& passive_rotate(const Rotation<Scalar>& rotation) noexcept {
        UnitPureQuat<qScalar>::operator=(rotation.conj() * *this * rotation);
        return *this;
    }
    // active_rotated
    template<typename Scalar>
    constexpr inline UnitAxis active_rotated(const Rotation<Scalar>& rotation) const noexcept {
        return UnitAxis(rotation * *this * rotation.conj());
    }    
    // passive_rotated
    template<typename Scalar>
    constexpr inline UnitAxis passive_rotated(const Rotation<Scalar>& rotation) const noexcept {
        return UnitAxis(rotation.conj() * *this * rotation);
    }
    // perpendicular
    template<typename Scalar>
    constexpr inline UnitAxis perpendicular(const UnitAxis<Scalar>& other) const noexcept {
        const qScalar axis_x = this->y() * other.z() - this->z() * other.y();
        const qScalar axis_y = this->z() * other.x() - this->x() * other.z();
        const qScalar axis_z = this->x() * other.y() - this->y() * other.x();
        return UnitAxis(axis_x, axis_y, axis_z);
    }
    // angle
    template<typename Scalar>
    constexpr inline qScalar angle(const UnitAxis<Scalar>& other) const noexcept {
        return std::acos(this->dot(other));
    }
    // rotation_to
    template<typename Scalar>
    constexpr inline Rotation<qScalar> rotation_to(const UnitAxis<Scalar>& other) const noexcept {
        const UnitAxis axis = perpendicular(other);
        const qScalar angle = angle(other);
        return Rotation<qScalar>(axis, angle);
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
class Pose : public UnitDualQuat<qScalar> {
public:
    // Default Constructor 
    constexpr explicit Pose() noexcept
        : UnitDualQuat<qScalar>() {

    }
    // Scalar Constructor
    constexpr explicit Pose(const qScalar w1, const qScalar x1=0, const qScalar y1=0, const qScalar z1=0, 
                      const qScalar w2=0, const qScalar x2=0, const qScalar y2=0, const qScalar z2=0) noexcept
        : UnitDualQuat<qScalar>( w1, x1, y1, z1, w2, x2, y2, z2 ) {

    }
    // Rotation-Translation Constructor 
    template<typename Scalar1, typename Scalar2>
    constexpr explicit Pose(const Rotation<Scalar1>& rotation, const Translation<Scalar2> translation) noexcept
        : UnitDualQuat<qScalar>(rotation, translation * rotation * 0.5) {
    }
    // Rotation Constructor
    template<typename Scalar>
    constexpr explicit Pose(const Rotation<Scalar>& rotation) 
        : UnitDualQuat<qScalar>(static_cast<Quat<qScalar>>(rotation)) {

    }
    // Translation Constructor
    template<typename Scalar>
    constexpr explicit Pose(const Translation<Scalar> translation) 
        : UnitDualQuat<qScalar>(Quat<qScalar>(1), translation * 0.5) {

    }
    // DualQuat Constructor 
    template<typename Scalar>
    constexpr Pose(const DualQuat<Scalar>& other) noexcept
        : UnitDualQuat<qScalar>(other) {
    }
    // Copy Constructor 
    template<typename Scalar>
    constexpr Pose(const Pose<Scalar>& other) noexcept
        : UnitDualQuat<qScalar>(other) {
    }
    // Copy Assignment 
    template<typename Scalar>
    constexpr inline Pose& operator=(const Pose<Scalar>& other) noexcept {
        UnitDualQuat<qScalar>::operator=(other);
        return *this;
    }

    constexpr Rotation<qScalar> rotation() const noexcept { return Rotation<qScalar>(this->real()); }
    constexpr Translation<qScalar> translation() const noexcept { return Translation<qScalar>(this->dual() * this->real().conj() * 2); }

    template<typename First_, typename... Args_>
    constexpr static Pose build_from(const First_& first, const Args_&... args){
        return Pose(build_from(first) * build_from(args...));
    }  
    template<typename Scalar>
    constexpr static Rotation<qScalar> build_from(const Rotation<Scalar>& rotation){
        return static_cast<Rotation<qScalar>>( rotation );
    }
    template<typename Scalar>
    constexpr static Pose build_from(const Translation<Scalar>& translation){
        return Pose(Rotation<qScalar>(), translation * 0.5);
    }
    template<typename Scalar>
    constexpr static Pose build_from(const Pose<Scalar>& pose){
        return Pose<qScalar>(pose);
    }
    // Default
        virtual ~Pose()=default;
                Pose(const Pose&)=default;
                Pose(Pose&&)=default;
    Pose& operator=(const Pose&)=default;
    Pose& operator=(Pose&&)=default;
};

using Rotf = Rotation<float>;
using Tranf = Translation<float>;
using Unitf = UnitAxis<float>;
using Posef = Pose<float>;
using Rotd = Rotation<double>;
using Trand = Translation<double>;
using Unitd = UnitAxis<double>;
using Posed = Pose<double>;
using Rotld = Rotation<long double>;
using Tranld = Translation<long double>;
using Unitld = UnitAxis<long double>;
using Poseld = Pose<long double>;

constexpr UnitAxis<std::uint8_t> i_(1,0,0);
constexpr UnitAxis<std::uint8_t> j_(0,1,0);
constexpr UnitAxis<std::uint8_t> k_(0,0,1);

}  // namespace dqpose
