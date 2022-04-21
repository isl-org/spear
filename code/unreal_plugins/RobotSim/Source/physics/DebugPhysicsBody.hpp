// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "PhysicsBody.hpp"
#include <exception>
#include <iostream>

namespace RobotSim {

    /**
     * @brief
     *
     */
    class DebugPhysicsBody : public PhysicsBody {
    private:
        class WrenchVertex : public PhysicsBodyVertex {
        public:
            /**
             * @brief Construct a new Wrench Vertex object
             *
             * @param position
             * @param normal
             * @param force
             */
            WrenchVertex(const Vector3r& position, const Vector3r& normal, const Vector3r& force = Vector3r::Zero()) : PhysicsBodyVertex(position, normal), force_(force)
            {
            }

        protected:
            /**
             * @brief Set the Wrench object
             *
             * @param wrench
             */
            virtual void setWrench(Wrench& wrench) override
            {
                wrench.force = force_;
            }

        private:
            Vector3r force_;
        };

    public:
        /**
         * @brief
         *
         * @param initial_kinematic_state
         * @param environment
         */
        void initialize(const Kinematics::State& initial_kinematic_state, Environment* environment)
        {
            computeInertiaMatrix(inertia_, mass_, body_box_);
            createWrenchVertices(wrench_vertices_, body_box_.x(), body_box_.y(), body_box_.z(), mass_);
            createDragVertices(drag_vertices_, 1.3f, body_box_.x(), body_box_.y(), body_box_.z());

            PhysicsBody::initialize(mass_, inertia_, initial_kinematic_state, environment);
        }

        /**
         * @brief
         *
         */
        virtual void kinematicsUpdated() override
        {
            const auto& kinematics = this->getKinematics();
            std::cout << " Pos: " << VectorMath::toString(kinematics.pose.position);
            std::cout << " Ori: " << VectorMath::toString(kinematics.pose.orientation) << std::endl;
            std::cout << " Lin Vel: " << VectorMath::toString(kinematics.twist.linear);
            std::cout << " Ang Vel: " << VectorMath::toString(kinematics.twist.angular) << std::endl;
            std::cout << " ------------------------------------------------" << std::endl;
        }

        /**
         * @brief
         *
         * @return real_T
         */
        virtual real_T getRestitution() const override
        {
            return restitution_;
        }

        /**
         * @brief
         *
         * @return real_T
         */
        virtual real_T getFriction() const override
        {
            return friction_;
        }

        /**
         * @brief
         *
         * @return uint
         */
        virtual uint wrenchVertexCount() const override
        {
            return static_cast<uint>(wrench_vertices_.size());
        }

        /**
         * @brief
         *
         * @param index
         * @return PhysicsBodyVertex&
         */
        virtual PhysicsBodyVertex& getWrenchVertex(uint index) override
        {
            return wrench_vertices_.at(index);
        }

        /**
         * @brief
         *
         * @param index
         * @return const PhysicsBodyVertex&
         */
        virtual const PhysicsBodyVertex& getWrenchVertex(uint index) const override
        {
            return wrench_vertices_.at(index);
        }

        /**
         * @brief
         *
         * @return uint
         */
        virtual uint dragVertexCount() const override
        {
            return static_cast<uint>(drag_vertices_.size());
        }

        /**
         * @brief
         *
         * @param index
         * @return PhysicsBodyVertex&
         */
        virtual PhysicsBodyVertex& getDragVertex(uint index) override
        {
            return drag_vertices_.at(index);
        }

        /**
         * @brief 
         * 
         * @param index 
         * @return const PhysicsBodyVertex& 
         */
        virtual const PhysicsBodyVertex& getDragVertex(uint index) const override
        {
            return drag_vertices_.at(index);
        }

        /**
         * @brief 
         * 
         * @param index 
         * @return Vector3r 
         */
        Vector3r getShapeVertex(uint index) const
        {
            real_T x = (index & 1) == 0 ? body_box_.x() / 2 : -body_box_.x() / 2;
            real_T y = (index & 2) == 0 ? body_box_.y() / 2 : -body_box_.y() / 2;
            real_T z = (index & 4) == 0 ? body_box_.z() / 2 : -body_box_.z() / 2;

            return Vector3r(x, y, z);
        }

        /**
         * @brief 
         * 
         * @return uint 
         */
        uint shapeVertexCount()
        {
            return 8; // for box
        }

    private:

        /**
         * @brief 
         * 
         * @param drag_vertices 
         * @param drag_coefficient 
         * @param body_x 
         * @param body_y 
         * @param body_z 
         */
        static void createDragVertices(vector<PhysicsBodyVertex>& drag_vertices, real_T drag_coefficient, real_T body_x, real_T body_y, real_T body_z)
        {
            real_T top_bottom_area = body_x * body_y;
            real_T left_right_area = body_x * body_z;
            real_T front_back_area = body_y * body_z;
            Vector3r drag_factor_unit = Vector3r(front_back_area, left_right_area, top_bottom_area) * drag_coefficient / 2; // Vector3r drag_factor_unit = Vector3r::Zero();

            // add six drag vertices representing 6 sides
            drag_vertices.clear();
            drag_vertices.emplace_back(Vector3r(0, 0, -body_z), Vector3r(0, 0, -1), drag_factor_unit.z());
            drag_vertices.emplace_back(Vector3r(0, 0, body_z), Vector3r(0, 0, 1), drag_factor_unit.z());
            drag_vertices.emplace_back(Vector3r(0, -body_y, 0), Vector3r(0, -1, 0), drag_factor_unit.y());
            drag_vertices.emplace_back(Vector3r(0, body_y, 0), Vector3r(0, 1, 0), drag_factor_unit.y());
            drag_vertices.emplace_back(Vector3r(-body_x, 0, 0), Vector3r(-1, 0, 0), drag_factor_unit.x());
            drag_vertices.emplace_back(Vector3r(body_x, 0, 0), Vector3r(1, 0, 0), drag_factor_unit.x());
        }

        /**
         * @brief 
         * 
         * @param wrench_vertices 
         * @param body_x 
         * @param body_y 
         * @param body_z 
         * @param mass 
         */
        static void createWrenchVertices(vector<WrenchVertex>& wrench_vertices, real_T body_x, real_T body_y, real_T body_z, real_T mass)
        {
            wrench_vertices.clear();
            wrench_vertices.emplace_back(Vector3r(0, 0, -body_z), Vector3r(0, 0, -1), Vector3r(0, 0, -18 * mass));
            wrench_vertices.emplace_back(Vector3r(0, 0, body_z), Vector3r(0, 0, 1));
            wrench_vertices.emplace_back(Vector3r(0, -body_y, 0), Vector3r(0, -1, 0));
            wrench_vertices.emplace_back(Vector3r(0, body_y, 0), Vector3r(0, 1, 0));
            wrench_vertices.emplace_back(Vector3r(-body_x, 0, 0), Vector3r(-1, 0, 0));
            wrench_vertices.emplace_back(Vector3r(body_x, 0, 0), Vector3r(1, 0, 0));
        }

        // TODO: put in common place?
        /**
         * @brief 
         * 
         * @param inertia 
         * @param box_mass 
         * @param body_box 
         */
        static void computeInertiaMatrix(Matrix3x3r& inertia, real_T box_mass, const Vector3r& body_box)
        {
            inertia = Matrix3x3r::Zero();

            // http://farside.ph.utexas.edu/teaching/336k/Newtonhtml/node64.html
            inertia(0, 0) = box_mass / 12.0f * (body_box.y() * body_box.y() + body_box.z() * body_box.z());
            inertia(1, 1) = box_mass / 12.0f * (body_box.x() * body_box.x() + body_box.z() * body_box.z());
            inertia(2, 2) = box_mass / 12.0f * (body_box.x() * body_box.x() + body_box.y() * body_box.y());
        }

    private:
        Vector3r body_box_ = Vector3r(0.20f, 0.12f, 0.04f);
        real_T mass_ = 1.0f;
        real_T restitution_ = 0.5f;
        real_T friction_ = 0.7f;

        Matrix3x3r inertia_;

        vector<PhysicsBodyVertex> drag_vertices_;
        vector<WrenchVertex> wrench_vertices_;
    };

} // namespace
