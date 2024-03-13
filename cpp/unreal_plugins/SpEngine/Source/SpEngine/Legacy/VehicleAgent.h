//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <memory> // std::unique_ptr
#include <string>
#include <vector>

#include "SpEngine/Legacy/Agent.h"
#include "SpEngine/Legacy/ClassRegistrationUtils.h"
#include "SpCore/ArrayDesc.h"

class UWorld;

class AVehiclePawn;
class CameraSensor;
class ImuSensor;

class VehicleAgent : public Agent
{
public:
    VehicleAgent() = delete;
    VehicleAgent(UWorld* world);
    ~VehicleAgent();

    void findObjectReferences(UWorld* world) override;
    void cleanUpObjectReferences() override;

    std::map<std::string, ArrayDesc> getActionSpace() const override;
    std::map<std::string, ArrayDesc> getObservationSpace() const override;
    std::map<std::string, ArrayDesc> getStepInfoSpace() const override;

    void applyAction(const std::map<std::string, std::vector<uint8_t>>& action) override;
    std::map<std::string, std::vector<uint8_t>> getObservation() const override;
    std::map<std::string, std::vector<uint8_t>> getStepInfo() const override;

    void reset() override;
    bool isReady() const override;

private:
    AVehiclePawn* vehicle_pawn_ = nullptr;

    std::unique_ptr<CameraSensor> camera_sensor_;
    std::unique_ptr<ImuSensor> imu_sensor_;

    inline static auto s_class_registration_handler_ = ClassRegistrationUtils::registerClass<VehicleAgent>(Agent::s_class_registrar_, "VehicleAgent");
};
