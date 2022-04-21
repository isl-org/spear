#pragma once

#include <map>
#include <string>
#include <vector>

struct Box;

class AgentController
{
public:

    AgentController() = default;
    virtual ~AgentController() = default;

    virtual std::map<std::string, Box> getActionSpace() const = 0;
    virtual std::map<std::string, Box> getObservationSpace() const = 0;
    virtual void applyAction(const std::map<std::string, std::vector<float>>& action) = 0;
    virtual std::map<std::string, std::vector<uint8_t>> getObservation() const = 0;
};
