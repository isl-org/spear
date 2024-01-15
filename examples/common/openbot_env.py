#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import gym
import numpy as np
import spear


# Custom Env implementation for OpenBot
class OpenBotEnv(spear.Env):

    def __init__(self, config, simulation_controller):
    
        assert config.SIMULATION_CONTROLLER.AGENT == "VehicleAgent"
        assert "set_drive_torques" in config.SIMULATION_CONTROLLER.VEHICLE_AGENT.ACTION_COMPONENTS
        assert "wheel_rotation_speeds" in config.SIMULATION_CONTROLLER.VEHICLE_AGENT.OBSERVATION_COMPONENTS

        self._wheel_rotation_speeds = None

        super().__init__(config, simulation_controller)

        # In this derived class, we are expecting different actions than the base spear.Env class. So
        # we need to override self.action_space. We need to do this after calling super().__init__(...),
        # because the base spear.Env class will set self.action_space to the lower-level actions it is
        # expecting internally.
        self.action_space = gym.spaces.Dict(spaces={"set_duty_cycles": gym.spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float64)})
    
    def reset(self, reset_info=None):
    
        obs = super().reset(reset_info=reset_info)
        assert "wheel_rotation_speeds" in obs.keys()
        self._wheel_rotation_speeds = obs["wheel_rotation_speeds"]
        return obs

    def _get_observation(self):

        obs = super()._get_observation()
        assert "wheel_rotation_speeds" in obs.keys()
        self._wheel_rotation_speeds = obs["wheel_rotation_speeds"]
        return obs
        
    def _apply_action(self, action):

        assert "set_duty_cycles" in action.keys()
        assert action["set_duty_cycles"].shape[0] == 2
        assert self._wheel_rotation_speeds is not None

        duty_cycles = np.array([action["set_duty_cycles"][0], action["set_duty_cycles"][1], action["set_duty_cycles"][0], action["set_duty_cycles"][1]], dtype=np.float64)

        motor_velocity_constant = self._config.OPENBOT_ENV.MOTOR_VELOCITY_CONSTANT # Motor torque constant in [N.m/A]
        gear_ratio              = self._config.OPENBOT_ENV.GEAR_RATIO              # Gear ratio of the OpenBot motors
        motor_torque_constant   = 1.0 / motor_velocity_constant                    # Motor torque constant in [rad/s/V]

        battery_voltage         = self._config.OPENBOT_ENV.BATTERY_VOLTAGE         # Voltage of the battery powering the OpenBot [V]
        control_dead_zone       = self._config.OPENBOT_ENV.CONTROL_DEAD_ZONE       # Absolute duty cycle (in the ACTION_SCALE range) below which a command does not
                                                                                   # produce any torque on the vehicle
        motor_torque_max        = self._config.OPENBOT_ENV.MOTOR_TORQUE_MAX        # Motor maximal torque [N.m]
        electrical_resistance   = self._config.OPENBOT_ENV.ELECTRICAL_RESISTANCE   # Motor winding electrical resistance [Ohms]

        # Speed multiplier defined in the OpenBot to map a [-1,1] action to a suitable command to 
        # be processed by the low-level microcontroller. For more details, feel free to check the
        # "speedMultiplier" command in the OpenBot code.
        #     https://github.com/isl-org/OpenBot/blob/master/android/app/src/main/java/org/openbot/vehicle/Vehicle.java#L375
        action_scale = self._config.OPENBOT_ENV.ACTION_SCALE

        # Acquire the ground truth motor and wheel velocity for motor counter-electromotive force
        # computation purposes (or alternatively friction computation purposes).

        # The ground truth velocity of the robot wheels in [rad/s]
        wheel_rotation_speeds = self._wheel_rotation_speeds

        # The ground truth rotation speed of the motors in [rad/s]
        motor_speeds = gear_ratio * wheel_rotation_speeds

        # Compute the counter electromotive force using the motor torque constant
        counter_electromotive_forces = motor_torque_constant * motor_speeds # Expressed in [V]

        # The electrical current allowed to circulate in the motor is the result of the
        # difference between the applied voltage and the counter electromotive force
        motor_winding_currents = ((battery_voltage * duty_cycles) - counter_electromotive_forces) / electrical_resistance # Expressed in [A]

        # The torque is then obtained using the torque coefficient of the motor in [N.m]
        motor_torques = motor_torque_constant * motor_winding_currents

        # Motor torque is saturated to match the motor limits
        motor_torques = np.clip(motor_torques, -motor_torque_max, motor_torque_max)

        # The torque applied to the robot wheels is finally computed accounting for the gear ratio
        drive_torques = gear_ratio * motor_torques

        # Control dead zone at near-zero speed. This is a simplified but reliable way to deal with
        # the friction behavior observed on the real vehicle in the low-speed/low-duty-cycle regime.
        # For simplicity, we hardcode the threshold value 1e-5, but in future we should get this
        # value from the config system.
        drive_torques[np.logical_and(abs(motor_speeds) < 1e-5, abs(duty_cycles) <= control_dead_zone / action_scale)] = 0.0

        # formulate action before sending it to the simulator
        super()._apply_action(action={"set_drive_torques": drive_torques})
