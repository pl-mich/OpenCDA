import carla.libcarla
from opencda.core.plan.behavior_agent import BehaviorAgent


class CustomizedBehaviorAgent(BehaviorAgent):
    def __init__(self, vehicle, carla_map, config_yaml):
        super().__init__(vehicle, carla_map, config_yaml)
        pass

    def car_following_manager(self, vehicle, distance, target_speed=None):
        return target_speed

    