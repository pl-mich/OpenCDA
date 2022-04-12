import carla.libcarla
from opencda.core.plan.behavior_agent import BehaviorAgent

class CustomizedBehaviorAgent(BehaviorAgent):
    """
    During simulation runtime, BehaviorAgent first saves the ego vehicle position, speed and surrounding objects information from PerceptionManager and LocalizationManager through function update_information. Afterwards, the BehaviorAgent will call function run_step() to execute a single step and return the target_speed and target_location.
    """
    
    def __init__(self, vehicle, carla_map, config_yaml):
        ...

    def update_information(self, ego_pos, ego_speed, objects):
        ########################################
        # this is where you put your algorithm #
        ########################################
        do_some_preprocessing(ego_pos, ego_speed, objects)

    def run_step(self):
        ########################################
        # this is where you put your algorithm #
        ########################################
        target_speed, target_loc = your_plan_algorithm()
        assert type(target_speed) == float
        assert type(target_loc) == carla.Location
        return target_speed, target_loc

    def do_some_preprocessing(self):
        pass

    def your_plan_algorithm(self):
        pass
