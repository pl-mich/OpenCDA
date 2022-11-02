from opencda.core.common.v2x_manager import V2XManager
from opencda.customize.core.platooning.platooning_plugin \
    import CustomizedPlatooningPlugin


class CustomizedV2XManager(V2XManager):
    def __init__(self, cav_world, config_yaml, vid):
        super(CustomizedV2XManager, self).__init__(
            cav_world, config_yaml, vid
        )
        self.v2v_count = config_yaml['v2v_count']

        # self.platooning_plugin = CustomizedPlatooningPlugin(
        #     self.v2v_count, self.cda_enabled
        # )

    # def get_platoon_front_rear_mult(self):
    #     # return multiple vehicles, number specified in yaml file
    #     return self.platooning_plugin.frontal_vehicles, \
    #         self.platooning_plugin.rear_vehicles

    def get_platoon_front_rear_mult(self):
        # remember linked lists?
        curr_front, curr_rear = self.get_platoon_front_rear_helper()
        front_vehicles, rear_vehicles = [curr_front], [curr_rear]
        for i in range(self.v2v_count - 1):
            if not curr_front:
                break
            curr_front, _ = curr_front.v2x_manager.get_platoon_front_rear_helper()
            front_vehicles.append(curr_front)
        for i in range(self.v2v_count - 1):
            if not curr_rear:
                break
            _, curr_rear = curr_rear.v2x_manager.get_platoon_front_rear_helper()
            rear_vehicles.append(curr_rear)
        assert (front_vehicles is not None) or (rear_vehicles is not None)
        # print(front_vehicles, rear_vehicles)
        return front_vehicles, rear_vehicles

    def get_platoon_front_rear(self):
        front, rear = self.get_platoon_front_rear_mult()
        return front[0], rear[0]

    def get_platoon_front_rear_helper(self):
        return self.platooning_plugin.front_vehicle, \
            self.platooning_plugin.rear_vehicle
