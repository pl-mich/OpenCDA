from opencda.core.application.platooning.platooning_plugin \
    import PlatooningPlugin
import math


class CustomizedPlatooningPlugin(PlatooningPlugin):
    def __init__(self, v2v_count, cda_enabled):
        # search_range does NOT seem to do anything!
        super(CustomizedPlatooningPlugin, self).__init__(
            45, cda_enabled)
        self.frontal_vehicles = []
        self.rear_vehicles = []
        self.v2v_count = v2v_count

    def reset(self):
        self.frontal_vehicle = None
        self.rear_vehicle = None
        self.frontal_vehicles = []
        self.rear_vehicles = []
        self.leader = False
        self.platooning_object = None
        self.platooning_id = None
        self.in_id = None

    def match_platoon(self, cav_nearby):
        """
        A naive way to find the best position to join a platoon

        Parameters
        ----------
        cav_nearby : dict
            The dictionary contains all the cavs nearby.

        Returns
        -------
        matched : bool
            The boolean indicator of matching result.

        min_index : int
            The minimum index inside the selected platoon.

        platoon_vehicle_list : list
            The list of platoon members.
        """

        # make sure the previous status won't influence current one
        self.reset()

        cur_loc = self.ego_pos.location
        cur_yaw = self.ego_pos.rotation.yaw

        pmid, pm = self.search_platoon(cur_loc, cav_nearby)

        if not pmid or pmid in self.platooning_blacklist:
            return False, -1, []

        # used to search the closest platoon member in the searched platoon
        min_distance = float('inf')
        min_index = -1
        min_angle = 0

        # if the platooning is not open to joining
        if not pm.response_joining_request(self.ego_pos.location):
            return False, -1, []

        platoon_vehicle_list = []

        for (i, vehicle_manager) in enumerate(pm.vehicle_manager_list):
            distance, angle = cal_distance_angle(
                vehicle_manager.vehicle.get_location(), cur_loc, cur_yaw)
            platoon_vehicle_list.append(vehicle_manager)

            if distance < min_distance:
                min_distance = distance
                min_index = i
                min_angle = angle

        # if the ego is in front of the platooning
        if min_index == 0 and min_angle > 90:
            self.frontal_vehicle = None
            self.rear_vehicle = pm.vehicle_manager_list[0]
            return True, min_index, platoon_vehicle_list

        self.fronal_vehicle = pm.vehicle_manager_list[min_index]

        if min_index < len(pm.vehicle_manager_list) - 1:
            self.rear_vehicle = pm.vehicle_manager_list[min_index + 1]

        # update front vehicles and rear vehicles basically, everything in front that's in the search range,
        # and everything in the back that's in the search range
        front_index_start = math.max(min_index - self.v2v_count + 1, 0)
        rear_index_end = math.min(
            pm.vehicle_manager_list.length - 1,
            min_index + self.v2v_count
        )

        self.frontal_vehicles = pm.vehicle_manager_list[front_index_start: min_index + 1]
        self.rear_vehicles = pm.vehicle_manager_list[min_index +
                                                     1: rear_index_end + 1]

        return True, min_index, platoon_vehicle_list
