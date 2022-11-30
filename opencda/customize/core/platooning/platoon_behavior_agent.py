from collections import deque

import carla
import numpy as np
import math

from opencda.core.application.platooning.platoon_behavior_agent \
    import PlatooningBehaviorAgent
from opencda.core.common.misc import compute_distance


class CustomizedPlatooningBehaviorAgent(PlatooningBehaviorAgent):
    def __init__(
            self,
            vehicle,
            vehicle_manager,
            v2x_manager,
            behavior_yaml,
            platoon_yaml,
            carla_map):

        super(
            CustomizedPlatooningBehaviorAgent,
            self).__init__(
            vehicle,
            vehicle_manager,
            v2x_manager,
            behavior_yaml,
            platoon_yaml,
            carla_map)

    # def idm_platooning_following_manager(self, inter_gap):
    #     """
    #     Car following behavior in platooning with gap regulation.
    #
    #     Parameters
    #     __________
    #     inter_gap : float
    #         The intervehicular time gap designed for platooning.
    #     """
    #
    #     frontal_vehicle_managers, _ = self.v2x_manager.get_platoon_front_rear_mult()
    #
    #     # vehicle in front
    #     frontal_vehicle_manager = frontal_vehicle_managers[0] \
    #         if len(frontal_vehicle_managers) >= 1 else None
    #     # second vehicle in front
    #     frontal_front_vehicle_manager = frontal_vehicle_managers[1] \
    #         if len(frontal_vehicle_managers) >= 2 else None
    #
    #     # we are not updating trajectories since update time step has not been reached
    #     if len(self._local_planner.get_trajectory()
    #            ) > self.get_local_planner().trajectory_update_freq - 2:
    #         return self._local_planner.run_step([], [], [], following=True)
    #
    #     # get frontal trajectories and speeds
    #     frontal_trajectories, frontal_speeds, tracked_lengths = [], [], []
    #
    #     for mgr in frontal_vehicle_managers:
    #         if mgr:
    #             frontal_trajectories.append(mgr.agent.get_local_planner().get_trajectory())
    #             frontal_speeds.append(mgr.agent._ego_speed)
    #
    #     for i, traj in enumerate(frontal_trajectories):
    #         tracked_len = len(traj) - 1 \
    #             if not frontal_vehicle_managers[i + 1] else len(traj)
    #         tracked_lengths.append(tracked_len)
    #
    #     # tracked_length = len(frontal_trajectory) - 1 \
    #     #     if not frontal_front_vehicle_manager \
    #     #     else len(frontal_trajectory)
    #     tracked_length = tracked_lengths[0]
    #
    #     # this agent is a behavior agent
    #     # frontal_trajectory = frontal_vehicle_manager. \
    #     # agent.get_local_planner().get_trajectory()
    #     frontal_trajectory = frontal_trajectories[0]
    #
    #     # get front speed
    #     # frontal_speed = frontal_vehicle_manager.agent._ego_speed
    #     frontal_speed = frontal_speeds[0]
    #
    #     ego_trajectory = deque(maxlen=30)
    #     ego_loc_x, ego_loc_y, ego_loc_z = \
    #         self._ego_pos.location.x, \
    #         self._ego_pos.location.y, \
    #         self._ego_pos.location.z
    #
    #     # get ego speed
    #     ego_speed = self._ego_speed
    #
    #     # compare speed with frontal veh
    #     # frontal_speed_diff = ego_speed - frontal_speed
    #     frontal_speed_diffs = [ego_speed - f_spd for f_spd in frontal_speeds]
    #     # frontal_speed_diff = frontal_speed_diffs[0]
    #     sum = 0
    #     sum_weight = 0
    #     # possibly the shittest weighted sum formula out there
    #     for i, diff in enumerate(frontal_speed_diffs):
    #         weight = len(frontal_speed_diffs) - i
    #         sum_weight += weight
    #         sum += diff * weight
    #     frontal_speed_diff = sum / sum_weight
    #
    #     for i in range(tracked_length):
    #         delta_t = self.get_local_planner().dt
    #
    #         # if leader is slowing down(leader target speed is smaller than
    #         # current speed), use a bigger dt.
    #         # spd diff max at 15. If diff greater than 8, increase dt
    #         if frontal_speed_diff > 3.0:
    #             '''
    #             # only increase dt when V_ego > V_front (avoid collision)
    #             # if V_ego < V_front (diff < 0), stick with small dt
    #             # todo: change delta_t to a function:
    #             #      --> 1. {V_ego > V_front}: decrease dt to increase
    #                           gap, help avoid collision
    #             #      --> 2. more difference, more dt adjustment
    #             #      --> 3. {V_ego < V_front}: will not collide,
    #                           keep default dt to keep gap
    #             #      --> 4. {V_ego ~ V_front}: keep default
    #                            dt to keep gap
    #             '''
    #             delta_t = delta_t + frontal_speed_diff * 0.0125
    #
    #         if i == 0:
    #             pos_x = (frontal_trajectory[i][0].location.x + inter_gap / delta_t * ego_loc_x) / \
    #                     (1 + inter_gap / delta_t)
    #             pos_y = (frontal_trajectory[i][0].location.y + inter_gap / delta_t * ego_loc_y) / \
    #                     (1 + inter_gap / delta_t)
    #         else:
    #             # weighted increment of frontal vehicle's location? how is this supposed to be made CIDM
    #             # and I can't do anything without concrete time steps!
    #             # or maybe all this does is indicate the direction, and I should be more worried about
    #             # the velocities!
    #             pos_x = (frontal_trajectory[i][0].location.x +
    #                      inter_gap / delta_t * ego_trajectory[i - 1][0].location.x) / \
    #                     (1 + inter_gap / delta_t)
    #             pos_y = (frontal_trajectory[i][0].location.y +
    #                      inter_gap / delta_t * ego_trajectory[i - 1][0].location.y) / \
    #                     (1 + inter_gap / delta_t)
    #
    #         distance = np.sqrt((pos_x - ego_loc_x) ** 2 + (pos_y - ego_loc_y) ** 2)
    #
    #         # UPDATING VELOCITY!
    #         # That
    #         velocity = distance / delta_t * 3.6
    #
    #         # maintain z coord and update x and y coordinates
    #         ego_trajectory.append([carla.Transform(carla.Location(pos_x, pos_y, ego_loc_z)), velocity])
    #
    #         # update ego x and y coordinates
    #         ego_loc_x, ego_loc_y = (pos_x, pos_y)
    #
    #     if not ego_trajectory:
    #         wpt = self._map.get_waypoint(self._ego_pos.location)
    #         next_wpt = wpt.next(max(2, int(self._ego_speed / 3.6 * 1)))[0]
    #         ego_trajectory.append((next_wpt.transform, self._ego_speed))
    #
    #     return self._local_planner.run_step([], [], [], trajectory=ego_trajectory)

    def cooperative_platooning_following_manager(self, inter_gap):
        """
        Extend the Gazis-Herman-Rothery car-following model (which I assumed is being used here)


        Parameters
        __________
        inter_gap : float
            The intervehicular time gap designed for platooning.
            MAY NOT BE RELEVANT IN CIDM CONTROL LOGIC
        """

        # vehicle in front
        frontal_vehicle_manager, _ = self.v2x_manager.get_platoon_front_rear()
        # second vehicle in front
        frontal_front_vehicle_manager, _ = \
            frontal_vehicle_manager.v2x_manager.get_platoon_front_rear()

        if len(self._local_planner.get_trajectory()
               ) > self.get_local_planner().trajectory_update_freq - 2:
            return self._local_planner.run_step([], [], [], following=True)

        # this agent is a behavior agent
        frontal_trajectory = frontal_vehicle_manager.agent.get_local_planner().get_trajectory()
        # get front speed
        frontal_speed = frontal_vehicle_manager.agent._ego_speed
        ego_trajectory = deque(maxlen=30)
        ego_loc_x, ego_loc_y, ego_loc_z = \
            self._ego_pos.location.x, \
            self._ego_pos.location.y, \
            self._ego_pos.location.z

        # get ego speed
        ego_speed = self._ego_speed

        # compare speed with frontal veh
        frontal_speed_diff = ego_speed - frontal_speed

        tracked_length = len(frontal_trajectory) - 1 \
            if not frontal_front_vehicle_manager \
            else len(frontal_trajectory)

        # todo: current not working well on curve
        frontal_x, frontal_y = None, None
        prev_ego_x, prev_ego_y = None, None
        c = 0.5
        prev_vel = None
        for i in range(tracked_length):
            delta_t = self.get_local_planner().dt

            # if leader is slowing down(leader target speed is smaller than
            # current speed), use a bigger dt.
            # spd diff max at 15. If diff greater than 8, increase dt
            # if frontal_speed_diff > 3.0:
            #     '''
            #     # only increase dt when V_ego > V_front (avoid collision)
            #     # if V_ego < V_front (diff < 0), stick with small dt
            #     # todo: change delta_t to a function:
            #     #      --> 1. {V_ego > V_front}: decrease dt to increase
            #                   gap, help avoid collision
            #     #      --> 2. more difference, more dt adjustment
            #     #      --> 3. {V_ego < V_front}: will not collide,
            #                   keep default dt to keep gap
            #     #      --> 4. {V_ego ~ V_front}: keep default
            #                    dt to keep gap
            #     '''
            #     delta_t = delta_t + frontal_speed_diff * 0.0125

            frontal_x = frontal_trajectory[i][0].location.x
            frontal_y = frontal_trajectory[i][0].location.y

            if i == 0:
                pos_x = (frontal_x + inter_gap / delta_t * ego_loc_x) / \
                        (1 + inter_gap / delta_t)
                pos_y = (frontal_y + inter_gap / delta_t * ego_loc_y) / \
                        (1 + inter_gap / delta_t)
                distance = math.sqrt((pos_x - ego_loc_x) ** 2 + (pos_y - ego_loc_y) ** 2)
                velocity = distance / delta_t * 3.6
            else:
                x_diff = abs(ego_loc_x - frontal_x)
                y_diff = abs(ego_loc_y - frontal_y)
                # okay I admit this is probably not the best call
                # BUT IT FUCKING WORKS!
                orientation = math.atan2(y_diff, x_diff)
                spacing = math.sqrt(x_diff ** 2 + y_diff ** 2)
                accel = 0.5 * frontal_speed_diff / spacing
                velocity = prev_vel + accel * delta_t
                distance = velocity * delta_t
                pos_x = ego_loc_x + distance * math.cos(orientation)
                pos_y = ego_loc_y + distance * math.sin(orientation)
                # prev_ego_x = ego_trajectory[i - 1][0].location.x
                # prev_ego_y = ego_trajectory[i - 1][0].location.y
                # pos_x = (frontal_x + inter_gap / delta_t * prev_ego_x) / \
                #         (1 + inter_gap / delta_t)
                # pos_y = (frontal_y + inter_gap / delta_t * prev_ego_y) / \
                #         (1 + inter_gap / delta_t)

            # distance = np.sqrt((pos_x - ego_loc_x) ** 2 + (pos_y - ego_loc_y) ** 2)
            # velocity = distance / delta_t * 3.6

            # we should have the pos_x, pos_y, and velocity for this time step
            # maintain z coord and update x and y coordinates
            ego_trajectory.append([carla.Transform(carla.Location(pos_x, pos_y, ego_loc_z)), velocity])

            # update ego x and y coordinates
            prev_vel = velocity
            ego_loc_x = pos_x
            ego_loc_y = pos_y

        if not ego_trajectory:
            wpt = self._map.get_waypoint(self._ego_pos.location)
            next_wpt = wpt.next(max(2, int(self._ego_speed / 3.6 * 1)))[0]
            ego_trajectory.append((next_wpt.transform, self._ego_speed))

        return self._local_planner.run_step(
            [], [], [], trajectory=ego_trajectory)

    def idm_platooning_following_manager(self, inter_gap):
            """
            Car following behavior in platooning with gap regulation.

            Parameters
            __________
            inter_gap : float
                The intervehicular time gap designed for platooning.
                MAY NOT BE RELEVANT IN CIDM CONTROL LOGIC
            """

            # vehicle in front
            frontal_vehicle_manager, _ = self.v2x_manager.get_platoon_front_rear()
            # second vehicle in front
            frontal_front_vehicle_manager, _ = \
                frontal_vehicle_manager.v2x_manager.get_platoon_front_rear()

            if len(self._local_planner.get_trajectory()
                   ) > self.get_local_planner().trajectory_update_freq - 2:
                return self._local_planner.run_step([], [], [], following=True)
            else:
                # this agent is a behavior agent
                frontal_trajectory = frontal_vehicle_manager. \
                    agent.get_local_planner().get_trajectory()

                # get front speed
                frontal_speed = frontal_vehicle_manager.agent._ego_speed

                ego_trajectory = deque(maxlen=30)
                ego_loc_x, ego_loc_y, ego_loc_z = \
                    self._ego_pos.location.x, \
                    self._ego_pos.location.y, \
                    self._ego_pos.location.z

                # get ego speed
                ego_speed = self._ego_speed

                # compare speed with frontal veh
                frontal_speed_diff = ego_speed - frontal_speed

                tracked_length = len(frontal_trajectory) - 1 \
                    if not frontal_front_vehicle_manager \
                    else len(frontal_trajectory)

                # todo: current not working well on curve
                for i in range(tracked_length):
                    delta_t = self.get_local_planner().dt

                    # if leader is slowing down(leader target speed is smaller than
                    # current speed), use a bigger dt.
                    # spd diff max at 15. If diff greater than 8, increase dt
                    if frontal_speed_diff > 3.0:
                        '''
                        # only increase dt when V_ego > V_front (avoid collision)
                        # if V_ego < V_front (diff < 0), stick with small dt
                        # todo: change delta_t to a function:
                        #      --> 1. {V_ego > V_front}: decrease dt to increase
                                      gap, help avoid collision
                        #      --> 2. more difference, more dt adjustment
                        #      --> 3. {V_ego < V_front}: will not collide,
                                      keep default dt to keep gap
                        #      --> 4. {V_ego ~ V_front}: keep default
                                       dt to keep gap
                        '''
                        delta_t = delta_t + frontal_speed_diff * 0.0125

                    if i == 0:
                        pos_x = (frontal_trajectory[i][0].location.x + inter_gap / delta_t * ego_loc_x) / \
                                (1 + inter_gap / delta_t)
                        pos_y = (frontal_trajectory[i][0].location.y + inter_gap / delta_t * ego_loc_y) / \
                                (1 + inter_gap / delta_t)
                    else:
                        pos_x = (frontal_trajectory[i][0].location.x +
                                 inter_gap / delta_t * ego_trajectory[i - 1][0].location.x) / \
                                (1 + inter_gap / delta_t)
                        pos_y = (frontal_trajectory[i][0].location.y +
                                 inter_gap / delta_t * ego_trajectory[i - 1][0].location.y) / \
                                (1 + inter_gap / delta_t)

                    distance = np.sqrt((pos_x - ego_loc_x) ** 2 + (pos_y - ego_loc_y) ** 2)
                    # where did 3.6 come from
                    velocity = distance / delta_t * 3.6

                    # maintain z coord and update x and y coordinates
                    ego_trajectory.append([carla.Transform(carla.Location(pos_x, pos_y, ego_loc_z)), velocity])

                    # update ego x and y coordinates
                    ego_loc_x = pos_x
                    ego_loc_y = pos_y

                if not ego_trajectory:
                    wpt = self._map.get_waypoint(self._ego_pos.location)
                    next_wpt = wpt.next(max(2, int(self._ego_speed / 3.6 * 1)))[0]
                    ego_trajectory.append((next_wpt.transform,
                                           self._ego_speed))

                return self._local_planner.run_step(
                    [], [], [], trajectory=ego_trajectory)

    def run_step_maintaining(self):
        """
        Next step behavior planning for speed maintaining.

        Returns
        -------
        target_speed : float
            The target speed for ego vehicle.

        target_waypoint : carla.waypoint
            The target waypoint for ego vehicle.
        """
        # frontal_vehicle_manager, _ = self.v2x_manager.get_platoon_front_rear()
        frontal_vehicle_managers, _ = self.v2x_manager.get_platoon_front_rear_mult()
        # frontal_vehicle_manager = frontal_vehicle_managers[0]
        self.current_gap = self.inter_gap

        frontal_vehicles = []
        frontal_vehicle_locs = []
        for mgr in frontal_vehicle_managers:
            if mgr:
                frontal_vehicles.append(mgr.vehicle)
                frontal_vehicle_locs.append(mgr.v2x_manager.get_ego_pos().location)

        # frontal_vehicle = frontal_vehicle_manager.vehicle
        frontal_vehicle = frontal_vehicles[0]
        frontal_vehicle_loc = frontal_vehicle_locs[0]
        # frontal_vehicle_loc = \
        #     frontal_vehicle_manager.v2x_manager.get_ego_pos().location
        ego_vehicle_loc = self._ego_pos.location

        # headway distance
        distance = compute_distance(ego_vehicle_loc, frontal_vehicle_loc)
        # we always use the true position to calculate the timegap for
        # evaluation
        self.calculate_gap(
            compute_distance(ego_vehicle_loc, frontal_vehicle.get_location()))

        # Distance is computed from the center of the two cars,
        # use bounding boxes to calculate the actual distance
        distance = distance - max(
            frontal_vehicle.bounding_box.extent.y,
            frontal_vehicle.bounding_box.extent.x) - max(
            self.vehicle.bounding_box.extent.y,
            self.vehicle.bounding_box.extent.x)

        # safe control for car following todo: make the coefficient
        # controllable
        if distance <= self._ego_speed / 3.6 * 0.01:
            print("emergency stop!")
            return 0, None

        # target_speed, target_waypoint = self.platooning_following_manager(self.inter_gap)
        target_speed, target_waypoint = self.cooperative_platooning_following_manager(self.inter_gap)

        return target_speed, target_waypoint
