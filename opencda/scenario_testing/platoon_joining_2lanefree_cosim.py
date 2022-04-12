# -*- coding: utf-8 -*-
# no shebang because it is not intended to run on its own
"""
Scenario testing: merging vehicle joining a platoon in the
customized 2-lane freeway simplified map sorely with co-sim
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

# import directives
import os
import carla
import opencda.scenario_testing.utils.cosim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import load_yaml


def run_scenario(opt, config_yaml):
    try:

        # 1. Add yaml file into a dictionary
        scenario_params = load_yaml(config_yaml)

        # 2. Create CAV world object to store all CAV VehicleManager info.
        # this is the key element to achieve cooperation
        # Note how this is orthogonal to creating a "World" object in Carla
        cav_world = CavWorld(opt.apply_ml)

        # 3. sumo conifg file path
        current_path = os.path.dirname(os.path.realpath(__file__))
        xodr_path = os.path.join(current_path,
                                 '../assets/2lane_freeway_simplified/'
                                 '2lane_freeway_simplified.xodr')
        sumo_cfg = os.path.join(current_path,
                                '../assets/2lane_freeway_simplified')

        # 4. create co-simulation scenario manager
        scenario_manager = sim_api.CoScenarioManager(
            scenario_params, # from YAML file in step 1
            opt.apply_ml, # from function argument
            opt.version, # from function argument
            cav_world=cav_world, # from step 2
            xodr_path=xodr_path, # from step 3
            sumo_file_parent_path=sumo_cfg # from step 3
        ) 

        # 5. create platoon members
        platoon_list = scenario_manager.create_platoon_manager(
            map_helper=map_api.spawn_helper_2lanefree, # map definition
            data_dump=False
        )
        single_cav_list = scenario_manager.create_vehicle_manager(
            application=['platooning'],
            map_helper=map_api.spawn_helper_2lanefree # map definition
        )

        # 6. create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='single_2lanefree_cosim',
                              current_time=scenario_params['current_time'])

        # 7. Assign spectator to second vehicle in platoon?
        spectator = scenario_manager.world.get_spectator()
        spectator_vehicle = platoon_list[0].vehicle_manager_list[1].vehicle

        # 8. Run Simulation
        while True:
            # 8.1 simulation tick
            scenario_manager.tick()

            # 8.2 Have spectator follow the assigned vehicle's Transform attribute
            transform = spectator_vehicle.get_transform()
            spectator.set_transform(
                carla.Transform(transform.location + carla.Location(z=80),
                                carla.Rotation(pitch=-90)))

            # 8.3 Run simulation step on platoons
            for platoon in platoon_list:
                platoon.update_information()
                platoon.run_step()

            # 8.4 Run simulation step in orphaned CAVs
            for i, single_cav in enumerate(single_cav_list):
                # Remove the orphaned cav from this process if it's already in a platoon
                if single_cav.v2x_manager.in_platoon():
                    single_cav_list.pop(i)
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)

    finally:
        # 9. When the simulation is over, the EvaluationManager will evaluate the performance, and save the results in ~/OpenCDA/evluation_outputs
        eval_manager.evaluate()
    
        # 10. Destroy objects
        scenario_manager.close()
        for platoon in platoon_list:
            platoon.destroy()
        for v in single_cav_list:
            v.destroy()
