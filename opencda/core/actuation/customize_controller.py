import carla.libcarla

class CustomizeController:
    def update_info(self, ego_pos, ego_spd):
        ########################################
        # this is where you put your algorithm #
        ########################################
        do_some_process(ego_pos, ego_spd)
    
    def run_step(self, target_speed, target_loc):
        ########################################
        # this is where you put your algorithm #
        ########################################
        control_command = control(target_speed, target_loc)
        assert control_command == carla.libcarla.VehicleControl
        return control_command
        