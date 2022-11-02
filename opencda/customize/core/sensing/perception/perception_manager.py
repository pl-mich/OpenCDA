import cv2
from opencda.core.sensing.perception.perception_manager import PerceptionManager
from opencda.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from opencda.core.sensing.perception.static_obstacle import TrafficLight


class CustomizedPerceptionManager(PerceptionManager):
    def __init__(self, vehicle, config_yaml, cav_world, data_dump=False):
        super(CustomizedPerceptionManager, self).__init(
            vehicle, config_yaml, cav_world, data_dump)

    def detect(self, ego_pos):
        objects = {
            'vehicles': [],
            'traffic_lights': [],
            'placeholder': []
        }

        rgb_images = []
        for rgb_camera in self.rgb_camera:
            while rgb_camera.image is None:
                continue
            rgb_images.append(
                cv2.cvtColor(np.array(rgb_camera.image), cv2.COLOR_BGR2RGB))
            lidar_data = self.lidar.data

            return objects
