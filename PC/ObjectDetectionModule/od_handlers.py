from abc import ABC, abstractmethod
from enum import Enum


class ODHandlerType(Enum):
    PEDESTRIAN = 0
    VEHICLE = 1
    TRAFFIC_LIGHT = 2


class ODHandler(ABC):
    @abstractmethod
    def get_speed_multiplier(self, detection_dict: dict) -> float:
        pass


class PedestrianODHandler(ODHandler):
    def get_speed_multiplier(self, detection_dict: dict) -> float:
        nearest = 0
        for pedestrian in detection_dict["pedestrian"]:
            x, y = pedestrian
            nearest = max(nearest, y)
        if nearest > 0.6:
            print("Pedestrian nearby.")
            return 0
        return 1


class VehicleODHandler(ODHandler):
    def get_speed_multiplier(self, detection_dict: dict) -> float:
        nearest = 0
        for vehicle in detection_dict["vehicle"]:
            x, y = vehicle
            nearest = max(nearest, y)

        if nearest > 0.3:
            print("Vehicle nearby.")
            return 0
        return 1


class TrafficLightODHandler(ODHandler):
    def __init__(self):
        self.stopped = False

    def get_speed_multiplier(self, detection_dict: dict) -> float:
        if self.stopped:
            for green in detection_dict["light_green"]:
                x, y = green
                if y > 0.4:
                    print("Started")
                    self.stopped = False
                    break
            else:
                return 0
        else:
            for red in detection_dict["light_red"]:
                x, y = red
                if y > 0.4:
                    print("stopped")
                    self.stopped = True
                    return 0
        return 1
