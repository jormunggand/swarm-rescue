import numpy as np

from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.utils.grid import Grid
from spg_overlay.utils.pose import Pose

from spg_overlay.utils.constants import MAX_RANGE_LIDAR_SENSOR


class OccupancyGrid(Grid):
    """Simple occupancy grid copy_pasted from examples/example_mapping"""

    def __init__(self,
                 size_area_world,
                 resolution: float,
                 lidar):
        super().__init__(size_area_world=size_area_world, resolution=resolution)

        self.size_area_world = size_area_world
        self.resolution = resolution

        self.lidar = lidar

        self.x_max_grid: int = int(self.size_area_world[0] / self.resolution + 0.5)
        self.y_max_grid: int = int(self.size_area_world[1] / self.resolution + 0.5)

        self.grid = np.zeros((self.x_max_grid, self.y_max_grid))
        self.zoomed_grid = np.empty((self.x_max_grid, self.y_max_grid))

    def update_grid(self, pose: Pose):
        """
        Bayesian map update with new observation
        lidar : lidar data
        pose : corrected pose in world coordinates
        """
        EVERY_N = 3
        LIDAR_DIST_CLIP = 40.0
        EMPTY_ZONE_VALUE = -0.602
        OBSTACLE_ZONE_VALUE = 2.0
        FREE_ZONE_VALUE = -4.0
        THRESHOLD_MIN = -40
        THRESHOLD_MAX = 40

        lidar_dist = self.lidar.get_sensor_values()[::EVERY_N].copy()
        lidar_angles = self.lidar.ray_angles[::EVERY_N].copy()

        # Compute cos and sin of the absolute angle of the lidar
        cos_rays = np.cos(lidar_angles + pose.orientation)
        sin_rays = np.sin(lidar_angles + pose.orientation)

        max_range = MAX_RANGE_LIDAR_SENSOR * 0.9

        # For empty zones
        # points_x and point_y contains the border of detected empty zone
        # We use a value a little bit less than LIDAR_DIST_CLIP because of the noise in lidar
        lidar_dist_empty = np.maximum(lidar_dist - LIDAR_DIST_CLIP, 0.0)
        # All values of lidar_dist_empty_clip are now <= max_range
        lidar_dist_empty_clip = np.minimum(lidar_dist_empty, max_range)
        points_x = pose.position[0] + np.multiply(lidar_dist_empty_clip, cos_rays)
        points_y = pose.position[1] + np.multiply(lidar_dist_empty_clip, sin_rays)

        for pt_x, pt_y in zip(points_x, points_y):
            self.add_value_along_line(pose.position[0], pose.position[1], pt_x, pt_y, EMPTY_ZONE_VALUE)

        # For obstacle zones, all values of lidar_dist are < max_range
        select_collision = lidar_dist < max_range

        points_x = pose.position[0] + np.multiply(lidar_dist, cos_rays)
        points_y = pose.position[1] + np.multiply(lidar_dist, sin_rays)

        points_x = points_x[select_collision]
        points_y = points_y[select_collision]

        self.add_points(points_x, points_y, OBSTACLE_ZONE_VALUE)

        # the current position of the drone is free !
        self.add_points(pose.position[0], pose.position[1], FREE_ZONE_VALUE)

        # threshold values
        self.grid = np.clip(self.grid, THRESHOLD_MIN, THRESHOLD_MAX)

        # compute zoomed grid for displaying
        self.zoomed_grid = self.grid.copy()
        new_zoomed_size = (int(self.size_area_world[1] * 0.5), int(self.size_area_world[0] * 0.5))
        #self.zoomed_grid = cv2.resize(self.zoomed_grid, new_zoomed_size, interpolation=cv2.INTER_NEAREST)


class MyDroneMapping(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.iter = 0

        self.grid = OccupancyGrid(size_area_world=self.size_area, resolution=2, lidar=self.lidar())

    def define_message_for_all(self):
        pass

    def control(self):
        self.iter += 1

        curPose = Pose(np.asarray(self.measured_gps_position()), self.measured_compass_angle())
        self.grid.update_grid(curPose);

        if self.iter % 5 == 0:
            self.grid.display(self.grid.grid, curPose)

        return {"forward": 0, "lateral": 0, "rotation": 0, "grasper": 0}
