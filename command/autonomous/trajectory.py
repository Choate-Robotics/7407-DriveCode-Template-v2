from pathplannerlib.trajectory import PathPlannerPath

class Choreo_Trajectory:
    def __init__(self, choreo_path: PathPlannerPath):
        self.choreo_path = choreo_path
        self.waypoints = choreo_path.getAllPathPoints()
        self.current_waypoint = 0
        self.current_pose = self.waypoints[self.current_waypoint]
        self.current_pose_index = 0
        self.current_pose_index_max = len(self.waypoints)
        self.is_done = False
