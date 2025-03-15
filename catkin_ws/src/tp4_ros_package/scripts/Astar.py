import rospy
import heapq
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

class AStarPlanner:
    def __init__(self):
        rospy.init_node("a_star_planner")
        
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)
        
        self.map_data = None
        self.width = 0
        self.height = 0
        self.resolution = 0
        self.origin = None
        
        self.start = (10, 10)  # À modifier selon les besoins
        self.goal = (40, 40)   # À modifier selon les besoins
        self.use_eight_connected = rospy.get_param("~use_eight_connected", False)  # Paramètre ROS pour choisir 4 ou 8 connexions

        rospy.spin()
    
    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        if self.start and self.goal:
            path = self.a_star(self.start, self.goal)
            self.publish_path(path)
    
    def heuristic(self, a, b):
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)  # Distance Euclidienne pour meilleure estimation
    
    def get_neighbors(self, node):
        if self.use_eight_connected:
            neighbors = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,1), (1,-1), (-1,-1)]  # 8-connecté
        else:
            neighbors = [(0,1), (1,0), (0,-1), (-1,0)]  # 4-connecté
        
        result = []
        for dx, dy in neighbors:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < self.height and 0 <= ny < self.width:
                if self.map_data[nx, ny] == 0:  # Pas d'obstacle
                    result.append((nx, ny))
        return result
    
    def a_star(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_list:
            _, current = heapq.heappop(open_list)
            
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + np.sqrt((current[0] - neighbor[0])**2 + (current[1] - neighbor[1])**2)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
        return []
    
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
    
    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for x, y in path:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = x * self.resolution + self.origin[0]
            pose.pose.position.y = y * self.resolution + self.origin[1]
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

if __name__ == "__main__":
    try:
        AStarPlanner()
    except rospy.ROSInterruptException:
        pass
