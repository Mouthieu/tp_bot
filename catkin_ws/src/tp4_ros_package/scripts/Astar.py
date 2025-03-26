#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import heapq
import numpy as np
import matplotlib.pyplot as plt
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
        
        # self.start = rospy.get_param("starting_point", (210, 280))
        self.start = (430, 290)
        self.goal = (425, 291)
        # self.goal = rospy.get_param("end_point", (450, 255))
        # self.goal = rospy.get_param("end_point", (280, 280))

        self.use_eight_connected = rospy.get_param("use_eight_connected", False)  # Paramètre ROS pour choisir 4 ou 8 connexions

        rospy.spin()
    
    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        # plt.imshow(self.map_data)
        # plt.scatter(self.start[1], self.start[0], c="green", marker="o", label="Start")  # Start en vert
        # plt.scatter(self.goal[1], self.goal[0], c="red", marker="x", label="Goal")  # Goal en rouge
        # plt.show()

        if self.map_data[self.start[0]][self.start[1]] != 0:
            print("pb")
        if self.map_data[self.goal[0]][self.goal[1]] != 0:
            print("pb")

        # self.map_data[self.start[1]][self.start[0]] = 50
        # self.map_data[self.goal[1]][self.goal[0]] = 50

        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        # rospy.loginfo(self.map_data)
        # rospy.loginfo(f"Carte reçue : {self.width}x{self.height}")
        # rospy.loginfo(f"Résolution : {self.resolution} m/pixel")
        # rospy.loginfo(f"Origine : {self.origin}")
        # rospy.loginfo(f"Start: {self.start}, Goal: {self.goal}")

        if self.start and self.goal:
            path = self.a_star(self.start, self.goal)
            self.publish_path(path)
            self.map = True

    
    def heuristic(self, a, b, mode="8-connected"):
        x1, y1 = a
        x2, y2 = b
        if mode == "4-connected":
            # Distance de Manhattan
            return abs(x1 - x2) + abs(y1 - y2)
        elif mode == "8-connected":
            # Distance de Chebyshev (diagonale permise)
            return max(abs(x1 - x2), abs(y1 - y2))
        else:
            # Distance Euclidienne pour la meilleure estimation
            return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    
    def get_neighbors(self, node, mode):
        if mode == "8-connected":
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
    
    def draw_exploration(self, ax, explored):
        for x, y in explored:
            ax.plot(y, x, marker="s", color="blue", markersize=2)  # Dessiner en bleu les cases explorées
            ax.scatter(self.start[1], self.start[0], c="green", marker="o", label="Start")  # Start en vert
            ax.scatter(self.goal[1], self.goal[0], c="red", marker="x", label="Goal")  # Goal en rouge
        plt.pause(0.01)  # Pause pour voir l'évolution en temps réel

    def draw_path(self, ax, path):
        for x, y in path:
            ax.plot(y, x, marker="s", color="red", markersize=4)  # Dessiner le chemin en rouge
        plt.show()  # Afficher l’image finale

    # def a_star_2(self, start, goal):
    #     fig, ax = plt.subplots()
    #     ax.imshow(self.map_data, cmap="gray")  # Afficher la carte initiale
    #     open_list = []
    #     heapq.heappush(open_list, (0, start))
    #     came_from = {}
    #     g_score = {start: 0}
    #     f_score = {start: self.heuristic(start, goal)}
    #     explored = set()  # Pour stocker les cases explorées
    #     explored_steps = []  # Liste pour stocker les étapes de l'exploration
    #     while open_list:
    #         _, current = heapq.heappop(open_list)
    #         rospy.loginfo(f"Open list size: {len(open_list)}")
    #         if current == goal:
    #             path = self.reconstruct_path(came_from, current)
    #             self.draw_path(ax, path)  # Afficher le chemin final
    #             return path
    #         explored.add(current)

    #         for neighbor in self.get_neighbors(current):
    #             tentative_g_score = g_score[current] + np.sqrt((current[0] - neighbor[0])**2 + (current[1] - neighbor[1])**2)
    #             if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
    #                 came_from[neighbor] = current
    #                 g_score[neighbor] = tentative_g_score
    #                 f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
    #                 heapq.heappush(open_list, (f_score[neighbor], neighbor))
    #         self.draw_exploration(ax, explored)  # Afficher les cases explorées
    #     self.show_path_on_map(explored_steps, [])  # Afficher uniquement l'exploration si pas de chemin trouvé
    #     return []

    # def a_star(self, start, goal):
    #     fig, ax = plt.subplots()
    #     ax.imshow(self.map_data, cmap="gray")  # Afficher la carte initiale

    #     open_list = []
    #     heapq.heappush(open_list, (0, start))  # La liste ouverte avec l'élément de départ
    #     came_from = {}  # Dictionnaire pour reconstruire le chemin
    #     g_score = {start: 0}  # Coût pour atteindre chaque point
    #     f_score = {start: self.heuristic(start, goal)}  # Coût total (g + h) pour chaque point
    #     explored = set()  # Pour stocker les cases explorées

    #     while open_list:
    #         _, current = heapq.heappop(open_list)  # Extraire le nœud avec le coût f le plus bas

    #         # Si on atteint l'objectif, on reconstruit et retourne le chemin
    #         if current == goal:
    #             path = self.reconstruct_path(came_from, current)
    #             self.draw_path(ax, path)  # Afficher le chemin final
    #             return path

    #         explored.add(current)  # Marquer la case comme explorée

    #         # Explorer les voisins
    #         for neighbor in self.get_neighbors(current):
    #             if neighbor in explored:  # Si ce voisin a déjà été exploré, on l'ignore
    #                 continue

    #             tentative_g_score = g_score[current] + 1  # Le coût pour atteindre ce voisin (c'est toujours 1)

    #             # Si ce voisin n'a pas encore été exploré ou si on trouve un chemin moins coûteux
    #             if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
    #                 came_from[neighbor] = current
    #                 g_score[neighbor] = tentative_g_score
    #                 f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)  # Ajouter l'heuristique
    #                 heapq.heappush(open_list, (f_score[neighbor], neighbor))

    #         # Afficher les cases explorées sur la carte à chaque étape
    #         self.draw_exploration(ax, explored)

    #     # Si aucun chemin n'est trouvé, retourne une liste vide
    #     return []

    def a_star(self, start, goal, mode="8-connected"):
        fig, ax = plt.subplots()
        ax.imshow(self.map_data, cmap="gray")  # Afficher la carte initiale

        open_list = []
        heapq.heappush(open_list, (0, start))  # La liste ouverte avec l'élément de départ
        came_from = {}  # Dictionnaire pour reconstruire le chemin
        g_score = {start: 0}  # Coût pour atteindre chaque point
        f_score = {start: self.heuristic(start, goal)}  # Coût total (g + h) pour chaque point
        explored = set()  # Pour stocker les cases explorées

        while open_list:
            _, current = heapq.heappop(open_list)  # Extraire le nœud avec le coût f le plus bas

            # Si on atteint l'objectif, on reconstruit et retourne le chemin
            if current == goal:
                path = self.reconstruct_path(came_from, current)
                self.draw_path(ax, path)  # Afficher le chemin final
                return path

            explored.add(current)  # Marquer la case comme explorée

            # Explorer les voisins
            for neighbor in self.get_neighbors(current, mode=mode):
                if neighbor in explored:  # Si ce voisin a déjà été exploré, on l'ignore
                    continue

                tentative_g_score = g_score[current] + 1  # Le coût pour atteindre ce voisin (c'est toujours 1)

                # Si ce voisin n'a pas encore été exploré ou si on trouve un chemin moins coûteux
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal, mode=mode)  # Ajouter l'heuristique

                    # Mettre à jour la liste ouverte uniquement si le voisin n'est pas déjà présent ou si le coût est plus bas
                    existing = [item for item in open_list if item[1] == neighbor]
                    if existing:
                        open_list.remove(existing[0])
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

            # Afficher les cases explorées sur la carte à chaque étape
            self.draw_exploration(ax, explored)

        # Si aucun chemin n'est trouvé, retourne une liste vide
        return []
    
    # def a_star(self, start, goal):
    #     open_list = []
    #     heapq.heappush(open_list, (0, start))
    #     came_from = {}
    #     g_score = {start: 0}
    #     f_score = {start: self.heuristic(start, goal)}
    #     explored_steps = []  # Liste pour stocker les étapes de l'exploration
    #     while open_list:
    #         _, current = heapq.heappop(open_list)
    #         if current == goal:
    #             path = self.reconstruct_path(came_from, current)
    #             self.show_path_on_map(explored_steps, path)  # Afficher l'exploration et le chemin final
    #             return path
    #         if current not in explored_steps:
    #             explored_steps.append(current)
    #         for neighbor in self.get_neighbors(current):
    #             tentative_g_score = g_score[current] + np.sqrt((current[0] - neighbor[0])**2 + (current[1] - neighbor[1])**2)
    #             if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
    #                 came_from[neighbor] = current
    #                 g_score[neighbor] = tentative_g_score
    #                 f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
    #                 heapq.heappush(open_list, (f_score[neighbor], neighbor))
    #     self.show_path_on_map(explored_steps, [])  # Afficher uniquement l'exploration si pas de chemin trouvé
    #     return []


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
