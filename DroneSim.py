import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QRectF
from PyQt5.QtWidgets import QGraphicsRectItem, QGraphicsLineItem, QGraphicsItemGroup
from PyQt5.QtGui import QBrush, QPen, QColor
import pyqtgraph as pg
import math
from configs import CROP_CODE, CROP_NAME, GRID_SIZE, GRID_RES, NUM_DRONES, NUM_CUSTOMERS, BASE_LOC, COALITION_PERCENT, DRONE_SPEED
from utils import random_grid_pos, random_customer_pos


class DroneSim(QtCore.QObject):
    def __init__(self, num_drones, num_customers, field_coords, label_grid):
        super().__init__()
        self.drone_pos = np.array([BASE_LOC.copy() for i in range(num_drones)])
        self.label_grid = label_grid
        self.customer_pos = random_customer_pos(num_customers, label_grid)
        self.num_drones = num_drones
        self.num_customers = num_customers
        self.field_coords = np.array(field_coords)
        self.pickup_points = np.array([
            self.field_coords[np.argmin(np.linalg.norm(self.field_coords - c, axis=1))]
            for c in self.customer_pos
        ])
        self.routes = [
            [self.pickup_points[i], self.customer_pos[i]]
            for i in range(self.num_customers)
        ]
        self.assignments = self.assign_customers()
        self.drone_targets = [None] * self.num_drones
        self.drone_route_idx = [0] * self.num_drones
        self.delivered = [False] * self.num_customers
        self.observers = []  
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(lambda: self.step(DRONE_SPEED))


    def assign_customers(self):
        unassigned = set(range(self.num_customers))
        assign = [[] for _ in range(self.num_drones)]
        drone_locs = [BASE_LOC.copy() for _ in range(self.num_drones)]

        while unassigned:
            for d in range(self.num_drones):
                if not unassigned:
                    break

                def total_path_len(c):
                    farm = self.pickup_points[c]
                    return np.linalg.norm(drone_locs[d] - farm) + np.linalg.norm(farm - self.customer_pos[c])

                cust_idx = min(unassigned, key=total_path_len)
                assign[d].append(cust_idx)
                drone_locs[d] = self.customer_pos[cust_idx].copy()
                unassigned.remove(cust_idx)

        return assign
    def step(self, speed):
        for i in range(self.num_drones):
            if not self.assignments[i]:
                continue
            cust_idx = self.assignments[i][0]
            route = self.routes[cust_idx]
            target_idx = self.drone_route_idx[i]
            target = route[target_idx]
            direction = target - self.drone_pos[i]
            dist = np.linalg.norm(direction)
            if dist < speed:
                self.drone_pos[i] = target.copy()
                self.drone_route_idx[i] += 1
                if self.drone_route_idx[i] == len(route):
                    self.assignments[i].pop(0)
                    self.drone_route_idx[i] = 0
                    self.delivered[cust_idx] = True
            else:
                self.drone_pos[i] += direction / dist * speed
        for obs in self.observers:
            obs.update_sim()
   