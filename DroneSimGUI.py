import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QRectF
from PyQt5.QtWidgets import QGraphicsRectItem, QGraphicsLineItem, QGraphicsItemGroup, QGraphicsEllipseItem
from PyQt5.QtGui import QBrush, QPen, QColor, QPolygonF
import pyqtgraph as pg
import math
from DroneSim import DroneSim
from configs import CROP_CODE, CROP_NAME, GRID_SIZE, GRID_RES, NUM_DRONES, NUM_CUSTOMERS, BASE_LOC, COALITION_PERCENT


class DroneSimGUI(QtWidgets.QMainWindow):
    def __init__(self, label_grid, field_counts, sim):
        super().__init__()
        self.setWindowTitle("CDL Drone Delivery")
        self.plot = pg.PlotWidget()
        self.setCentralWidget(self.plot)
        self.plot.setXRange(0, GRID_SIZE)
        self.plot.setYRange(0, GRID_SIZE)
        self.plot.setBackground('w')
        self.plot.getViewBox().setAspectLocked(True)
        self.plot.hideAxis('bottom')
        self.plot.hideAxis('left')
        self.label_grid = label_grid
        self.field_counts = field_counts
        self.sim = sim
        sim.observers.append(self)
        self.drone_colors = [(255, 0, 0), (0, 0, 255), (0, 200, 0), (200, 0, 200), (255, 128, 0)]
        self.drone_shapes = []
        self.customer_boxes = []
        self.base_scatter = None
        self.draw_grid()
        self.update_sim()

    def draw_grid(self):
        self.grid_items = []
        cell_size = GRID_SIZE / GRID_RES
        for i in range(GRID_RES):
            for j in range(GRID_RES):
                if self.label_grid[i, j]:
                    rect = QGraphicsRectItem(j * cell_size, i * cell_size, cell_size, cell_size)
                    rect.setBrush(QBrush(QColor(0, 180, 0)))
                    rect.setPen(QPen(QtCore.Qt.NoPen))
                    rect.setZValue(-10)
                    self.plot.addItem(rect)
                    self.grid_items.append(rect)

    def update_sim(self):
        for item in self.drone_shapes:
            self.plot.getViewBox().scene().removeItem(item)
        self.drone_shapes = []

        for item in self.customer_boxes:
            self.plot.getViewBox().scene().removeItem(item)
        self.customer_boxes = []

        if self.base_scatter:
            self.plot.removeItem(self.base_scatter)

        cell_size = GRID_SIZE / GRID_RES
        for idx, pos in enumerate(self.sim.customer_pos):
            cx = int(pos[0] / cell_size)
            cy = int(pos[1] / cell_size)
            rect = QGraphicsRectItem(cx * cell_size, cy * cell_size, cell_size, cell_size)
            if self.sim.delivered[idx]:
                rect.setBrush(QBrush(QColor(173, 216, 230))) 
            else:
                rect.setBrush(QBrush(QColor(0, 0, 200, 160)))
            rect.setPen(QPen(QtCore.Qt.NoPen))
            rect.setZValue(1)
            self.plot.addItem(rect)
            self.customer_boxes.append(rect)

        self.base_scatter = pg.ScatterPlotItem(x=[BASE_LOC[0]], y=[BASE_LOC[1]], size=13,
                                               brush=pg.mkBrush(255, 200, 0, 220), pen=pg.mkPen('k'))
        self.plot.addItem(self.base_scatter)

        for i, pos in enumerate(self.sim.drone_pos):
            x, y = pos
            color = QColor(*self.drone_colors[i % len(self.drone_colors)])
            group = QGraphicsItemGroup()

            leg_len = 6
            for angle in range(0, 360, 60):
                rad = math.radians(angle)
                x2 = leg_len * math.cos(rad)
                y2 = leg_len * math.sin(rad)
                leg = QGraphicsLineItem(0, 0, x2, y2)
                leg.setPen(QPen(color, 1))
                group.addToGroup(leg)

                # Add 4-blade rotor at leg tip
                for rotor_angle in [0, 90, 180, 270]:
                    rotor_rad = math.radians(rotor_angle)
                    rx = x2 + 2 * math.cos(rotor_rad)
                    ry = y2 + 2 * math.sin(rotor_rad)
                    rotor = QGraphicsLineItem(x2, y2, rx, ry)
                    rotor.setPen(QPen(QtCore.Qt.black, 0.5))
                    group.addToGroup(rotor)

            if self.drone_is_carrying(i):
                box = QGraphicsRectItem(-3, 4, 6, 3)
                box.setBrush(QBrush(QColor(139, 69, 19)))
                box.setPen(QPen(QColor('black')))
                group.addToGroup(box)

            group.setPos(x, y)
            group.setZValue(10)
            self.plot.addItem(group)
            self.drone_shapes.append(group)

    def drone_is_carrying(self, idx):
        if not self.sim.assignments[idx]:
            return False
        cust_idx = self.sim.assignments[idx][0]
        route = self.sim.routes[cust_idx]
        t = self.sim.drone_route_idx[idx]
        return t == 1