# Drone Delivery Sim

**Drone Delivery Sim** is a Python-based simulator that models drone delivery logistics across agricultural fields using real USDA Crop Data Layer (CDL) data. Drones pick up goods from randomly selected soybean farms and deliver them to randomly located customers. A PyQt5 + PyQtGraph GUI animates drones with six legs.

---

## Features

- **Real USDA CDL Data**: Downloads and parses 2024 GeoTIFF cropland layers.
- **Farm Selection**: Identifies soybean fields (motifiable for different crops) and selects a random 10% for drone operations.
- **Autonomous Drones**: Each drone does:
  - Pickup from an assigned farm
  - Delivery to a customer
- **Real-Time GUI**:
  - Displays farms, customers, and drones
  - Changes color for customers upon delivery

---

## Dependencies

Install these Python packages:

pip install numpy requests rasterio pyqt5 pyqtgraph
