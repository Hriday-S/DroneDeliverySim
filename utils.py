from configs import GRID_SIZE, GRID_RES
import numpy as np
def random_grid_pos(n):
    return np.random.uniform(0, GRID_SIZE, size=(n, 2))
def random_customer_pos(num_customers, label_grid):
    allowed = np.argwhere(~label_grid)
    if allowed.shape[0] < num_customers:
        raise ValueError("Not enough non-farm grid cells for customers.")
    chosen = allowed[np.random.choice(len(allowed), num_customers, replace=False)]
    cell_size = GRID_SIZE / GRID_RES
    return np.array([[(j + 0.5) * cell_size, (i + 0.5) * cell_size] for i, j in chosen])
