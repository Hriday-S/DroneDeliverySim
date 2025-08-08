# DroneSim.py
import numpy as np

class DroneSim:
    def __init__(self, n, customers, farms, base_loc, speed, eps=1e-6):
        self.eps = float(eps)
        self.spd = float(speed)

        self.farms = farms.astype(bool)
        self.farm_cells = np.argwhere(self.farms)  # precompute for nearest lookup
        if self.farm_cells.size == 0:
            raise ValueError("No farms available.")

        self.cust = [tuple(map(int, c)) for c in customers]
        self.base = tuple(map(int, base_loc))
        self.dpos = [self.base] * int(n)
        self.done = [False] * len(self.cust)

        # nearest farm for each customer
        self.pick = [self._nearest_farm(c) for c in self.cust]

        # two-leg route: farm -> customer
        self.routes = [[self.pick[i], self.cust[i]] for i in range(len(self.cust))]

        # simple greedy allocation
        self.assign = self._alloc()
        self.idx = [0] * int(n)
        self.fin = False

    @staticmethod
    def _dist(a, b):
        return float(np.hypot(a[0]-b[0], a[1]-b[1]))

    def _nearest_farm(self, c):
        di = self.farm_cells[:, 0] - c[0]
        dj = self.farm_cells[:, 1] - c[1]
        k = int(np.argmin(np.hypot(di, dj)))
        return tuple(map(int, self.farm_cells[k]))

    def _alloc(self):
        A = [[] for _ in self.dpos]
        left = set(range(len(self.cust)))
        loc = [self.base] * len(self.dpos)
        while left:
            for d in range(len(self.dpos)):
                if not left: break
                best = min(left, key=lambda i: self._dist(loc[d], self.pick[i]) + self._dist(self.pick[i], self.cust[i]))
                A[d].append(best)
                loc[d] = self.cust[best]
                left.remove(best)
        return A

    def step(self):
        if self.fin:
            return True

        for d, pos in enumerate(self.dpos):
            rem = self.spd
            while rem > self.eps and self.assign[d]:
                ci = self.assign[d][0]
                tgt = self.routes[ci][self.idx[d]]

                vec0 = tgt[0] - pos[0]
                vec1 = tgt[1] - pos[1]
                dist = float(np.hypot(vec0, vec1))

                if dist < self.eps:
                    self.idx[d] += 1
                    if self.idx[d] == 2:
                        self.done[ci] = True
                        self.assign[d].pop(0)
                        self.idx[d] = 0
                    continue

                stepv = min(rem, dist)
                frac = stepv / dist
                pos = (pos[0] + vec0 * frac, pos[1] + vec1 * frac)
                rem -= stepv

            self.dpos[d] = pos

        self.fin = all(self.done)
        return self.fin
