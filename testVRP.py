import unittest as ut
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.distance import pdist
from scipy.spatial.distance import squareform
from VRP import VRP


class TestVRP(ut.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestVRP, self).__init__(*args, **kwargs)
        self.b_loc = pd.read_csv('testdata/BasicLocations.csv')
        self.b_dist = squareform(pdist(self.b_loc.values[:, 1:3])).astype(int)
        self.b_car = pd.read_csv('testdata/BasicVehicles.csv')
        self.b_capa = []
        for i, row in self.b_car.iterrows():
            self.b_capa.extend([row['capacity']] * row['number'])

        self.colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
        self.result = {}

    def test_solve_both(self):
        vrp = VRP(
            self.b_dist, self.b_loc['demand'], depot=0,
            num_vehicles=len(self.b_capa),
            vehicle_capacities=self.b_capa
        )
        vrp.solve()
        self.result = vrp.result()
        print(self.result['routes'])
        self.draw()

    def test_solve_nonly(self):
        vrp = VRP(
            self.b_dist, self.b_loc['demand'], depot=0,
            num_vehicles=len(self.b_capa)
        )
        vrp.solve()
        self.result = vrp.result()
        print(self.result['routes'])
        print(self.result['vehicle_capacities'])
        self.draw()

    def test_solve_conly(self):
        vrp = VRP(
            self.b_dist, self.b_loc['demand'], depot=0,
            vehicle_capacities=[14, 16]
        )
        vrp.solve()
        self.result = vrp.result()
        print(self.result['routes'])
        print(self.result['vehicle_capacities'])
        self.draw()

    def test_solve_none(self):
        vrp = VRP(self.b_dist, self.b_loc['demand'], depot=0)
        vrp.solve()
        self.result = vrp.result()
        print(self.result['routes'])
        print(self.result['vehicle_capacities'])
        print(self.result['num_vehicles'])
        self.draw()

    def test_solve_classic_vrp(self):
        vrp = VRP(self.b_dist, demands=0, depot=0, num_vehicles=4)
        vrp._solve_classic_vrp()
        self.result = vrp.result()
        print(self.result['routes'])
        self.draw()

    def test_solve_vrp_with_capacity_constraint(self):
        vrp = VRP(
            self.b_dist, self.b_loc['demand'], depot=0,
            num_vehicles=len(self.b_capa),
            vehicle_capacities=self.b_capa
        )
        vrp._solve_vrp_with_capacity_constraint()
        self.result = vrp.result()
        print(self.result['routes'])
        self.draw()

    def draw(self):
        for i, row in self.b_loc.iterrows():
            edgecolor = self.colors[1] if i == self.result['depot'] else self.colors[0]
            plt.scatter(
                row['x'], row['y'], s=750,
                facecolor='white', edgecolor=edgecolor
            )
            plt.text(
                row['x'], row['y'], row['name'],
                fontsize=15, ha='center', va='center'
            )
        for i, route in enumerate(self.result['routes']):
            for j in range(len(route)-1):
                loc_start = self.b_loc.loc[route[j]]
                loc_end = self.b_loc.loc[route[j + 1]]
                pos_start = loc_start[1:3].values
                pos_end = loc_end[1:3].values
                len_percent = 60 / 100
                arrow_start = pos_start + (pos_end - pos_start) * (1 - len_percent) / 2
                arrow_d = (pos_end - pos_start) * len_percent
                plt.arrow(
                    *arrow_start, *arrow_d, head_width=20, head_length=35,
                    fc=self.colors[-i], ec=self.colors[-i], zorder=0,
                    length_includes_head=True
                )

        plt.gca().axis('off')
        plt.savefig('routing.png')


if __name__ == '__main__':
    ut.main()
