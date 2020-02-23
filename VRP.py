from ortools.constraint_solver  import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import numpy as np


class VRP:
    def __init__(
            self, distance_matrix, demands, depot=0,
            num_vehicles=None, vehicle_capacities=None,
            time_limit=20, strategy='AUTOMATIC'
    ):
        self.distance_matrix = distance_matrix
        self.demands = demands
        self.depot = depot
        self.time_limit = time_limit
        self.strategy = strategy
        self.routes = []
        self.status = 0
        if num_vehicles is not None and vehicle_capacities is not None:
            if num_vehicles != len(vehicle_capacities):
                raise ValueError('`num_vehicles` must be equal to `len(vehicle_capacities`'\
                    ' when both provided')
            self.mode = 'BOTH'
            self.num_vehicles = num_vehicles
            self.vehicle_capacities = vehicle_capacities
        elif num_vehicles is not None and vehicle_capacities is None:
            self.mode = 'NONLY'
            self.num_vehicles = num_vehicles
            self.vehicle_capacities = []
        elif num_vehicles is None and vehicle_capacities is not None:
            if len(vehicle_capacities) != len(set(vehicle_capacities)):
                raise ValueError('`vehicle_capacities` must contain unique numbers '\
                    ' when `num_vehicles` is not provided')
            self.mode = 'CONLY'
            self.capacity_options = vehicle_capacities
            self.num_vehicles = None
            self.vehicle_capacities = vehicle_capacities
        else:
            self.mode = 'NONE'
            self.num_vehicles = len(distance_matrix) // 5
            self.vehicle_capacities = None

    def solve(self):
        if self.mode == 'BOTH':
            self._solve_vrp_with_capacity_constraint()
        elif self.mode == 'CONLY':
            capacity_options = self.vehicle_capacities
            max_capacity = max(self.vehicle_capacities)
            self.num_vehicles = int(np.ceil(np.sum(self.demands) / max_capacity))
            self.vehicle_capacities = self.num_vehicles * [max_capacity]
            self._solve_vrp_with_capacity_constraint()
            self._sum_demands()
            self.vehicle_capacities = np.array(self.vehicle_capacities)
            self.capacity_options = sorted(self.capacity_options)
            self.vehicle_capacities[self.vehicle_capacities <= capacity_options[0]] = capacity_options[0]
            for i in range(1, len(self.capacity_options)):
                bound1 = capacity_options[i-1] < self.vehicle_capacities
                bound2 = self.vehicle_capacities <= capacity_options[i]
                bound = bound1 & bound2
                self.vehicle_capacities[bound] = capacity_options[i]
            self.vehicle_capacities = self.vehicle_capacities.tolist()
        else:
            self._solve_classic_vrp()
            self._sum_demands()

    def result(self):
        return {
            'distance_matrix': self.distance_matrix,
            'demands': self.demands,
            'depot': self.depot,
            'num_vehicles': self.num_vehicles,
            'vehicle_capacities': self.vehicle_capacities,
            'routes': self.routes
        }

    def _solve_classic_vrp(self):
        manager = pywrapcp.RoutingIndexManager(
            len(self.distance_matrix),
            self.num_vehicles,
            self.depot
        )
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return self.distance_matrix[from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        dimension_name = 'Distance'
        routing.AddDimension(transit_callback_index, 0, 3000, True, dimension_name)
        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.time_limit.seconds = self.time_limit
        search_parameters.first_solution_strategy = \
        getattr(routing_enums_pb2.FirstSolutionStrategy, self.strategy)

        solution = routing.SolveWithParameters(search_parameters)
        if solution:
            self.routes = []
            for route_nbr in range(self.num_vehicles):
                index = routing.Start(route_nbr)
                route = [manager.IndexToNode(index)]
                while not routing.IsEnd(index):
                    index = solution.Value(routing.NextVar(index))
                    route.append(manager.IndexToNode(index))
                self.routes.append(route)

    def _solve_vrp_with_capacity_constraint(self):
        manager = pywrapcp.RoutingIndexManager(
            len(self.distance_matrix),
            self.num_vehicles,
            self.depot
        )
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return self.distance_matrix[from_node][to_node]

        def demand_callback(from_index):
            from_node = manager.IndexToNode(from_index)
            return self.demands[from_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0, 
            self.vehicle_capacities,
            True,
            'Capacity'
        )

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.time_limit.seconds = self.time_limit
        search_parameters.first_solution_strategy = \
        getattr(routing_enums_pb2.FirstSolutionStrategy, self.strategy)

        solution = routing.SolveWithParameters(search_parameters)
        if solution:
            self.routes = []
            for route_nbr in range(self.num_vehicles):
                index = routing.Start(route_nbr)
                route = [manager.IndexToNode(index)]
                while not routing.IsEnd(index):
                    index = solution.Value(routing.NextVar(index))
                    route.append(manager.IndexToNode(index))
                self.routes.append(route)

    def _sum_demands(self):
        self.vehicle_capacities = []
        for route in self.routes:
            capacity = 0
            for point in route:
                capacity += self.demands[point]
            self.vehicle_capacities.append(capacity)