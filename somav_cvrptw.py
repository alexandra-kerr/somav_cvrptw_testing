# This Python file uses the following encoding: utf-8
# Copyright 2015 Tin Arm Engineering AB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Capacitated Vehicle Routing Problem with Time Windows (and optional orders).

   This is a sample using the routing library python wrapper to solve a
   CVRPTW problem.
   A description of the problem can be found here:
   http://en.wikipedia.org/wiki/Vehicle_routing_problem.
   The variant which is tackled by this model includes a capacity dimension,
   time windows and optional orders, with a penalty cost if orders are not
   performed.
   To help explore the problem, two classes are provided Customers() and
   Vehicles(): used to randomly locate orders and depots, and to randomly
   generate demands, time-window constraints and vehicles.
   Distances are computed using the Great Circle distances. Distances are in km
   and times in seconds.

   A function for the displaying of the vehicle plan
   display_vehicle_output

   The optimization engine uses local search to improve solutions, first
   solutions being generated using a cheapest addition heuristic.
   Numpy and Matplotlib are required for the problem creation and display.
"""

#NOTE: Please refer to the README for ORTools before making any changes to this file
import os
import numpy as np
from collections import namedtuple
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from datetime import datetime, timedelta
import sys


class Customers():
  """
        A class that generates and holds customers information.

        Randomly normally distribute a number of customers and locations within
        a region described by a rectangle.  Generate a random demand for each
        customer. Generate a random time window for each customer.
        May either be initiated with the extents, as a dictionary describing
        two corners of a rectangle in latitude and longitude OR as a center
        point (lat, lon), and box_size in km.  The default arguments are for a
        10 x 10 km square centered in Sheffield).

        Args: extents (Optional[Dict]): A dictionary describing a rectangle in
        latitude and longitude with the keys 'llcrnrlat', 'llcrnrlon' &
        'urcrnrlat' & 'urcrnrlat'  center (Optional(Tuple): A tuple of
        (latitude, longitude) describing the centre of the rectangle.  box_size
        (Optional float: The length in km of the box's sides.  num_stops (int):
        The number of customers, including the depots that are placed normally
        distributed in the rectangle.  min_demand (int): Lower limit on the
        randomly generated demand at each customer.  max_demand (int): Upper
        limit on the randomly generated demand at each customer.
            min_tw: shortest random time window for a customer, in hours.
            max_tw: longest random time window for a customer, in hours.
        Examples: To place 100 customers randomly within 100 km x 100 km
        rectangle, centered in the default location, with a random demand of
        between 5 and 10 units:  >>> customers = Customers(num_stops=100,
        box_size=100, ...                 min_demand=5, max_demand=10)
        alternatively, to place 75 customers in the same area with default
        arguments for demand:  >>> extents = {'urcrnrlon': 0.03403, 'llcrnrlon':
        -2.98325, ...     'urcrnrlat': 54.28127, 'llcrnrlat': 52.48150} >>>
        customers = Customers(num_stops=75, extents=extents)
  """

  def __init__(self,
               extents=None,
               center=(53.381393, -1.474611),
               box_size=10,
               num_stops=100,
               lats=None,
               lons=None,
               dems=None,
               maxFlightTime=15,
               #min_demand=0,
               #max_demand=25,
               min_tw=1,
               max_tw=5):
    self.number = num_stops  #: The number of customers and depots
    #: Location, a named tuple for locations.
    Location = namedtuple('Location', ['lat', 'lon'])
    if extents is not None:
      self.extents = extents  #: The lower left and upper right points
      #: Location[lat,lon]: the centre point of the area.
      self.center = Location(
          extents['urcrnrlat'] -
          0.5 * (extents['urcrnrlat'] - extents['llcrnrlat']),
          extents['urcrnrlon'] -
          0.5 * (extents['urcrnrlon'] - extents['llcrnrlon']))
    else:
      #: Location[lat,lon]: the centre point of the area.
      (clat, clon) = self.center = Location(center[0], center[1])
      rad_earth = 6367  # km
      circ_earth = np.pi * rad_earth
      #: The lower left and upper right points
      self.extents = {
          'llcrnrlon': (
              clon - 180 * box_size / (circ_earth * np.cos(np.deg2rad(clat)))),
          'llcrnrlat':
              clat - 180 * box_size / circ_earth,
          'urcrnrlon': (
              clon + 180 * box_size / (circ_earth * np.cos(np.deg2rad(clat)))),
          'urcrnrlat':
              clat + 180 * box_size / circ_earth
      }

    #Add latitudes and longitudes
    if lats is not None and lons is not None:
      self.lats = lats
      self.lons = lons
      stops = np.array(range(0, num_stops))
      if dems is None:  #If no demands are loaded, make all 0
        demands = np.zeros(num_stops)
      else:
        demands = dems
    else:
      # The 'name' of the stop, indexed from 0 to num_stops-1
      stops = np.array(range(0, num_stops))
      # normaly distributed random distribution of stops within the box
      stdv = 6  # the number of standard deviations 99.9% will be within +-3
      lats = (self.extents['llcrnrlat'] + np.random.randn(num_stops) *
              (self.extents['urcrnrlat'] - self.extents['llcrnrlat']) / stdv)
      lons = (self.extents['llcrnrlon'] + np.random.randn(num_stops) *
              (self.extents['urcrnrlon'] - self.extents['llcrnrlon']) / stdv)
      # uniformly distributed integer demands.
      demmands = 0#np.random.randint(min_demand, max_demand, num_stops)
    
    flightTime = maxFlightTime * 60 #a 15 min. period
    self.time_horizon = flightTime

    ## The customers demand min_tw to max_tw hour time window for each
    ## delivery
    #time_windows = np.random.random_integers(min_tw * 3600, max_tw * 3600,
    #                                         num_stops)
    ## The last time a delivery window can start
    #latest_time = self.time_horizon - time_windows
    #start_times = [None for o in time_windows]
    #stop_times = [None for o in time_windows]
    ## Make random timedeltas, nominaly from the start of the day.
    #for idx in range(self.number):
    #  stime = int(np.random.random_integers(0, latest_time[idx]))
    #  start_times[idx] = timedelta(seconds=stime)
    #  stop_times[idx] = (
    #      start_times[idx] + timedelta(seconds=int(time_windows[idx])))
    start_times = [None for o in range(num_stops)]
    stop_times = [None for o in range(num_stops)]
    for index in range(self.number):
      start_times[index] = timedelta(seconds=0)
      stop_times[index] = timedelta(seconds=flightTime)
    ## A named tuple for the customer
    #Customer = namedtuple(
    #    'Customer',
    #   [
    #        'index',  # the index of the stop
    #        'demand',  # the demand for the stop
    #        'lat',  # the latitude of the stop
    #        'lon',  # the longitude of the stop
    #        'tw_open',  # timedelta window open
    #        'tw_close'
    #    ])  # timedelta window cls
    Customer = namedtuple("Customer", ['index', #the index of the stop
                                        'demand', #the demand for the stop,
                                        'lat',  #the latitude of the stop
                                        'lon',  #the longitude of the stop
                                        'tw_open',  #timedelta window open
                                        'tw_close'  #timedelta window close
                                      ])

    #self.customers = [
    #    Customer(idx, dem, lat, lon, tw_open, tw_close)
    #    for idx, dem, lat, lon, tw_open, tw_close in zip(
    #        stops, demmands, lats, lons, start_times, stop_times)
    #]
    self.customers = [Customer(index, dem, lat, lon, tw_open, tw_close) for 
                        index, dem, lat, lon, tw_open, tw_close in
                        zip(stops,demands,lats, lons, start_times,stop_times)]

    # The number of seconds needed to 'unload' 1 unit of goods.
    self.service_time_per_dem = 3  # seconds

  def central_start_node(self, invert=False):
    """
        Return a random starting node, with probability weighted by distance
        from the centre of the extents, so that a central starting node is
        likely.

        Args: invert (Optional bool): When True, a peripheral starting node is
        most likely.

        Returns:
            int: a node index.

        Examples:
            >>> customers.central_start_node(invert=True)
            42
        """
    #num_nodes = len(self.customers)
    #dist = np.empty((num_nodes, 1))
    #for idx_to in range(num_nodes):
    #  dist[idx_to] = self._haversine(self.center.lon, self.center.lat,
    #                                 self.customers[idx_to].lon,
    #                                 self.customers[idx_to].lat)
    #furthest = np.max(dist)

    #if invert:
    #  prob = dist * 1.0 / sum(dist)
    #else:
    #  prob = (furthest - dist * 1.0) / sum(furthest - dist)
    #indexes = np.array([range(num_nodes)])
    #start_node = np.random.choice(
    #    indexes.flatten(), size=1, replace=True, p=prob.flatten())
    #return start_node[0]
    
    #The depot is the first node
    return 0

  def make_distance_mat(self, method='haversine'):
    """
        Return a distance matrix and make it a member of Customer, using the
        method given in the call. Currently only Haversine (GC distance) is
        implemented, but Manhattan, or using a maps API could be added here.
        Raises an AssertionError for all other methods.

        Args: method (Optional[str]): method of distance calculation to use. The
        Haversine formula is the only method implemented.

        Returns:
            Numpy array of node to node distances.

        Examples:
            >>> dist_mat = customers.make_distance_mat(method='haversine')
            >>> dist_mat = customers.make_distance_mat(method='manhattan')
            AssertionError
        """
    self.distmat = np.zeros((self.number, self.number))
    methods = {'haversine': self._haversine}
    assert (method in methods)
    for frm_idx in range(self.number):
      for to_idx in range(self.number):
        if frm_idx != to_idx:
          frm_c = self.customers[frm_idx]
          to_c = self.customers[to_idx]
          self.distmat[frm_idx, to_idx] = self._haversine(
              frm_c.lon, frm_c.lat, to_c.lon, to_c.lat)
    return (self.distmat)

  def _haversine(self, lon1, lat1, lon2, lat2):
    """
        Calculate the great circle distance between two points
        on the earth specified in decimal degrees of latitude and longitude.
        https://en.wikipedia.org/wiki/Haversine_formula

        Args:
            lon1: longitude of pt 1,
            lat1: latitude of pt 1,
            lon2: longitude of pt 2,
            lat2: latitude of pt 2

        Returns:
            the distace in km between pt1 and pt2
        """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(np.radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = (
        np.sin(dlat / 2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2)**2)
    c = 2 * np.arcsin(np.sqrt(a))

    # 6367 km is the radius of the Earth
    m = 6367 * 1000 * c #convert to meters
    return m

  def get_total_demand(self):
    """
        Return the total demand of all customers.
        """
    return (sum([c.demand for c in self.customers]))

  def return_dist_callback(self, **kwargs):
    """
        Return a callback function for the distance matrix.

        Args: **kwargs: Arbitrary keyword arguments passed on to
        make_distance_mat()

        Returns:
            function: dist_return(a,b) A function that takes the 'from' node
                index and the 'to' node index and returns the distance in km.
        """
    self.make_distance_mat(**kwargs)

    def dist_return(a, b):
      return (self.distmat[a][b])

    return dist_return

  def return_dem_callback(self):
    """
        Return a callback function that gives the demands.

        Returns:
            function: dem_return(a,b) A function that takes the 'from' node
                index and the 'to' node index and returns the distance in km.
        """

    def dem_return(a, b):
      return (self.customers[a].demand)

    return dem_return

  def zero_depot_demands(self, depot):
    """
        Zero out the demands and time windows of depot.  The Depots do not have
        demands or time windows so this function clears them.

        Args:  depot (int): index of the stop to modify into a depot.
        Examples:  >>> customers.zero_depot_demands(5) >>>
        customers.customers[5].demand == 0 True
    """
    start_depot = self.customers[depot]
    self.customers[depot] = start_depot._replace(
        demand=0, tw_open=None, tw_close=None)

  def make_service_time_call_callback(self):
    """
        Return a callback function that provides the time spent servicing the
        customer.  Here is it proportional to the demand given by
        self.service_time_per_dem, default 300 seconds per unit demand.

        Returns:
            function [dem_return(a, b)]: A function that takes the from/a node
                index and the to/b node index and returns the service time at a

        """

    def service_time_return(a, b):
      #return (self.customers[a].demand * self.service_time_per_dem)
      return(self.service_time_per_dem)

    return service_time_return

  def make_transit_time_callback(self, speed_kmph=5):
    """
        Creates a callback function for transit time. Assuming an average
        speed of speed_kmph
        Args:
            speed_kmph: the average speed in m/s

        Returns:
            function [tranit_time_return(a, b)]: A function that takes the
                from/a node index and the to/b node index and returns the
                tranit time from a to b.
        """

    def tranit_time_return(a, b):
      return (self.distmat[a][b] / (speed_kmph * 1.0))

    return tranit_time_return


class Vehicles():
  """
    A Class to create and hold vehicle information.

    The Vehicles in a CVRPTW problem service the customers and belong to a
    depot. The class Vehicles creates a list of named tuples describing the
    Vehicles.  The main characteristics are the vehicle capacity, fixed cost,
    and cost per km.  The fixed cost of using a certain type of vehicles can be
    higher or lower than others. If a vehicle is used, i.e. this vehicle serves
    at least one node, then this cost is added to the objective function.

    Note:
        If numpy arrays are given for capacity and cost, then they must be of
        the same length, and the number of vehicles are infered from them.
        If scalars are given, the fleet is homogenious, and the number of
        vehicles is determied by number.

    Args: capacity (scalar or numpy array): The integer capacity of demand
    units.  cost (scalar or numpy array): The fixed cost of the vehicle.  number
    (Optional [int]): The number of vehicles in a homogenious fleet.
  """

  def __init__(self, capacity=1e6, cost=100, number=None):

    Vehicle = namedtuple('Vehicle', ['index', 'capacity', 'cost'])

    if number is None:
      self.number = np.size(capacity)
    else:
      self.number = number
    idxs = np.array(range(0, self.number))

    if np.isscalar(capacity):
      capacities = capacity * np.ones_like(idxs)
    elif np.size(capacity) != np.size(capacity):
      print('capacity is neither scalar, nor the same size as num!')
    else:
      capacities = capacity

    if np.isscalar(cost):
      costs = cost * np.ones_like(idxs)
    elif np.size(cost) != self.number:
      print(np.size(cost))
      print('cost is neither scalar, nor the same size as num!')
    else:
      costs = cost

    self.vehicles = [
        Vehicle(idx, capacity, cost)
        for idx, capacity, cost in zip(idxs, capacities, costs)
    ]

  def get_total_capacity(self):
    return (sum([c.capacity for c in self.vehicles]))

  def return_starting_callback(self, customers, sameStartFinish=False):
    # create a different starting and finishing depot for each vehicle
    self.starts = [
        int(customers.central_start_node()) for o in range(self.number)
    ]
    if sameStartFinish:
      self.ends = self.starts
    else:
      self.ends = [
          int(customers.central_start_node(invert=True))
          for o in range(self.number)
      ]
    # the depots will not have demands, so zero them.
    for depot in self.starts:
      customers.zero_depot_demands(depot)
    for depot in self.ends:
      customers.zero_depot_demands(depot)

    def start_return(v):
      return (self.starts[v])

    return start_return

def vehicle_output_string(routing, plan):
  """
    Return a string displaying the output of the routing instance and
    assignment (plan).

    Args: routing (ortools.constraint_solver.pywrapcp.RoutingModel): routing.
    plan (ortools.constraint_solver.pywrapcp.Assignment): the assignment.

    Returns:
        (string) plan_output: describing each vehicle's plan.

        (List) dropped: list of dropped orders.

    """
  dropped = []
  for order in range(routing.Size()):
    if (plan.Value(routing.NextVar(order)) == order):
      dropped.append(str(order))

  capacity_dimension = routing.GetDimensionOrDie('Capacity')
  time_dimension = routing.GetDimensionOrDie('Time')
  plan_output = ''

  for route_number in range(routing.vehicles()):
    order = routing.Start(route_number)
    plan_output += 'Route {0}:'.format(route_number)
    if routing.IsEnd(plan.Value(routing.NextVar(order))):
      plan_output += ' Empty \n'
    else:
      while True:
        load_var = capacity_dimension.CumulVar(order)
        time_var = time_dimension.CumulVar(order)
        plan_output += \
            ' {order} Load({load}) Time({tmin}, {tmax}) -> '.format(
                order=order,
                load=plan.Value(load_var),
                tmin=str(timedelta(seconds=plan.Min(time_var))),
                tmax=str(timedelta(seconds=plan.Max(time_var))))

        if routing.IsEnd(order):
          plan_output += ' EndRoute {0}. \n'.format(route_number)
          break
        order = plan.Value(routing.NextVar(order))
    plan_output += '\n'

  return (plan_output, dropped)


def build_vehicle_route(routing, plan, customers, veh_number):
  """
    Build a route for a vehicle by starting at the strat node and
    continuing to the end node.

    Args: routing (ortools.constraint_solver.pywrapcp.RoutingModel): routing.
    plan (ortools.constraint_solver.pywrapcp.Assignment): the assignment.
    customers (Customers): the customers instance.  veh_number (int): index of
    the vehicle

    Returns:
        (List) route: indexes of the customers for vehicle veh_number
    """
  veh_used = routing.IsVehicleUsed(plan, veh_number)
  print('Vehicle {0} is used {1}'.format(veh_number, veh_used))
  if veh_used:
    route = []
    node = routing.Start(veh_number)  # Get the starting node index
    route.append(customers.customers[routing.IndexToNode(node)])
    while not routing.IsEnd(node):
      route.append(customers.customers[routing.IndexToNode(node)])
      node = plan.Value(routing.NextVar(node))

    route.append(customers.customers[routing.IndexToNode(node)])
    return route
  else:
    return None

def buildQGCReturn(routing, plan, waypoints, vehicleIds, depotCount):
  """
  Builds the route for each vehicle in a way that QGC can interpret and prints it to stdout
  so that QGC can receive it.
  """
  dropped = "*Dropped:"
  dropCount = 0
  for order in range(routing.Size()):
    if (plan.Value(routing.NextVar(order)) == order):
      if dropCount > 0:
        dropped += ","
      dropped += str(order-depotCount+1)
      dropCount += 1
  
  if dropCount > 0:
    print("{}".format(dropped)) 

  for routeNum in range(routing.vehicles()):
    plan_output = '*Route#' + str(vehicleIds[routeNum]) + ":"

    order = routing.Start(routeNum)
    if routing.IsEnd(plan.Value(routing.NextVar(order))):
      continue
    else:
      count = 0
      while True:
        if count > 0:
          plan_output += ","
        if order in range(len(waypoints)):
          plan_output += "{0:.8f}+{1:.8f}".format(waypoints[order][0], waypoints[order][1]) #str(waypoints[order][0]) + "+" + str(waypoints[order][1])
        else:
          #the waypoint is a depot (that is a fake order #) return the depot location
          depotIndex = order-len(waypoints)
          plan_output += "{0:.8f}+{1:.8f}".format(waypoints[depotIndex][0], waypoints[depotIndex][1]) #str(waypoints[depotIndex][0]) + "+" + str(waypoints[depotIndex][1])

        if routing.IsEnd(order):
          break

        count += 1
        order = plan.Value(routing.NextVar(order))

    print("{}".format(plan_output))  


########
# Parse Input from argv
########
def parseWaypoints(tokens):
  points = tokens.split(",")
  locations = []
  for point in points:
    ptSplit = point.split("+")
    lat = float(ptSplit[0])
    lon = float(ptSplit[1])
    locations.append([lat, lon])
  return locations

def parseDepots(tokens):
  depots = []
  points = tokens.split(",")
  for pt in points:
    coords = pt.split("+")
    depots.append([float(coords[0]), float(coords[1])])
  
  return depots

def main():
  vehicleList = None
  waypointsList = None
  depotList = None
  flightTime = 15
  maxSpeed = 5
  for arg in sys.argv[1:]:
    #print('{}'.format(arg))
    tokens = arg.split(":")
    if tokens[0] == "Vehicles":
      vehicleList = (tokens[1].split(","))
    elif tokens[0] == "Waypoints":
      waypointsList = parseWaypoints(tokens[1])
    elif tokens[0] == "Depot":
      depotList = parseDepots(tokens[1])
    elif tokens[0] == "Flight Time":
      flightTime = int(tokens[1])
    elif tokens[0] == "Speed":
      maxSpeed = int(tokens[1])

  
  print("Vehicles: {}".format(vehicleList))
  print("Waypoints: {}".format(waypointsList))
  print("Depot: {}".format(depotList))

  #Waypoints have no demaonds
  demands = None

  #Add the depot(s) to the begining of the waypoints list
  depotIndices = []
  for idx in range(len(depotList)):
    waypointsList.insert(idx, depotList[idx])
    depotIndices.append(idx)

  # Create a set of customer, (and depot) stops.
  customers = Customers(
      num_stops=len(waypointsList),
      lats=[loc[0] for loc in waypointsList],
      lons=[loc[1] for loc in waypointsList],
      dems=demands,
      maxFlightTime=flightTime,
      #min_demand=1,
      #max_demand=15,
      #box_size=40,
      min_tw=15,
      max_tw=15)

  # Create callback fns for distances, demands, service and transit-times.
  dist_fn = customers.return_dist_callback()
  dem_fn = customers.return_dem_callback()
  #serv_time_fn = customers.make_service_time_call_callback()

  #Turn the 
  ## Create a list of inhomgenious vehicle capacities as integer units.
  #capacity = [50, 175, 200, 250]
  capacity = []
  for veh in vehicleList:
    capacity.append(100)
  
  transit_time_fn = customers.make_transit_time_callback(speed_kmph=maxSpeed)

  def tot_time_fn(a, b):
    """
        The time function we want is both transit time and service time.
        """
    #return serv_time_fn(a, b) + transit_time_fn(a, b)
    return transit_time_fn(a,b)

  # Create a list of inhomogenious fixed vehicle costs.
  cost = [int(100) for c in capacity]

  # Create a set of vehicles, the number set by the length of capacity.
  vehicles = Vehicles(capacity=capacity, cost=cost)

  ## check to see that the problem is feasible, if we don't have enough
  ## vehicles to cover the demand, there is no point in going further.
  #assert (customers.get_total_demand() < vehicles.get_total_capacity())

  # Set the starting nodes, and create a callback fn for the starting node.
  start_fn = vehicles.return_starting_callback(customers, sameStartFinish=True)

  # Set model parameters
  model_parameters = pywrapcp.RoutingModel.DefaultModelParameters()

  # The solver parameters can be accessed from the model parameters. For example :
  #   model_parameters.solver_parameters.CopyFrom(
  #       pywrapcp.Solver.DefaultSolverParameters())
  #    model_parameters.solver_parameters.trace_propagation = True

  # Make the routing model instance.
  # if we have multiple depots, then each vehicle has a differnt start/end location, pass those in 
  # instead of start/ends in the vehicle class
  routing = pywrapcp.RoutingModel(
       customers.number, #in tnumber
       vehicles.number,  #int number
       depotIndices,            #list of start depot
       depotIndices,            #list of end depots
       model_parameters)
  
  #routing = pywrapcp.RoutingModel(
  #     customers.number,  # int number
  #     vehicles.number,  # int number
  #     vehicles.starts,  # List of int start depot
  #     vehicles.ends,  # List of int end depot
  #     model_parameters)

  parameters = routing.DefaultSearchParameters()
  # Setting first solution heuristic (cheapest addition).
  parameters.first_solution_strategy = (
      routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
  # Disabling Large Neighborhood Search, (this is the default behaviour)
  parameters.local_search_operators.use_path_lns = False
  parameters.local_search_operators.use_inactive_lns = False
  # Routing: forbids use of TSPOpt neighborhood,
  parameters.local_search_operators.use_tsp_opt = False

  parameters.time_limit_ms = 30 * 1000  # 30 seconds
  parameters.use_light_propagation = False
  # parameters.log_search = True

  # Set the cost function (distance callback) for each arc, homogenious for
  # all vehicles.
  routing.SetArcCostEvaluatorOfAllVehicles(dist_fn)

  # Set vehicle costs for each vehicle, not homogenious.
  for veh in vehicles.vehicles:
    routing.SetFixedCostOfVehicle(veh.cost, int(veh.index))

  # Add a dimension for vehicle capacities
  null_capacity_slack = 0
  routing.AddDimensionWithVehicleCapacity(
      dem_fn,  # demand callback
      null_capacity_slack,
      capacity,  # capacity array
      True,
      'Capacity')
  # Add a dimension for time and a limit on the total time_horizon
  routing.AddDimension(
      tot_time_fn,  # total time function callback
      customers.time_horizon,
      customers.time_horizon,
      True,
      'Time')

  time_dimension = routing.GetDimensionOrDie('Time')
  for cust in customers.customers:
    if cust.tw_open is not None:
      time_dimension.CumulVar(routing.NodeToIndex(cust.index)).SetRange(
          cust.tw_open.seconds, cust.tw_close.seconds)
  """
     To allow the dropping of orders, we add disjunctions to all the customer
    nodes. Each disjunction is a list of 1 index, which allows that customer to
    be active or not, with a penalty if not. The penalty should be larger
    than the cost of servicing that customer, or it will always be dropped!
    """
  # To add disjunctions just to the customers, make a list of non-depots.
  non_depot = set(range(customers.number))
  non_depot.difference_update(depotIndices)
  non_depot.difference_update(depotIndices)
  penalty = 400000  # The cost for dropping a node from the plan.
  nodes = [routing.AddDisjunction([int(c)], penalty) for c in non_depot]

  # This is how you would implement partial routes if you already knew part
  # of a feasible solution for example:
  # partial = np.random.choice(list(non_depot), size=(4,5), replace=False)

  # routing.CloseModel()
  # partial_list = [partial[0,:].tolist(),
  #                 partial[1,:].tolist(),
  #                 partial[2,:].tolist(),
  #                 partial[3,:].tolist(),
  #                 [],[],[],[]]
  # print(routing.ApplyLocksToAllVehicles(partial_list, False))

  # Solve the problem !
  assignment = routing.SolveWithParameters(parameters)

  # The rest is all optional for saving, printing or plotting the solution.
  if assignment:
    #print('The Objective Value is {0}'.format(assignment.ObjectiveValue()))

    buildQGCReturn(routing, assignment, waypointsList, vehicleList, len(depotList))
    #plan_output, dropped = vehicle_output_string(routing, assignment)
    #print(plan_output)
    #print('dropped nodes: ' + ', '.join(dropped))

    # you could print debug information like this:
    # print(routing.DebugOutputAssignment(assignment, 'Capacity'))

    #vehicle_routes = {}
    #for veh in range(vehicles.number):
    #  vehicle_routes[veh] = build_vehicle_route(routing, assignment, customers,veh)


  else:
    print('No assignment')


if __name__ == '__main__':
  main()
