"""This module contains my solution for HashCode 2018"""
# Code for our small team effort in Google Hash 2018
# (no link as links to competition are sure to expire, but.. Google it)

# Input file formats

#  ● a – the row of the start intersection (0 ≤ a < R)
# ● b – the column of the start intersection (0 ≤ b < C)
# ● x – the row of the finish intersection (0 ≤ x < R)
# ● y – the column of the finish intersection (0 ≤ y < C)
# ● s – the earliest start(0 ≤ s < T)
# ● f – the latest finish (0 ≤ f ≤ T) , (f ≥ s + |x − a| + |y − b|)
# ○ note that f can be equal to T – this makes the latest finish equal to the end of the simulation

# First line of input file
# ● R – number of rows of the grid (1 ≤ R ≤ 10000)
# ● C – number of columns of the grid (1 ≤ C ≤ 10000)
# ● F – number of vehicles in the fleet (1 ≤ F ≤ 1000)
# ● N – number of rides (1 ≤ N ≤ 10000)
# ● B – per-ride bonus for starting the ride on time (1 ≤ B ≤ 10000)
# ● T – number of steps in the simulation (1 ≤ T ≤ 10 )

import os
import pandas as pd


def calc_distance(pos_x, pos_a, pos_y, pos_b):
    """"Calculate distance between two points"""
    distance = abs(pos_x - pos_a) + abs(pos_y - pos_b)
    return distance


def vehicle_ride_start_distance(vehicle, ride):
    """Helper function for dist calc using vehicle and ride class"""
    dist = calc_distance(vehicle.pos_x, vehicle.pos_y, ride.pos_a, ride.pos_b)
    return dist


class Simulation:

    """Class to store the current simulation properties"""

    def __init__(self, row):
        self.rows = row[0]
        self.cols = row[1]
        self.no_of_vehicles = row[2]
        self.rides = row[3]
        self.bonus = row[4]
        self.steps = row[5]

    def dummy_1(self):
        """Function to fool pylint"""
        self.steps = self.steps
        print('dummy 1')

    def dummy_2(self):
        """function to fool pylint"""
        self.steps = self.steps
        print('dummy 2')


class Vehicle:
    """Vehicle class"""

    def __init__(self, vehicle_number, simul):
        self.pos_x = 0
        self.pos_y = 0
        self.vehicle_number = vehicle_number
        self.simul = simul
        self.ride = ''
        self.start_of_ride = -1  # timestep of start of ride
        self.end_of_ride = 0  # timestep of end of ride
        self.end_x = 0
        self.end_y = 0
        self.status = 'ready'
        self.ride_history = []
        self.assigned_rides = []

    def get_next_ride(self, time_step, rides, vehicles):

        """Function to look for the next ride"""

        # Check for assignedRides
        # This is not used yet, was part of design to check points for final submission

        if not self.assigned_rides:

            # Get rides for which this vehicle is a candidate
            new_ride = ''
            target_rides_calc = [rc for rc in rides if rc.vehicle == -1 and rc.finish_time >
                                 time_step]
            target_rides = [(target_ride, arr_start_time(self, target_ride, time_step, 9000)) for
                            target_ride in target_rides_calc if target_ride.start_time > time_step]
            target_rides = [target_ride for target_ride in target_rides if target_ride[1] >= 0]

            if target_rides:
                # Find ride with lowest cost for start bonus
                cost = 999999999999
                for target_ride in target_rides:
                    if target_ride[1] < cost:
                        all_vehicles_cost = min([arr_start_time(v, target_ride[0], time_step,
                                                                9000) for v in vehicles if
                                                 vehicles != self])
                        if target_ride[1] > all_vehicles_cost:
                            new_ride = target_ride[0]
                            cost = target_ride[1]

            if new_ride == '':
                # Find ride with lowest cost for arriving at finish just on time
                target_rides = [(target_ride, arr_last_time(self, target_ride, time_step,
                                                            9000)) for target_ride in
                                target_rides_calc if target_ride.finish_time >= time_step + 9000]
                target_rides = [target_ride for target_ride in target_rides if target_ride[1] >= 0]
                if target_rides:
                    # Find ride with lowest cost
                    cost = 999999999999
                    for ride in target_rides:
                        if ride[1] < cost:
                            all_vehicles_cost = min(
                                [arr_last_time(v, ride[0], time_step, 9000) for v in vehicles if
                                 vehicles
                                 != self])
                            if ride[1] <= all_vehicles_cost:
                                new_ride = ride[0]
                                cost = ride[1]

            if new_ride == '':
                # Find ride with lowest cost for starting distance and waiting time
                closest = [(ride_closest, arr_start_time_closest(self, ride_closest, time_step))
                           for ride_closest in target_rides_calc if ride_closest.vehicle == -1]
                cost = 999999999999
                for ride in closest:
                    if ride[1] < cost:
                        all_vehicles_cost = min([arr_start_time_closest(v, ride[0], time_step) for
                                                 v in vehicles if vehicles != self])
                        if ride[1] <= all_vehicles_cost:
                            new_ride = ride[0]
                            cost = ride[1]

            if new_ride == '':
                # Lowest cost for starting distance and waiting time - ignore other cars
                closest = [(ride_closest, arr_start_time_closest(self, ride_closest, time_step))
                           for ride_closest in target_rides_calc if ride_closest.vehicle == -1]
                cost = 999999999999
                for ride in closest:
                    if ride[1] < cost:
                        new_ride = ride[0]
                        cost = ride[1]

            if new_ride == '':
                self.status = 'Done'
                print("Error - ERROR - error")
                return

            #Calc end_time of ride
            end_time = max(time_step + calc_distance(self.pos_x, new_ride.pos_a, self.pos_y,
                                                     new_ride.pos_b), new_ride.start_time) + \
                                                        new_ride.distance
            if end_time > self.simul.steps:
                print('Ride ends after simulation')
                return

            #Start the process of assigning ride to vehicle and vice versa
            new_ride.assign_vehicle(self)
            self.ride = new_ride

            self.end_x = new_ride.pos_x
            self.end_y = new_ride.pos_y

            new_ride.set_start(max(time_step + calc_distance(self.pos_x, new_ride.pos_a, self.pos_y,
                                                             new_ride.pos_b), new_ride.start_time))
            self.end_of_ride = end_time

            if self.end_of_ride <= self.simul.steps - 1 and self.end_of_ride <= \
                    self.ride.finish_time:
                new_ride.points = new_ride.distance
                if new_ride.actual_start <= new_ride.start_time:
                    new_ride.points += self.simul.bonus
            self.ride_history.append(new_ride)

            print('Assigned vehicle %s to ride %s reaching start at time %s with ride bonus start '
                  'at %s and points of %s ride end %s' %
                  (self.vehicle_number, new_ride.ride_number, new_ride.actual_start,
                   new_ride.start_time, new_ride.points, self.end_of_ride))

    def timestep(self, time_step, rides, vehicles):
        """Advance vehicle by one timestep"""
        if self.end_of_ride == time_step:
            self.pos_x = self.end_x
            self.pos_y = self.end_y
            self.get_next_ride(time_step, rides, vehicles)

    def assign_rides(self, ride_list):
        """Assign rides"""
        self.assigned_rides = ride_list


class Ride:
    """Class to manage a ride"""
    def __init__(self, row, ride_number):
        self.pos_a = row[0]  # StartRow
        self.pos_b = row[1] # StartCol
        self.pos_x = row[2] # FinishRow
        self.pos_y = row[3]  # FinishCol
        self.start_time = row[4]  # StartTime
        self.finish_time = row[5]  # FinishTime
        self.ride_number = ride_number  #R idenumber
        self.vehicle = -1  #v ehicleassigned
        self.distance = calc_distance(self.pos_x, self.pos_a, self.pos_y, self.pos_b)
        self.actual_start = -1
        self.complete = 0
        self.points = 0

    def assign_vehicle(self, vehicle):
        """assign vehicle to ride"""
        self.vehicle = vehicle

    def set_start(self, actual_start):
        """set actual start of ride"""
        self.actual_start = actual_start


def cost_simple(vehicle, ride, time_step):

    """Calculates waiting time for vehicle to arrive on early start time"""

    time_left_ride = vehicle.end_of_ride - time_step
    time_next_ride = calc_distance(vehicle.pos_x, ride.pos_a, vehicle.pos_y, ride.pos_b)
    waiting_cost = ride.start_time - time_step - time_left_ride - time_next_ride

    return waiting_cost


def cost_simple_closest(vehicle, ride, time_step):

    """Calculates waiting time for vehicle to arrive on early start time"""

    time_left_curr_ride = vehicle.end_of_ride - time_step
    time_to_next_ride = calc_distance(vehicle.pos_x, ride.pos_a, vehicle.pos_y, ride.pos_b)
    closest_cost = time_left_curr_ride + time_to_next_ride

    return closest_cost


def cost_simple_last(vehicle, ride, time_step):

    """Calculates waiting time for vehicle to arrive on early start time"""

    time_left_curr_ride = vehicle.end_of_ride - time_step
    time_to_next_ride = calc_distance(vehicle.pos_x, ride.pos_a, vehicle.pos_y, ride.pos_b)
    waiting_cost = ride.finish_time - time_step - time_left_curr_ride - time_to_next_ride - \
                   ride.distance
    waiting_cost += 500

    return waiting_cost


def arr_start_time(vehicle, ride, time_step, threshold):

    """Find ride with intention to get bonus and minimise waiting time"""
    arr_waiting_time = cost_simple(vehicle, ride, time_step)
    if arr_waiting_time < 0 or arr_waiting_time > threshold:
        arr_waiting_time = -1

    return arr_waiting_time


def arr_last_time(vehicle, ride, time_step, threshold):
    """Helper functiom to calc arr last time"""
    arr_waiting_time = cost_simple_last(vehicle, ride, time_step)
    if arr_waiting_time < 0 or arr_waiting_time > threshold:
        arr_waiting_time = -1

    return arr_waiting_time


def arr_start_time_closest(vehicle, ride, time_step):
    """Find closest ride taking into account starting time"""
    arr_waiting_time = cost_simple_closest(vehicle, ride, time_step)

    return arr_waiting_time


def run_simul(filename):
    """Run an iteration of the simulation"""
    vehicles = []
    rides = []
    simul = ''
    #  filename = 'a_example.in'
    scenario = pd.read_csv(filename, sep=" ", header=None)

    for i, fin in scenario.iterrows():
        if i == 0:
            simul = Simulation(fin)
        else:
            rides.append(Ride(fin, i - 1))

    for i in range(simul.no_of_vehicles):
        vehicles.append(Vehicle(i, simul))

    time_step_main = 0
    while time_step_main < simul.steps:

        print(time_step_main)

        vehicle = [vehicle for vehicle in vehicles if vehicle.end_of_ride == time_step_main]

        ridelist = [x for x in rides if x.vehicle == -1]
        if not ridelist:
            time_step_main += 1
            continue

        if not vehicle:
            time_step_main = max(min([vehicle.end_of_ride for vehicle in vehicles]) - 1,
                                 time_step_main)
            time_step_main += 1
            continue

        # print('Executing time step: %s'  % tt)
        for vehicle in vehicles:
            vehicle.timestep(time_step_main, rides, vehicles)

        time_step_main += 1

    points = sum(a.points for a in rides)
    print('-------')
    print('Points: %s' % points)
    return points


def main():
    """Main function to loop trhough all the simulations and tally points"""
    total_points = 0

    for filename in os.listdir('.'):
        if filename.endswith(".in"):
            points = run_simul(filename)
            total_points += points

    print('-----')
    print('Final_Total_Points: %s' % total_points)

# filename = 'b_should_be_easy.in' --171 700
# filename = 'c_no_hurry.in'   --15 729 576
# filename = 'd_metropolis.in'   --7 613 202
# filename = 'e_high_bonus.in'  --21 340 239

# Total Score        -- 44 854 717
# run_simul(filename)
