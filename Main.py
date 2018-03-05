
#● a – the row of the start intersection (0 ≤ a < R)
#● b – the column of the start intersection (0 ≤ b < C)
#● x – the row of the finish intersection (0 ≤ x < R)
#● y – the column of the finish intersection (0 ≤ y < C)
#● s – the earliest start(0 ≤ s < T)
#● f – the latest finish (0 ≤ f ≤ T) , (f ≥ s + |x − a| + |y − b|)
#○ note that f can be equal to T – this makes the latest finish equal to the end of the simulation


#● R – number of rows of the grid (1 ≤ R ≤ 10000)
#● C – number of columns of the grid (1 ≤ C ≤ 10000)
#● F – number of vehicles in the fleet (1 ≤ F ≤ 1000)
#● N – number of rides (1 ≤ N ≤ 10000)
#● B – per-ride bonus for starting the ride on time (1 ≤ B ≤ 10000)
#● T – number of steps in the simulation (1 ≤ T ≤ 10 )


import pandas as pd
import os


def CalcDistance(x,a,y,b):
    distance = abs(x-a) + abs(y-b)
    return distance


def calcVehToRideStart(vh,r):
    dist = CalcDistance(vh.x,vh.y,r.a,r.b)
    return dist


class simulation:
    def __init__(self,rows,cols,vehicles,rides,bonus,steps):
        self.rows = rows
        self.cols = cols
        self.vehicles = vehicles
        self.rides = rides
        self.bonus = bonus
        self.steps = steps


class vehicle:
    def __init__(self,n,simul):
        self.x = 0
        self.y = 0
        self.n = n
        self.simul = simul
        self.r = ''
        self.startofride = -1 #timestep of start of ride
        self.endofride = 0 #timestep of end of ride
        self.endx = 0
        self.endy = 0
        self.status = 'ready'
        self.rideHistory = []
        self.assignedRides =[]

    def getnextride(self,t,rides,vehicles):

        # Check for assignedRides
        if not self.assignedRides:

            # Get rides for which this vehice is a candidate
            newRide = ''
            targetRidesCalc = [rc for rc in rides if rc.vehicle == -1 and rc.f > t]
            targetRides = [(tr,arrStartTime(self, tr, t, 9000)) for tr in targetRidesCalc if tr.s > t]
            targetRides = [tr for tr in targetRides if tr[1] >= 0]

            if targetRides:
                # Find ride with lowest cost for start bonus
                cost = 999999999999
                for ri in targetRides:
                    if ri[1] < cost:
                        allvehiclescost = min([arrStartTime(v, ri[0], t, 9000) for v in vehicles if vehicles != self])
                        if ri[1] > allvehiclescost:
                            newRide = ri[0]
                            cost = ri[1]

            if newRide == '':
                # Find ride with lowest cost for arriving at finish just on time
                targetRides = [(tr, arrLastTime(self, tr, t, 9000)) for tr in targetRidesCalc if tr.f >= t + 9000]
                targetRides = [tr for tr in targetRides if tr[1] >= 0]
                if targetRides:
                    # Find ride with lowest cost
                    cost = 999999999999
                    for ri in targetRides:
                        if ri[1] < cost:
                            allvehiclescost = min(
                                [arrLastTime(v, ri[0], t, 9000) for v in vehicles if vehicles != self])
                            if ri[1] <= allvehiclescost:
                                newRide = ri[0]
                                cost = ri[1]

            if newRide == '':
                # Find ride with lowest cost for starting distance and waiting time
                closest = [(rcl,arrStartTimeClosest(self,rcl,t)) for rcl in targetRidesCalc if rcl.vehicle == -1]
                cost = 999999999999
                for ri in closest:
                    if ri[1] < cost:
                        allvehiclescost = min([arrStartTimeClosest(v, ri[0], t) for v in vehicles if vehicles != self])
                        if ri[1] <= allvehiclescost:
                            newRide = ri[0]
                            cost = ri[1]

            if newRide == '':
                # Find ride with lowest cost for starting distance and waiting time - ignore other cars
                closest = [(rcl,arrStartTimeClosest(self,rcl,t)) for rcl in targetRidesCalc if rcl.vehicle == -1]
                cost = 999999999999
                for ri in closest:
                    if ri[1] < cost:
                        newRide = ri[0]
                        cost = ri[1]

            if newRide == '':
                self.status = 'Done'
                print ("Error - ERROR - error")
                return

            #Calc endtime of ride
            endtime = max(t + CalcDistance(self.x,newRide.a,self.y,newRide.b),newRide.s) + newRide.distance
            if endtime > self.simul.steps:
                print('Ride ends after simul')
                return

            newRide.assignVehicle(self)
            self.r = newRide

            self.endx = newRide.x
            self.endy = newRide.y

            newRide.setStart(max(t + CalcDistance(self.x,newRide.a,self.y,newRide.b),newRide.s))

            self.endofride = endtime

            if self.endofride <= self.simul.steps - 1 and self.endofride <= self.r.f:
                newRide.points = newRide.distance
                if newRide.start <= newRide.s:
                    newRide.points += self.simul.bonus
            self.rideHistory.append(newRide)
            print('Assigned vehicle %s to ride %s reaching start at time %s with ride bonus start at %s and points of %s ride end %s' %
                  (self.n,newRide.r,newRide.start,newRide.s,newRide.points,self.endofride))

    def timestep(self,t,rides,vehicles):
        if self.endofride == t:
            self.x = self.endx
            self.y = self.endy
            self.getnextride(t,rides,vehicles)

    def assignRides(self,rideList):
        self.assignedRides = rideList


class ride:
    def __init__(self,a,b,x,y,s,f,r):
        self.a = a  #StartRow
        self.b = b  #StartCol
        self.x = x  #FinishRow
        self.y = y  #FinishCol
        self.s = s  #StartTime
        self.f = f  #FinishTime
        self.r = r  #Ridenumber
        self.vehicle = -1 #vehicleassigned
        self.distance = CalcDistance(x,a,y,b)
        self.start = -1
        self.complete = 0
        self.points = 0

    def assignVehicle(self,vehicle):
        self.vehicle = vehicle.n

    def setStart(self,strt):
        self.start = strt


def CostSimple(vehicle,ride,t):

    # Calculates waiting time for vehicle to arrive on early start time

    timeLeftCurrRide = vehicle.endofride - t
    timeToNextRide = CalcDistance(vehicle.x,ride.a,vehicle.y,ride.b)
    WaitingCost = ride.s - t - timeLeftCurrRide - timeToNextRide
    points = 0

    return WaitingCost


def CostSimpleClosest(vehicle,ride,t):

    # Calculates waiting time for vehicle to arrive on early start time

    timeLeftCurrRide = vehicle.endofride - t
    timeToNextRide = CalcDistance(vehicle.x,ride.a,vehicle.y,ride.b)
    ClosestCost = timeLeftCurrRide + timeToNextRide

    return ClosestCost


def CostSimpleLast(vehicle,ride,t):

    # Calculates waiting time for vehicle to arrive on early start time

    timeLeftCurrRide = vehicle.endofride - t
    timeToNextRide = CalcDistance(vehicle.x,ride.a,vehicle.y,ride.b)
    WaitingCost = ride.f - t - timeLeftCurrRide - timeToNextRide - ride.distance
    WaitingCost += 500

    return WaitingCost


def arrStartTime(vehicle,ride,t,threshold):
    # Find ride with intention to get bonus and minimise waiting time
    arrWaitingTime = CostSimple(vehicle,ride,t)
    if arrWaitingTime <0 or arrWaitingTime > threshold:
        arrWaitingTime = -1

    return arrWaitingTime


def arrLastTime(vehicle,ride,t,threshold):

    arrWaitingTime = CostSimpleLast(vehicle, ride,t)
    if arrWaitingTime < 0 or arrWaitingTime > threshold:
        arrWaitingTime = -1

    return arrWaitingTime


def arrStartTimeClosest(vehicle,ride,t):
    # Find closest ride taking into account starting time
    arrWaitingTime = CostSimpleClosest(vehicle,ride,t)

    return arrWaitingTime


def runSimul(filename):
    vehicles = []
    rides = []
    simul = ''
    #  filename = 'a_example.in'
    scenario = pd.read_csv(filename, sep=" ", header=None)

    for i, fin in scenario.iterrows():
        if i == 0:
            simul = simulation(fin[0], fin[1], fin[2], fin[3], fin[4], fin[5])
        else:
            rides.append(ride(fin[0], fin[1], fin[2], fin[3], fin[4], fin[5], i - 1))

    for i in range(simul.vehicles):
        vehicles.append(vehicle(i, simul))

    tt = 0
    while tt < simul.steps:

        print(tt)

        vh = [vh for vh in vehicles if vh.endofride == tt]

        ridelist = [x for x in rides if x.vehicle == -1]
        if not ridelist:
            tt += 1
            continue

        if not vh:
            tt = max(min([vh.endofride for vh in vehicles]) - 1, tt)
            tt += 1
            continue

        rideCandidates = []

        # print('Executing time step: %s'  % tt)
        for vh in vehicles:
            vh.timestep(tt, rides,vehicles)
            #rideCandidates = [rc for rc in rideCandidates if rc[0].vehicle == -1]

        tt += 1

    points = sum(a.points for a in rides)
    print('-------')
    print('Points: %s' % points)
    return(points)


def main():
    totalpoints = 0
    for filename in os.listdir('.'):
        if filename.endswith(".in"):
            points = runSimul(filename)
            totalpoints += points

    print('-----')
    print('FinalTotalPoints: %s' % totalpoints)

# filename = 'b_should_be_easy.in' --171 700
# filename = 'c_no_hurry.in'   --15 729 576
# filename = 'd_metropolis.in'   --7 613 202
# filename = 'e_high_bonus.in'  --21 340 239

# Total Score        -- 44 854 717
# runSimul(filename)