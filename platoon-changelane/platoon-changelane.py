import os
import sys
import random

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
from plexe import Plexe, ACC, CACC, GEAR, RPM
from utils import start_sumo, running, add_platooning_vehicle, add_vehicle

# block time
T_PUSH = 30
# vehicle length
LENGTH = 5
# vehicle speed
SPEED = 80 / 3.6
# vehicle distance
DISTANCE = SPEED / 2 * 3.6
# creat platoon numbers
N_VEHICLES = 10
# creat vehicle time
updatestep = T_PUSH * 100 + 1000


# creat platoon of n vehicles to the simulation
def add_vehicles(plexe, n, number, position, step, updatestep, real_engine=False):
    # platoon leader number
    first_i = number

    # generate 20 vehicles in lane1 and lane2 in first time, and 10 vehicles later.
    if step < updatestep:
        n = n + number + N_VEHICLES
    else:
        n = n + number

    LEADER1 = "lane1.%d" % first_i
    LEADER2 = "lane2.%d" % first_i
    LEADER3 = "lane3.%d" % first_i
    LEADER4 = "lane4.%d" % first_i

    # creat platoon vehicles
    for i in range(first_i, n):

        vid1 = "lane1.%d" % i  # lane1
        add_platooning_vehicle(plexe, vid1, position - i * DISTANCE, 0, SPEED, DISTANCE, real_engine, vtype="lane1")

        vid2 = "lane2.%d" % i  # lane2
        add_platooning_vehicle(plexe, vid2, position - i * DISTANCE, 1, SPEED, DISTANCE, real_engine, vtype="lane2")

        if i <= N_VEHICLES or i >= (2 * N_VEHICLES) + 1:
            vid3 = "lane3.%d" % i  # lane3
            add_platooning_vehicle(plexe, vid3, position - i * DISTANCE, 2, SPEED, DISTANCE, real_engine, vtype="lane3")

            vid4 = "lane4.%d" % i  # lane4
            add_platooning_vehicle(plexe, vid4, position - i * DISTANCE, 3, SPEED, DISTANCE, real_engine, vtype="lane4")

        plexe.set_fixed_lane(vid1, 0, safe=False)
        plexe.set_fixed_lane(vid2, 1, safe=False)
        plexe.set_fixed_lane(vid3, 2, safe=False)
        plexe.set_fixed_lane(vid4, 3, safe=False)

        traci.vehicle.setSpeedMode(vid1, 0)
        traci.vehicle.setSpeedMode(vid2, 0)
        traci.vehicle.setSpeedMode(vid3, 0)
        traci.vehicle.setSpeedMode(vid4, 0)

        if i == number:
            plexe.set_active_controller(vid1, ACC)
            plexe.set_active_controller(vid2, ACC)
            plexe.set_active_controller(vid3, ACC)
            plexe.set_active_controller(vid4, ACC)
        else:
            plexe.set_active_controller(vid1, ACC)
            plexe.set_active_controller(vid2, ACC)
            plexe.set_active_controller(vid3, CACC)
            plexe.set_active_controller(vid4, CACC)
            plexe.enable_auto_feed(vid1, True, LEADER1, vid1)
            plexe.enable_auto_feed(vid2, True, LEADER2, vid2)
            plexe.enable_auto_feed(vid3, True, LEADER3, vid3)
            plexe.enable_auto_feed(vid4, True, LEADER4, vid4)
            plexe.add_member(LEADER1, vid1, i)
            plexe.add_member(LEADER2, vid2, i)
            plexe.add_member(LEADER3, vid3, i)
            plexe.add_member(LEADER4, vid4, i)


# Push-for-full queue algorithm for lane2
def pushforfull_lane2(T_PUSH, SPEED, N_VEHICLES, LENGTH, DISTANCE, exitlane_a, Remaining_LEADER):
    plexe = Plexe()
    N_pi = int((T_PUSH * SPEED) / ((N_VEHICLES * LENGTH / N_VEHICLES) + DISTANCE) - exitlane_a)

    platoon2_distance = DISTANCE / 2

    if Remaining_LEADER != 1:
        Remaining_LEADER += 1

    for i in range(Remaining_LEADER, Remaining_LEADER + N_pi):
        traci.vehicle.changeLane("lane2.%d" % i, 2, 1)
        traci.vehicle.setMinGap("lane2.%d" % i, 0)

        # leader ACC model, following CACC model
        if i == Remaining_LEADER:
            plexe.set_active_controller("lane2.%d" % i, ACC)
        else:
            plexe.set_active_controller("lane2.%d" % i, CACC)
            plexe.set_path_cacc_parameters("lane2.%d" % i, distance=platoon2_distance)
            plexe.enable_auto_feed("lane2.%d" % i, True, "lane2.%d" % Remaining_LEADER, "lane2.%d" % i)
            plexe.add_member("lane2.%d" % Remaining_LEADER, "lane2.%d" % i, i)

    return N_VEHICLES - N_pi


# Push-for-full queue algorithm for lane1
def pushforfull_lane1(T_PUSH, SPEED, N_VEHICLES, LENGTH, DISTANCE, exitlane_a, Remaining_LEADER):
    plexe = Plexe()
    N_pi = int((T_PUSH * SPEED) / ((N_VEHICLES * LENGTH / N_VEHICLES) + DISTANCE) - exitlane_a)

    if Remaining_LEADER != 1:
        Remaining_LEADER += 1

    for i in range(Remaining_LEADER, Remaining_LEADER + N_pi):
        traci.vehicle.changeLane("lane1.%d" % i, 1, 10)
        plexe.set_active_controller("lane1.%d" % i, ACC)

    return N_VEHICLES - N_pi


def main(demo_mode, real_engine):

    start_sumo("cfg/road.sumo.cfg", False)
    plexe = Plexe()
    traci.addStepListener(plexe)

    step = 0
    number_distance = 13000
    count = 1
    number = 1
    number_3 = 1
    number_2 = 1
    number_1 = 1
    LEADER_3_ALL = []
    LEADER_2_ALL = []
    LEADER_1_ALL = []
    LEADER_3_ALL_FINAL = []
    Remaining_LEADER_2 = 1
    Remaining_LEADER_1 = 1
    Remaining_vehicle_num_lane2 = 0
    Remaining_vehicle_num_lane1 = 0
    N_VEHICLES_num_lane2 = 0
    N_VEHICLES_num_lane1 = 0
    lane3_length = 0
    lane2_length = 0
    lane1_length = 0
    platoon1_distance = DISTANCE / 2
    v_number = 3
    v_position = 15020

    N_pi_lane1 = int((T_PUSH * SPEED) / ((N_VEHICLES * LENGTH / N_VEHICLES) + DISTANCE) - 2)
    N_pi_lane2 = int((T_PUSH * SPEED) / ((N_VEHICLES * LENGTH / N_VEHICLES) + DISTANCE) - 1)

    platoon1_lane_leader_num = 1
    platoon1_lane_final_num = Remaining_LEADER_1 + N_pi_lane1
    platoon2_lane_leader_num = 1
    platoon2_lane_final_num = Remaining_LEADER_1 + N_pi_lane2

    add_vehicle(plexe, "v1", 15000, 0, 0, "accident")
    plexe.set_fixed_lane("v1", 0, safe=False)
    traci.vehicle.setSpeed("v1", 0)
    add_vehicle(plexe, "v2", 15000, 1, 0, "accident")
    plexe.set_fixed_lane("v2", 1, safe=False)
    traci.vehicle.setSpeed("v2", 0)

    traci.gui.trackVehicle("View #0", "v2")
    traci.gui.setZoom("View #0", 15000)

    while running(demo_mode, step, 100000):

        N_pi_lane1 = int((T_PUSH * SPEED) / ((N_VEHICLES * LENGTH / N_VEHICLES) + DISTANCE) - 2)
        N_pi_lane2 = int((T_PUSH * SPEED) / ((N_VEHICLES * LENGTH / N_VEHICLES) + DISTANCE) - 1)

        # crate accident
        if step > 1000 and step % 28000 == 0:
            add_vehicle(plexe, "v.%d" % v_number, v_position, 0, 0, "accident")
            plexe.set_fixed_lane("v.%d" % v_number, 0, safe=False)
            traci.vehicle.setSpeed("v.%d" % v_number, 0)
            v_number += 1
            add_vehicle(plexe, "v.%d" % v_number, v_position, 1, 0, "accident")
            plexe.set_fixed_lane("v.%d" % v_number, 1, safe=False)
            traci.vehicle.setSpeed("v.%d" % v_number, 0)
            v_number += 1
            v_position += 20

        # when reaching 500000 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 500000:
            start_sumo("cfg/road.sumo.cfg", True)
            step = 0
            random.seed(1)

        traci.simulationStep()

        if step % updatestep == 1:
            add_vehicles(plexe, N_VEHICLES, number, number_distance, step, updatestep, real_engine)

            LEADER1 = "lane1." + str(number_1)
            LEADER2 = "lane2." + str(number_2)
            LEADER3 = "lane3." + str(number_3)

            # generate 20 vehicles in lane1 and lane2 in first time, and 10 vehicles later.
            if step < updatestep:
                number = number + N_VEHICLES + 10
                number_3 = number_3 + N_VEHICLES + 10
            else:
                number = number + N_VEHICLES

            number_1 = number_1 + N_pi_lane1
            number_2 = number_2 + N_pi_lane2
            LEADER_1_ALL.append(LEADER1)
            LEADER_2_ALL.append(LEADER2)

            if step < updatestep:
                number_3 = number_3
                FINAL3 = "lane3." + str(number_3 - 1 - 10)
            else:
                number_3 = number_3 + N_VEHICLES
                FINAL3 = "lane3." + str(number_3 - 1)

            LEADER_3_ALL.append(LEADER3)
            LEADER_3_ALL_FINAL.append(FINAL3)


        if step % 10 == 1:
            lane1_dis = traci.vehicle.getLanePosition("v1") - traci.vehicle.getLanePosition(LEADER_1_ALL[0])
            lane2_dis = traci.vehicle.getLanePosition("v2") - traci.vehicle.getLanePosition(LEADER_2_ALL[0])
            lane3_dis = traci.vehicle.getLanePosition("v2") - traci.vehicle.getLanePosition(LEADER_3_ALL[0])
            lane2_3_dis = traci.vehicle.getLanePosition("lane2.%d" % (platoon2_lane_leader_num)) - traci.vehicle.getLanePosition(LEADER_3_ALL_FINAL[0])

            if step == 1001:
                lane3_length = traci.vehicle.getLanePosition("lane3.1") - traci.vehicle.getLanePosition(LEADER_3_ALL_FINAL[0])
                lane2_length = (LENGTH + (DISTANCE / 2.4)) * (number_2 - 1)
                lane1_length = (LENGTH + (DISTANCE / 2)) * (number_1 - 1)


        if step % 100 == 1:
            if (lane2_dis < 100 and lane2_dis > 0 and lane2_3_dis < 1) or (lane1_dis > 0 and lane2_dis < 100 and lane2_dis > 0 and lane3_dis > lane2_length * 1.5):

                distance_number_lane2 = DISTANCE / 2

                Remaining_vehicle_num_lane2 = pushforfull_lane2(T_PUSH, SPEED, N_VEHICLES, LENGTH, DISTANCE, 1, Remaining_LEADER_2)
                Remaining_vehicle_num_lane1 = pushforfull_lane1(T_PUSH, SPEED, N_VEHICLES, LENGTH, DISTANCE, 2, Remaining_LEADER_1)

            if (lane1_dis < 100 and lane1_dis > 0 and lane2_dis < -(lane2_length)) and (lane3_dis > lane1_length or lane1_dis < 100 and lane1_dis > 0 and lane2_dis < -(
                    lane2_length) and lane2_3_dis < 1) or (lane1_dis < 100 and lane1_dis > 0 and lane2_dis < -(lane2_length) and lane3_dis < -(lane3_length)):

                distance_number_lane1 = DISTANCE / 2

                for i in range(platoon1_lane_leader_num, platoon1_lane_final_num):
                    traci.vehicle.changeLane("lane1.%d" % i, 2, 1)
                    traci.vehicle.setMinGap("lane1.%d" % i, 0)

                    # leader ACC model, following CACC model
                    if i == platoon1_lane_leader_num:
                        plexe.set_active_controller("lane1.%d" % i, ACC)
                    else:
                        plexe.set_active_controller("lane1.%d" % i, CACC)
                        plexe.set_path_cacc_parameters("lane1.%d" % i, distance=platoon1_distance)
                        plexe.enable_auto_feed("lane1.%d" % i, True, "lane1.%d" % platoon1_lane_leader_num, "lane1.%d" % i)
                        plexe.add_member("lane1.%d" % platoon1_lane_leader_num, "lane1.%d" % i, i)

            if lane2_dis < 0:

                distance_number_lane2 = distance_number_lane2 * 1.03

                if distance_number_lane2 < DISTANCE:
                    for i in range(platoon2_lane_leader_num, platoon2_lane_final_num):
                        plexe.set_path_cacc_parameters("lane2.%d" % i, distance=distance_number_lane2)
                else:
                    distance_number_lane2 = DISTANCE
                    for i in range(platoon2_lane_leader_num, platoon2_lane_final_num):
                        plexe.set_path_cacc_parameters("lane2.%d" % i, distance=DISTANCE)

            if lane1_dis < 0:

                distance_number_lane1 = distance_number_lane1 * 1.03

                if distance_number_lane1 < DISTANCE:
                    for i in range(platoon1_lane_leader_num, platoon1_lane_final_num):
                        plexe.set_path_cacc_parameters("lane1.%d" % i, distance=distance_number_lane1)
                else:
                    distance_number_lane1 = DISTANCE
                    for i in range(platoon1_lane_leader_num, platoon1_lane_final_num):
                        plexe.set_path_cacc_parameters("lane1.%d" % i, distance=DISTANCE)

            if lane3_dis < -(lane3_length):
                LEADER_3_ALL.pop(0)
                LEADER_3_ALL_FINAL.pop(0)

            lane1_platoon_length = traci.vehicle.getLanePosition("lane1.%d" % (platoon1_lane_final_num - 1)) - traci.vehicle.getLanePosition("v2")

            if lane1_platoon_length >= 0:
                platoon1_lane_leader_num = platoon1_lane_final_num
                platoon1_lane_final_num = platoon1_lane_final_num + N_pi_lane1

                platoon2_lane_leader_num = platoon2_lane_final_num
                platoon2_lane_final_num = platoon2_lane_final_num + N_pi_lane2

                N_VEHICLES_num_lane2 = N_VEHICLES_num_lane2 + N_VEHICLES - Remaining_vehicle_num_lane2
                N_VEHICLES_num_lane1 = N_VEHICLES_num_lane1 + N_VEHICLES - Remaining_vehicle_num_lane1

                Remaining_LEADER_2 = N_VEHICLES_num_lane2
                Remaining_LEADER_1 = N_VEHICLES_num_lane1

                count += 1

                LEADER_2_ALL.pop(0)
                LEADER_1_ALL.pop(0)

        step += 1

    traci.close()

if __name__ == "__main__":
    main(True, False)
