import math
import numpy as np
from loguru import logger

def calculate_distance(ego, npc):
    return math.sqrt((math.pow(ego.location.x - npc.location.x,2) + math.pow(ego.location.y - npc.location.y,2)))

def compute_fitness_collision(ego, npc, ego_fault, execution_time):
    # TODO: Add min_dis although technically min_dis is 0 when collision
    ego_vel = ego.vehicle.get_velocity()
    npc_vel = npc.get_velocity()
    ego_pos = ego.vehicle.get_transform()
    npc_pos = npc.get_transform()
    ego_speed = np.array([ego_vel.x, ego_vel.y, ego_vel.z])
    npc_speed = np.array([npc_vel.x, npc_vel.y, npc_vel.z])
    min_dis = calculate_distance(ego_pos, npc_pos)
    fitness = np.linalg.norm(ego_speed - npc_speed) # get the module of the difference of speeds
    fitness = fitness + 100 # assuming mettc = 0 (maximum value to add to fitness function)
    fitness = fitness + (1/execution_time) * 100  if execution_time != 0.0 else 100# Add execution time
    fitness = fitness + (1/min_dis) * 100 if min_dis != 0.0 else 100
    if ego_fault:
        fitness = fitness + 100
    else: # TODO: Maybe if it's NPC fault it shuold penalise fitness function?
        fitness = fitness - 150
    return fitness

def compute_fitness(ego, npc, execution_time, mettc, min_dis):
    ego_vel = ego.vehicle.get_velocity()
    npc_vel = npc.vehicle.get_velocity()
    ego_speed = np.array([ego_vel.x, ego_vel.y, ego_vel.z])
    npc_speed = np.array([npc_vel.x, npc_vel.y, npc_vel.z])
    #min_dis = calculate_distance(ego.vehicle, npc.vehicle)
    fitness = np.linalg.norm(ego_speed - npc_speed) # get the module of the difference of speeds
    fitness = fitness + (1/mettc) * 100 if mettc != 0 else fitness + 90 # assuming mettc = 0 (maximum value to add to fitness function)
    fitness = fitness + (1/execution_time) * 100  if execution_time != 0.0 else 100# Add execution time
    fitness = fitness + (1/min_dis) * 100 if min_dis != 0 else 90

    #TODO if ego_speed like 0: ego_fault = False

    return fitness

def calculate_ettc(ego, npc):
    ego_pos = ego.vehicle.get_transform()
    npc_vel = npc.vehicle.get_velocity()
    npc_pos = npc.vehicle.get_transform()
    x2 = npc_pos.location.x
    x1 = ego_pos.location.x
    y2 = npc_pos.location.y
    y1 = ego_pos.location.y
    theta2 = npc_pos.rotation.yaw
    theta1 = ego_pos.rotation.yaw
    try:
        xplus = (y2-y1 + x1*math.tan(math.radians(theta1)) + x2*math.tan(math.radians(theta2)))/(math.tan(math.radians(theta1))-math.tan(math.radians(theta2)))
    except:
        xplus = 0
    try:
        yplus = (x2-x1 + y1*(1/math.tan(math.radians(theta1))) + y2*(1/math.tan(math.radians(theta2))))/(1/math.tan(math.radians(theta1))-1/math.tan(math.radians(theta2)))
    except:
        yplus = 0
    try:
        ettc = math.sqrt((yplus - y2) ** 2 + (xplus - x2) ** 2) / np.linalg.norm(np.array([npc_vel.x, npc_vel.y, npc_vel.z]))
    except:
        ettc = 100
    return ettc