from collections import deque
import json
import os
import string
import sys
import math
from typing import Dict , List, Optional
from Read_input import Input
from Object import Factory, Node, Vehicle, VehicleInfo, OrderItem, Destination
import copy
import numpy as np
from constant import APPROACHING_DOCK_TIME , Delta , debugPeriod , SLACK_TIME_THRESHOLD , modle4 , modle6 , modle8
from local_search import * 

def inter_couple_exchange(vehicleid_to_plan: Dict[str , List[Node]], id_to_vehicle: Dict[str , Vehicle] , route_map: Dict[tuple , tuple]):
    is_improved = False
    cp_vehicle_id_to_planned_route : Dict[str , List[Node]] = copy.deepcopy(vehicleid_to_plan)
    dis_order_super_node : Dict[int, Dict[str , Node]] = get_OngoingSuperNode(vehicleid_to_plan , id_to_vehicle)
    ls_node_pair_num = len(dis_order_super_node)
    if ls_node_pair_num == 0:
        return False
    vehicleID = ""
    pdg_Map : Dict[str , List[Node]] = {}
    
    for idx, pdg in dis_order_super_node.items():
        pickup_node = None
        delivery_node = None
        node_list: List[Node] = []
        pos_i = 0
        pos_j = 0
        d_num = len(pdg) / 2
        index = 0

        if pdg:
            for v_and_pos_str, node in pdg.items():
                if index % 2 == 0:
                    vehicle_id = v_and_pos_str.split(",")[0]
                    pos_i = int(v_and_pos_str.split(",")[1])
                    pickup_node = node
                    node_list.insert(0, pickup_node)
                    index += 1
                else:
                    pos_j = int(v_and_pos_str.split(",")[1])
                    delivery_node = node
                    node_list.append(delivery_node)
                    index += 1
                    pos_j = pos_j - d_num + 1

            k : str = f"{vehicle_id},{pos_i}+{pos_j}"
            pdg_Map[k] = node_list
    if len(pdg_Map) < 2:
        return False
    
    vehicle = id_to_vehicle.get(vehicleID)
    route_node_list = vehicleid_to_plan.get(vehicleID)
    cost0 = cost_of_a_route(route_node_list, vehicle , id_to_vehicle , route_map , vehicleid_to_plan)
    origin_cost = cost0
    min_cost = cost0

    min_cost_pdg1_key_str : str = None
    min_cost_pdg2_key_str :str = None
    min_cost_pdg1 : List[Node]= None
    min_cost_pdg2 : List[Node]= None

    idx_i = 0
    idx_j = 0
    for before_key , before_DPG in pdg_Map.items():
        before_vehicle = id_to_vehicle.get(before_key.split(",")[0])
        before_posI = int(before_key.split(",")[1].split("+")[0])
        before_posJ = int(before_key.split(",")[1].split("+")[1])
        d1num = len(before_DPG) // 2
        
        idx_j = 0
        for next_key , next_DPG in pdg_Map.items():
            if idx_i >= idx_j:
                idx_j += 1
                continue
            
            next_vehicle = id_to_vehicle.get(next_key.split(",")[0])
            next_posI = int(next_key.split(",")[1].split("+")[0])
            next_posJ = int(next_key.split(",")[1].split("+")[1])
            d2num = len(next_DPG) // 2
            if before_vehicle is next_vehicle:
                continue
            
            route_node_list1 = vehicleid_to_plan.get(before_vehicle.id , [])
            route_node_list2 = vehicleid_to_plan.get(next_vehicle.id , [])
            
            temp1 = route_node_list1[before_posI: before_posI + d1num]
            temp11 = route_node_list1[before_posJ: before_posJ + d1num]
            temp2 = route_node_list2[next_posI: next_posI + d2num]
            temp22 = route_node_list2[next_posJ: next_posJ + d2num]
            
            del route_node_list1[before_posI: before_posI + d1num]
            del route_node_list2[next_posI: next_posI + d2num]
            route_node_list1[before_posI:before_posI] = temp2
            route_node_list2[next_posI:next_posI] = temp1
            
            real_before_post_j = before_posJ + (d2num - d1num)
            real_next_post_j = next_posJ + (d1num - d2num)
            
            del route_node_list1[real_before_post_j: real_before_post_j + d1num]
            if len(route_node_list2) < real_next_post_j + d2num:
                print(222 , file= sys.stderr)
            del route_node_list2[real_next_post_j: real_next_post_j + d2num]
            
            route_node_list1[real_before_post_j:real_before_post_j] = temp22
            route_node_list2[real_next_post_j:real_next_post_j] = temp11
            
            carry_items = next_vehicle.carrying_items if next_vehicle.des else []
            cost1 = float('inf') if not isFeasible(route_node_list2 , carry_items , next_vehicle.board_capacity) else cost_of_a_route(route_node_list1, before_vehicle , id_to_vehicle , route_map , vehicleid_to_plan)
            
            if cost1 < min_cost:
                min_cost = cost1
                is_improved = True
                min_cost_pdg1_key_str = before_key
                min_cost_pdg2_key_str = next_key
                min_cost_pdg1 = before_DPG[:]
                min_cost_pdg2 = next_DPG[:]
            
            del route_node_list1[real_before_post_j: real_before_post_j + d2num]
            del route_node_list2[real_next_post_j: real_next_post_j + d1num]
            route_node_list1[real_before_post_j:real_before_post_j] = temp11
            route_node_list2[real_next_post_j:real_next_post_j] = temp22
            
            del route_node_list1[before_posI: before_posI + d2num]
            del route_node_list2[next_posI: next_posI + d1num]
            route_node_list1[before_posI:before_posI] = temp1
            route_node_list2[next_posI:next_posI] = temp2
        idx_j += 1
    idx_i += 1
    
    if is_improved:
        before_DPG = min_cost_pdg1
        before_key = min_cost_pdg1_key_str
        before_vehicle = id_to_vehicle.get(before_key.split(",")[0])
        before_posI = int(before_key.split(",")[1].split("+")[0])
        before_posJ = int(before_key.split(",")[1].split("+")[1])
        d1num = len(before_DPG) // 2
        
        next_key = min_cost_pdg2_key_str
        next_DPG = min_cost_pdg2
        next_vehicle = id_to_vehicle.get(next_key.split(",")[0])
        next_posI = int(next_key.split(",")[1].split("+")[0])
        next_posJ = int(next_key.split(",")[1].split("+")[1])
        d2num = len(next_DPG) // 2
        
        route_node_list1 = vehicleid_to_plan.get(before_vehicle.id , [])
        route_node_list2 = vehicleid_to_plan.get(next_vehicle.id , [])
        
        temp1 = route_node_list1[before_posI: before_posI + d1num]
        temp11 = route_node_list1[before_posJ: before_posJ + d1num]
        temp2 = route_node_list2[next_posI: next_posI + d2num]
        temp22 = route_node_list2[next_posJ: next_posJ + d2num]
        
        del route_node_list1[before_posI: before_posI + d1num]
        del route_node_list2[next_posI: next_posI + d2num]
        route_node_list1[before_posI:before_posI] = temp2
        route_node_list2[next_posI:next_posI] = temp1
        
        real_before_post_j = before_posJ + (d2num - d1num)
        real_next_post_j = next_posJ + (d1num - d2num)
        
        del route_node_list1[real_before_post_j: real_before_post_j + d1num]
        del route_node_list2[real_next_post_j: real_next_post_j + d2num]
        route_node_list1[real_before_post_j:real_before_post_j] = temp22
        route_node_list2[real_next_post_j:real_next_post_j] = temp11

    return is_improved


def block_exchange(vehicleid_to_plan: Dict[str , List[Node]], id_to_vehicle: Dict[str , Vehicle] , route_map: Dict[tuple , tuple]):
    is_improved = False
    dis_order_super_node : Dict[int, Dict[str , Node]] = get_OngoingSuperNode(vehicleid_to_plan , id_to_vehicle)
    ls_node_pair_num = len(dis_order_super_node)
    if ls_node_pair_num == 0:
        return False
    
    vehicleID = None
    block_map : Dict[str , List[Node]] = {}
    for idx , pdg in dis_order_super_node.items():
        pickup_node : Node = None
        delivery_node : Node = None
        node_list :List[Node] = []
        posI :int =0 ; posJ : int= 0
        dNum : int= len(pdg) // 2
        index :int= 0
        if pdg:
            for v_and_pos_str, node in pdg.items():
                if index % 2 == 0:
                    vehicleID = v_and_pos_str.split(",")[0]
                    posI = int(v_and_pos_str.split(",")[1])
                    pickup_node = node
                    node_list.insert(0, pickup_node)
                    index += 1
                else:
                    posJ = int(v_and_pos_str.split(",")[1])
                    delivery_node = node
                    node_list.append(delivery_node)
                    index += 1
                    posJ = posJ - dNum + 1
                    
            vehicle_node_route : List[Node] = vehicleid_to_plan.get(vehicleID , [])
            for i in range(posI + dNum , posJ):
                node_list.insert(i - posI , vehicle_node_route.get(i , []))
            k : str = f"{vehicleID},{posI}+{posJ + dNum - 1}"    
            block_map[k] = node_list
    if len(block_map)  <2:
        return False
    
    origin_cost = total_cost(id_to_vehicle , route_map , vehicleid_to_plan)
    min_cost = origin_cost
    min_cost_block1_key_str : str = None
    min_cost_block2_key_str :str = None
    min_cost_block1 : List[Node] = None
    min_cost_block2 : List[Node] = None
    idxI = 0
    idxJ = 0
    
    for before_key , before_block in block_map.items():
        before_vehicle = id_to_vehicle.get(before_key.split(",")[0])
        before_posI = int(before_key.split(",")[1].split("+")[0])
        before_posJ = int(before_key.split(",")[1].split("+")[1])
        block1_len = len(before_block)
        
        idxJ = 0
        for next_key , next_block in block_map.items():
            if idxI >= idxJ: 
                idxJ +=1
                continue
            
            next_vehicle = id_to_vehicle.get(next_key.split(",")[0])
            next_posI = int(next_key.split(",")[1].split("+")[0])
            next_posJ = int(next_key.split(",")[1].split("+")[1])
            block2_len = len(next_block) 
            
            route_node_list1 = vehicleid_to_plan.get(before_vehicle.id , [])
            route_node_list2 = vehicleid_to_plan.get(next_vehicle.id , [])
            
            if before_vehicle is not next_vehicle: 
                temp1 : List[Node]= route_node_list1[before_posI , before_posI + block1_len]
                temp2 : List[Node]= route_node_list2[next_posI , next_posI + block2_len]
                
                del route_node_list1[before_posI , before_posI + block1_len]
                del route_node_list2[next_posI , next_posI + block2_len]
                route_node_list1[before_posI: before_posI] = temp2
                route_node_list2[next_posI: next_posI] = temp1
                
                carrying_items : List[OrderItem] = []
                cost1 = 0.0
                if next_vehicle.des is not None: 
                    carrying_items = next_vehicle.carrying_items
                if not isFeasible(route_node_list2, carrying_items , next_vehicle.board_capacity):
                    cost1 = math.inf
                else:
                    cost1 = cost_of_a_route(route_node_list1 , before_vehicle , id_to_vehicle , route_map , vehicleid_to_plan)
                    
                if cost1 < min_cost:
                    is_improved = True
                    min_cost_block1_key_str = before_key
                    min_cost_block2_key_str = next_key
                    min_cost_block1  = before_block[:]
                    min_cost_block2 = next_block[:]

                del route_node_list1[before_posI : before_posI + block2_len]
                del route_node_list2[next_posI : next_posI + block1_len]
                route_node_list1 [before_posI: before_posI] = temp1
                route_node_list2 [next_posI: next_posI] = temp2
            else:
                if before_posJ < next_posI or next_posJ < before_posI:
                    if before_post_j < next_post_i or next_post_j < before_post_i:
                        if next_post_j < before_post_i:
                            before_post_i, next_post_i = next_post_i, before_post_i
                            before_post_j, next_post_j = next_post_j, before_post_j
                            block1_len, block2_len = block2_len, block1_len

                        temp1 = route_node_list1[before_post_i: before_post_i + block1_len]
                        temp2 = route_node_list1[next_post_i: next_post_i + block2_len]

                        del route_node_list1[next_post_i: next_post_i + block2_len]
                        del route_node_list1[before_post_i: before_post_i + block1_len]

                        route_node_list1[before_post_i:before_post_i] = temp2
                        real_next_post_i = next_post_i + (block2_len - block1_len)
                        route_node_list1[real_next_post_i:real_next_post_i] = temp1

                        cost1 = cost_of_a_route(route_node_list1, before_vehicle , id_to_vehicle , route_map , vehicleid_to_plan)
                        if cost1 < min_cost:
                            is_improved = True
                            min_cost_block1_key_str = before_key
                            min_cost_block2_key_str = next_key
                            min_cost_block1 = before_block[:]
                            min_cost_block2 = next_block[:]

                        del route_node_list1[real_next_post_i: real_next_post_i + block1_len]
                        del route_node_list1[before_post_i: before_post_i + block2_len]
                        route_node_list1[before_post_i:before_post_i] = temp1
                        route_node_list1[next_post_i:next_post_i] = temp2
                        
            idxJ +=1
        idxI +=1
    
    if is_improved:
        before_key = min_cost_block1_key_str
        before_vid, before_positions = before_key.split(",")
        before_post_i, before_post_j = map(int, before_positions.split("+"))
        before_block = min_cost_block1[:]
        block1_len = len(before_block)

        next_key = min_cost_block2_key_str
        next_vid, next_positions = next_key.split(",")
        next_post_i, next_post_j = map(int, next_positions.split("+"))
        next_dpg = min_cost_block2[:]
        block2_len = len(next_dpg)

        route_node_list1 = vehicleid_to_plan.get(before_vid, [])
        if before_vid != next_vid:
            route_node_list2 = vehicleid_to_plan.get(next_vid, [])
            temp1 = route_node_list1[before_post_i:before_post_j + 1]
            temp2 = route_node_list2[next_post_i:next_post_j + 1]

            del route_node_list1[before_post_i:before_post_i + block1_len]
            del route_node_list2[next_post_i:next_post_i + block2_len]
            route_node_list1[before_post_i:before_post_i] = temp2
            route_node_list2[next_post_i:next_post_i] = temp1
        else:
            temp1 = route_node_list1[before_post_i:before_post_i + block1_len]
            temp2 = route_node_list1[next_post_i:next_post_i + block2_len]

            del route_node_list1[next_post_i:next_post_i + block2_len]
            del route_node_list1[before_post_i:before_post_i + block1_len]

            route_node_list1[before_post_i:before_post_i] = temp2
            real_next_post_i = next_post_i + (block2_len - block1_len)
            route_node_list1[real_next_post_i:real_next_post_i] = temp1

    return is_improved


def block_relocate():
    pass

def multi_pd_group_relocate():
    pass

def improve_ci_path_by_2_opt():
    pass
