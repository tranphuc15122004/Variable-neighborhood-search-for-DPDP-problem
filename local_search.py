import json
import os
import string
import sys
from typing import Dict
from Read_input import Input
from Object import Factory, Node, Vehicle, VehicleInfo, OrderItem, Destination
import copy
import numpy as np
from constant import APPROACHING_DOCK_TIME , Delta , debugPeriod , SLACK_TIME_THRESHOLD , modle4 , modle6 , modle8


def create_Pickup_Delivery_nodes(tmp_itemList: list[OrderItem] , id_to_factory: Dict[str , Factory]) -> list[Node]:
    res: list[Node] = []
    if tmp_itemList:
        pickup_address =tmp_itemList[0].pickup_factory_id
        delivery_address =  tmp_itemList[0].delivery_factory_id
        for order_item in tmp_itemList:
            if order_item.pickup_factory_id != pickup_address:
                print("The pickup factory of these items is not the same")
                pickup_address = ""
                break

        for order_item in tmp_itemList:
            if order_item.delivery_factory_id != delivery_address:
                print("The delivery factory of these items is not the same")
                delivery_address = ""
                break

    if len(pickup_address) ==0 or len(delivery_address) == 0:
        return None
    
    pickup_factory = id_to_factory[pickup_address]
    delivery_factory = id_to_factory[delivery_address]

    pickup_item_list = []
    for item in tmp_itemList:
        pickup_item_list.append(item)
    pickup_node = Node(factory_id= pickup_factory.factory_id , delivery_item_list=[] , pickup_item_list= pickup_item_list , lng= pickup_factory.lng , lat= pickup_factory.lat)

    delivery_item_list = []
    for item in reversed(tmp_itemList):
        delivery_item_list.append(item)
    delivery_node = Node(delivery_factory.factory_id,delivery_item_list,[],delivery_factory.lng,delivery_factory.lat)

    res.append(pickup_node)
    res.append(delivery_node)
    return res


def isFeasible (tempRouteNodeList: list[Node] , carrying_Items: list [OrderItem] , capacity: float) -> bool:
    unload_item_list = [OrderItem]
    
    # Copy carrying_items in reverse order
    if carrying_Items and len(carrying_Items) > 0:
        for i in range(len(carrying_Items) - 1, -1, -1):
            unload_item_list.append(carrying_Items[i])
    
    for node in tempRouteNodeList:
        delivery_items = node.delivery_item_list
        pickup_items = node.pickup_item_list
        
        # Process delivery items
        if delivery_items and len(delivery_items) > 0:
            for order_item in delivery_items:
                if not unload_item_list or not unload_item_list[0] or unload_item_list[0].id  != order_item.id:
                    print("Violate FILO", file=sys.stderr)
                    return False
                unload_item_list.pop(0)
        
        # Process pickup items
        if pickup_items and len(pickup_items) > 0:
            for order_item in pickup_items:
                unload_item_list.insert(0, order_item)
    
    if unload_item_list:
        print("Violate FILO", file=sys.stderr)
        return False

    left_capacity = capacity
    if carrying_Items:
        for order_item in carrying_Items:
            left_capacity -= order_item.demand
    
    for node in tempRouteNodeList:
        delivery_items = node.delivery_item_list
        pickup_items = node.pickup_item_list

        # Update capacity for delivery items
        if delivery_items:
            for order_item in delivery_items:
                left_capacity += order_item.demand
                if left_capacity > capacity:
                    return False
        
        # Update capacity for pickup items
        if pickup_items:
            for order_item in pickup_items:
                left_capacity -= order_item.demand
                if left_capacity < 0:
                    return False
    
    return (not unload_item_list)

def cost(tempRouteNodeList: list[Node] ,  vehicle: Vehicle , id_to_vehicle: Dict[str , Vehicle] , route_map : Dict[tuple , tuple] , vehicleid_to_plan: Dict[str, list[Node]]) -> float:
    curFactoryID = vehicle.cur_factory_id
    driving_distance = 0
    overTime_sum = 0
    F = 0.0
    capacity = vehicle.board_capacity
    
    carrying_Items = []
    if vehicle.des is not None:
        carrying_Items = vehicle.carrying_items
    if len(tempRouteNodeList) > 0 and isFeasible(tempRouteNodeList , carrying_Items , capacity) == False:
        return float('inf')
    docktable: Dict[str, list] = {}
    n = 0
    vehicle_Num = len(id_to_vehicle)
    index = 0
    curr_Node = [0] * vehicle_Num
    curr_Time = [0] * vehicle_Num
    leaveLastNodeTime = [0] * vehicle_Num
    nNode = [0] * vehicle_Num
    
    for vehicleID , other_vehicle in id_to_vehicle.items():
        distance = 0
        time = 0
        
        if other_vehicle.cur_factory_id and len(other_vehicle.cur_factory_id) > 0:
            if other_vehicle.leave_time_at_current_factory > other_vehicle.gps_update_time:
                tw = [other_vehicle.arrive_time_at_current_factory, other_vehicle.leave_time_at_current_factory]
                tw_list = docktable.get(other_vehicle.cur_factory_id, [])
                tw_list.append(tw)
                docktable[other_vehicle.cur_factory_id] = tw_list
            
            leaveLastNodeTime[index] = other_vehicle.leave_time_at_current_factory
        else:
            leaveLastNodeTime[index] = other_vehicle.gps_update_time

        if vehicleID == vehicle.id:
            if not tempRouteNodeList or len(tempRouteNodeList) == 0:
                curr_Node[index] = float('inf')
                curr_Time[index] = float('inf')
                nNode[index] = 0
            else:
                curr_Node[index] = 0
                nNode[index] = len(tempRouteNodeList)

                if vehicle.des is None:
                    if vehicle.cur_factory_id == "":
                        print("Error: cur factory have no value", file=sys.stderr)

                    if len(tempRouteNodeList) == 0:
                        print("Error: tempRouteNodeList have no length", file=sys.stderr)

                    if vehicle.cur_factory_id == tempRouteNodeList[0].id:
                        curr_Node[index] = vehicle.leave_time_at_current_factory
                    else:
                        dis_and_time = route_map[(vehicle.cur_factory_id , tempRouteNodeList[0].id)]
                    
                        if dis_and_time:
                            distance = float(dis_and_time[0])
                            time = int(dis_and_time[1])
                            curr_Time[index] = vehicle.leave_time_at_current_factory + time
                            driving_distance += distance
                else:
                    if curFactoryID and len(curFactoryID) > 0:
                        if curFactoryID != tempRouteNodeList[0].id:
                            curr_Time[index] = vehicle.leave_time_at_current_factory
                            dis_and_time= route_map[(curFactoryID ,tempRouteNodeList[0].id)]
                            
                            if dis_and_time: 
                                distance = float(dis_and_time[0])
                                time = int(dis_and_time[1])
                                driving_distance += distance
                                curr_Time[index] += time
                        else:
                            curr_Time[index] = vehicle.leave_time_at_current_factory
                    else:
                        curr_Time[index] = vehicle.des.arrive_time

                n += 1
        elif vehicleid_to_plan[vehicleID] and len(vehicleid_to_plan[vehicleID]) > 0:
            curr_Node[index] = 0
            nNode[index] = len(vehicleid_to_plan[vehicleID])

            if other_vehicle.des is None:
                if other_vehicle.cur_factory_id == vehicleid_to_plan[vehicleID][0].id:
                    curr_Node[index] = other_vehicle.leave_time_at_current_factory
                else:
                    dis_and_time = route_map[(other_vehicle.cur_factory_id , vehicleid_to_plan[vehicleID][0].id)]
                    if not dis_and_time:
                        print("Error: no distance", file=sys.stderr)
                    else:
                        distance = float(dis_and_time[0])
                        time = int(dis_and_time[1])
                        curr_Time[index] = other_vehicle.leave_time_at_current_factory + time
                        driving_distance += distance
            else:
                if other_vehicle.cur_factory_id and len(other_vehicle.cur_factory_id) > 0:
                    if other_vehicle.cur_factory_id != vehicleid_to_plan[vehicleID][0].id:
                        curr_Time[index] = other_vehicle.leave_time_at_current_factory
                        dis_and_time = route_map[(other_vehicle.cur_factory_id , vehicleid_to_plan[vehicleID][0].id)]

                        if dis_and_time:
                            distance = float(dis_and_time[0])
                            time = int(dis_and_time[1])
                            curr_Time[index] += time
                            driving_distance += distance
                    else:
                        curr_Time[index] = other_vehicle.leave_time_at_current_factory
                else:
                    curr_Time[index] = other_vehicle.des.arrive_time

            n += 1
        else:
            curr_Node[index] = float('inf')
            curr_Time[index] = float('inf')
            nNode[index] = 0

        index += 1
        
    #flag = False

    while n > 0:
        minT = sys.maxsize
        minT2VehicleIndex = 0
        tTrue = minT
        idx = 0

        for i in range(vehicle_Num):
            if curr_Time[i] < minT:
                minT = curr_Time[i]
                minT2VehicleIndex = i

        minT2VehicleId = f"V_{minT2VehicleIndex + 1}"

        if minT2VehicleId == vehicle.id:
            minTNodeList = tempRouteNodeList
        else:
            minTNodeList = vehicleid_to_plan[minT2VehicleId]

        minTNode = minTNodeList[curr_Node[minT2VehicleIndex]]

        if minTNode.delivery_item_list:
            beforeOrderId = ""
            for orderItem in minTNode.delivery_item_list:
                nextOrderId = orderItem.order_id
                if beforeOrderId != nextOrderId:
                    commitCompleteTime = orderItem.committed_completion_time
                    overTime_sum += max(0, curr_Time[minT2VehicleIndex] - commitCompleteTime)
                beforeOrderId = nextOrderId

        usedEndTime = []

        timeSlots = docktable.get(minTNode.id , [])
        newTimeSlots = []
        for timeSlot in timeSlots:
            if timeSlot[1] <= minT:
                continue  # Bỏ qua phần tử
            elif timeSlot[0] <= minT < timeSlot[1]:
                usedEndTime.append(timeSlot[1])
            else:
                print("------------ timeslot.start > minT --------------")
            newTimeSlots.append(timeSlot)
        timeSlots = newTimeSlots


        if len(usedEndTime) < 6:
            tTrue = minT
        else:
            #flag = True
            idx = len(usedEndTime) - 6
            usedEndTime.sort()
            tTrue = usedEndTime[idx]

        #isSameAddress = False
        serviceTime = minTNodeList[curr_Node[minT2VehicleIndex]].service_time
        curFactoryId = minTNodeList[curr_Node[minT2VehicleIndex]].id
        curr_Node[minT2VehicleIndex] += 1

        while (curr_Node[minT2VehicleIndex] < nNode[minT2VehicleIndex] and 
            curFactoryId == minTNodeList[curr_Node[minT2VehicleIndex]].id):
            nextNode = minTNodeList[curr_Node[minT2VehicleIndex]]
            if nextNode.delivery_item_list:
                beforeOrderId = ""
                for orderItem in nextNode.delivery_item_list:
                    nextOrderId = orderItem.order_id
                    if beforeOrderId != nextOrderId:
                        commitCompleteTime = orderItem.committed_completion_time
                        overTime_sum += max(0, curr_Time[minT2VehicleIndex] - commitCompleteTime)
                    beforeOrderId = nextOrderId

            #isSameAddress = True
            serviceTime += nextNode.service_time
            curr_Node[minT2VehicleIndex] += 1

        if curr_Node[minT2VehicleIndex] >= nNode[minT2VehicleIndex]:
            n -= 1
            curr_Node[minT2VehicleIndex] = sys.maxsize
            curr_Time[minT2VehicleIndex] = sys.maxsize
            nNode[minT2VehicleIndex] = 0
        else:
            disAndTime = route_map[(curFactoryId , minTNodeList[curr_Node[minT2VehicleIndex]].id)]
            
            distance, time = disAndTime
            curr_Time[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + serviceTime + time
            leaveLastNodeTime[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + serviceTime
            driving_distance += distance

        tw = [minT, tTrue + APPROACHING_DOCK_TIME + serviceTime]
        docktable.setdefault(minTNode.id, []).append(tw)

    F = Delta * overTime_sum + driving_distance / len(id_to_vehicle)
    if F < 0:
        print("The objective function is less than 0.")
    return F

def dispatch_nodePair(node_list: list[Node] , id_to_vehicle: Dict[str , Vehicle] , vehicleid_to_plan: Dict[str, list[Node]], route_map: Dict[tuple , tuple] ):
    bestInsertVehicleID: str = None
    bestInsertPosI: int = -1
    bestInsertPosJ: int = -1
    bestNodeList : list[Node] = []
    isExhausive  = False
    
    new_pickup_node = node_list[0]
    new_delivery_node = node_list[1]
    minCostDelta = float('inf')
    for vehicleID , vehicle in id_to_vehicle.items():
        vehicle_plan = vehicleid_to_plan[vehicleID]
        node_list_size = 0
        if vehicle_plan is not None:
            node_list_size = len(vehicle_plan)

        insert_pos = 0
        first_node_is_locked = False
        first_node_is_delivery = False
        first_node_is_pickup = False
        model_nodes_num = node_list_size + 2
        begin_delivery_node_num = 0
        first_merge_node_num = 0

        if vehicle.des is not None:
            if new_pickup_node.id != vehicle.des.id:
                insert_pos = 1
            first_node_is_locked = True

            if vehicle.des.delivery_item_list is not None and len(vehicle.des.delivery_item_list) > 0:
                first_node_is_delivery = True

            if vehicle.des.pickup_item_list is not None and len(vehicle.des.pickup_item_list) > 0:
                first_node_is_pickup = True

            if vehicle_plan is not None:
                for node in vehicle_plan:
                    if vehicle.des.id != node.id:
                        break
                    first_merge_node_num += 1

        model_nodes_num -= first_merge_node_num

        modle_node_list = [] #thêm các cặp node gửi và nhận theo thứ tự mới
        exhaustive_route_node_list = [] # chứa các node đầu tiến của plan (node giao) (node không thể thay đổi do đang trên đường thực hiện rồi)
        cp_route_node_list = None # chứa phần còn lại sau khi tách các node giao ở đầu (phần mà các node đều đầy đủ)

        if vehicle_plan is not None:
            cp_route_node_list = list(vehicle_plan)
        empty_pos_num = 0
        
        if model_nodes_num <= 8:
            begin_idx = 0
            if first_merge_node_num > 0:
                begin_idx = first_merge_node_num
                while first_merge_node_num > 0:
                    exhaustive_route_node_list.append(cp_route_node_list[0])
                    cp_route_node_list.pop(0)
                    first_merge_node_num -= 1
            
            count = 0
            i = 0
            while i < len(cp_route_node_list) if cp_route_node_list is not None else 0:
                pickup_node, delivery_node = None, None
                order_item_id = ""
                
                if cp_route_node_list[i].pickup_item_list and len(cp_route_node_list[i].pickup_item_list) > 0:
                    order_item_id = cp_route_node_list[i].pickup_item_list[0].id
                    pickup_node = cp_route_node_list[i]
                    cp_route_node_list.pop(i)

                    for j in range(i, len(cp_route_node_list)):
                        if cp_route_node_list[j].delivery_item_list and len(cp_route_node_list[j].delivery_item_list) > 0:
                            item_len = len(cp_route_node_list[j].delivery_item_list)
                            item_id = cp_route_node_list[j].delivery_item_list[item_len - 1].id
                            
                            if order_item_id == item_id:
                                delivery_node = cp_route_node_list[j]
                                cp_route_node_list.pop(j)
                                break
                    modle_node_list.insert(count, pickup_node)
                    modle_node_list.insert(count + 1, delivery_node)
                    count += 2
                    continue  

                i += 1 

            modle_node_list.insert(count, new_pickup_node)
            modle_node_list.insert(count + 1, new_delivery_node)

            empty_pos_num = 0 if cp_route_node_list is None else len(cp_route_node_list)
            
            while cp_route_node_list:
                modle_node_list.append(cp_route_node_list.pop(0))

            model_nodes_num = len(modle_node_list) + empty_pos_num

        if model_nodes_num <= 8:
            if model_nodes_num == 2:
                exhaustive_route_node_list.append(new_pickup_node)
                exhaustive_route_node_list.append(new_delivery_node)
                tmp_cost = cost(exhaustive_route_node_list , vehicle , id_to_vehicle , route_map , vehicleid_to_plan)
                if tmp_cost < minCostDelta: 
                    minCostDelta = tmp_cost
                    isExhausive = True
                    bestInsertVehicleID = vehicleID
                    bestNodeList = exhaustive_route_node_list
            elif model_nodes_num == 4:
                for i in range(len(modle4)):
                    if empty_pos_num == 1:
                        if modle4[i][0] == 0:
                            for j in range(1, 4):
                                exhaustive_route_node_list.append(modle_node_list[modle4[i][j] - 1])

                            costValue = cost(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map , vehicleid_to_plan)
                            
                            if costValue < minCostDelta:
                                minCostDelta = costValue
                                bestNodeList = exhaustive_route_node_list[:]
                                bestInsertVehicleID = vehicleID
                                isExhausive = True

                            del exhaustive_route_node_list[-3:]
                    else:
                        for j in range(4):
                            exhaustive_route_node_list.append(modle_node_list[modle4[i][j]])

                        costValue = cost(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map , vehicleid_to_plan)

                        if costValue < minCostDelta:
                            minCostDelta = costValue
                            bestNodeList = exhaustive_route_node_list[:]
                            bestInsertVehicleID = vehicleID
                            isExhausive = True

                        del exhaustive_route_node_list[-4:]
            elif model_nodes_num == 6:
                for i in range(len(modle6)):
                    if empty_pos_num == 1:
                        if modle6[i][0] == 0:
                            for j in range(1, 6):
                                exhaustive_route_node_list.append(modle_node_list[modle6[i][j] - 1])
                            costValue = cost(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map , vehicleid_to_plan)
                            if costValue < minCostDelta:
                                minCostDelta = costValue
                                bestNodeList = exhaustive_route_node_list[:]
                                bestInsertVehicleID = vehicleID
                                isExhausive = True
                            del exhaustive_route_node_list[-5:]
                    elif empty_pos_num == 2:
                        if modle6[i][0] == 0 and modle6[i][1] == 1:
                            for j in range(2, 6):
                                exhaustive_route_node_list.append(modle_node_list[modle6[i][j] - 2])
                            costValue = cost(exhaustive_route_node_list, vehicle ,id_to_vehicle ,route_map , vehicleid_to_plan )
                            if costValue < minCostDelta:
                                minCostDelta = costValue
                                bestNodeList = exhaustive_route_node_list[:]
                                bestInsertVehicleID = vehicleID
                                isExhausive = True
                            del exhaustive_route_node_list[-4:]
                    else:
                        for j in range(6):
                            exhaustive_route_node_list.append(modle_node_list[modle6[i][j]])
                        costValue = cost(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map, vehicleid_to_plan)
                        if costValue < minCostDelta:
                            minCostDelta = costValue
                            bestNodeList = exhaustive_route_node_list[:]
                            bestInsertVehicleID = vehicleID
                            isExhausive = True
                        del exhaustive_route_node_list[-6:]
            elif model_nodes_num == 8:
                for i in range(len(modle8)):
                    if empty_pos_num == 1:
                        if modle8[i][0] == 0:
                            for j in range(1, 8):
                                exhaustive_route_node_list.append(modle_node_list[modle8[i][j] - 1])
                            costValue = cost(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map, vehicleid_to_plan)
                            if costValue < minCostDelta:
                                minCostDelta = costValue
                                bestNodeList = exhaustive_route_node_list[:]
                                bestInsertVehicleID = vehicleID
                                isExhausive = True
                            del exhaustive_route_node_list[-7:]
                    elif empty_pos_num == 2:
                        if modle8[i][0] == 0 and modle8[i][1] == 1:
                            for j in range(2, 8):
                                exhaustive_route_node_list.append(modle_node_list[modle8[i][j] - 2])
                            costValue = cost(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map, vehicleid_to_plan)
                            if costValue < minCostDelta:
                                minCostDelta = costValue
                                bestNodeList = exhaustive_route_node_list[:]
                                bestInsertVehicleID = vehicleID
                                isExhausive = True
                            del exhaustive_route_node_list[-6:]
                    elif empty_pos_num == 3:
                        if modle8[i][0] == 0 and modle8[i][1] == 1 and modle8[i][2] == 2:
                            for j in range(3, 8):
                                exhaustive_route_node_list.append(modle_node_list[modle8[i][j] - 3])
                            costValue = cost(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map, vehicleid_to_plan)
                            if costValue < minCostDelta:
                                minCostDelta = costValue
                                bestNodeList = exhaustive_route_node_list[:]
                                bestInsertVehicleID = vehicleID
                                isExhausive = True
                            del exhaustive_route_node_list[-5:]
                    else:
                        for j in range(8):
                            exhaustive_route_node_list.append(modle_node_list[modle8[i][j]])
                        costValue = cost(exhaustive_route_node_list, vehicle , id_to_vehicle , route_map, vehicleid_to_plan)
                        if costValue < minCostDelta:
                            minCostDelta = costValue
                            bestNodeList = exhaustive_route_node_list[:]
                            bestInsertVehicleID = vehicleID
                            isExhausive = True
                        del exhaustive_route_node_list[-8:]
        else:
            for i in range(insert_pos, node_list_size + 1):
                if vehicle_plan is not None:
                    tempRouteNodeList = vehicle_plan[:]
                else:
                    tempRouteNodeList = []

                tempRouteNodeList.insert(i, new_pickup_node)

                for j in range(i + 1, node_list_size + 2):
                    if j != i + 1 and tempRouteNodeList[j - 1].pickup_item_list is not None and tempRouteNodeList[j - 1].pickup_item_list:
                        for k in range(j, node_list_size + 2):
                            if tempRouteNodeList[k].delivery_item_list is not None and tempRouteNodeList[k].delivery_item_list:
                                len_k = len(tempRouteNodeList[k].delivery_item_list)
                                if tempRouteNodeList[j - 1].pickup_item_list[0].id == tempRouteNodeList[k].delivery_item_list[len_k - 1].id:
                                    j = k + 1
                                    break

                    elif tempRouteNodeList[j - 1].delivery_item_list is not None and tempRouteNodeList[j - 1].delivery_item_list:
                        is_terminal = True
                        for k in range(j - 2, -1, -1):
                            if tempRouteNodeList[k].pickup_item_list is not None and tempRouteNodeList[k].pickup_item_list:
                                len_j = len(tempRouteNodeList[j - 1].delivery_item_list)
                                if tempRouteNodeList[j - 1].delivery_item_list[len_j - 1].id == tempRouteNodeList[k].pickup_item_list[0].id:
                                    if k < i:
                                        is_terminal = True
                                        break
                                    elif k > i:
                                        is_terminal = False
                                        break
                        if is_terminal:
                            break

                    tempRouteNodeList.insert(j, new_delivery_node)

                    costValue = cost(tempRouteNodeList, vehicle, id_to_vehicle , route_map , vehicleid_to_plan)
                    if costValue < minCostDelta:
                        minCostDelta = costValue
                        bestInsertPosI = i
                        bestInsertPosJ = j
                        bestInsertVehicleID = vehicleID
                        isExhausive = False

                    tempRouteNodeList.pop(j)

        return isExhausive , bestInsertVehicleID, bestInsertPosI, bestInsertPosJ , bestNodeList


def inter_couple_exchange():
    pass

def block_exchange():
    pass

def block_relocate():
    pass

def multi_pd_group_relocate():
    pass

def improve_ci_path_by_2_opt():
    pass

def cost():
    pass