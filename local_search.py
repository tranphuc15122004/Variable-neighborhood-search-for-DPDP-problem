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


# Truyền tham số bình thường vào hàm này (Không cần truyền Copy)
def get_OngoingSuperNode (vehicleid_to_plan: Dict[str , List[Node]] , id_to_vehicle: Dict[str , Vehicle] ) -> Dict[int , Dict[str, Node]]:
    OngoingSuperNodes : Dict[int , Dict[str , Node]] 
    
    vehicleNo = 1
    NodePairNum = 0 
    # xet từng kế hoạch di chuyển của phương tiện
    for vehicleID , vehicle_plan in vehicleid_to_plan.items():
        vehicle = id_to_vehicle[vehicleID]
        if len(vehicle_plan) > 0:
            index = 0
            if vehicle.des is not None:
                index =1
            
            pickup_node_heap: deque[Node] = deque()
            p_node_idx_heap= deque()
            p_and_d_node_map = {}
            idx = 0
            before_p_factory_id = before_d_factory_id = None
            before_p_node_idx = before_d_node_idx = 0
            
            # đối với mỗi node của phương tiện
            for i in range (index , len(vehicle_plan)):
                curr = vehicle_plan[i]
                if curr.delivery_item_list  and curr.pickup_item_list :
                    print ("Exits combine node exception when Local search" , file= sys.stderr)
                
                heapTopOrderItemId = pickup_node_heap[0].pickup_item_list[0].id if pickup_node_heap else ""
                if curr.delivery_item_list:
                    len = len(curr.delivery_item_list)
                    if curr.delivery_item_list[-1].id == heapTopOrderItemId:
                        pickup_node_key = f"V_{vehicleNo},{p_node_idx_heap[0]}"
                        delivery_node_key = f"V_{vehicleNo},{i}"
                
                    if len(p_and_d_node_map) >= 2:
                        if (pickup_node_heap[0].id != before_p_factory_id or p_node_idx_heap[0] + 1 != before_p_node_idx or curr.get_id() != before_d_factory_id or i - 1 != before_d_node_idx):
                            OngoingSuperNodes[NodePairNum] = p_and_d_node_map
                            NodePairNum += 1
                            p_and_d_node_map = {}

                    p_and_d_node_map[pickup_node_key] = pickup_node_heap[0]
                    p_and_d_node_map[delivery_node_key] = curr

                    before_p_factory_id = pickup_node_heap[0].get_id()
                    before_p_node_idx = p_node_idx_heap[0]
                    before_d_factory_id = curr.id
                    before_d_node_idx = i
                    pickup_node_heap.pop(0)
                    p_node_idx_heap.pop(0)
                
                
                if curr.pickup_item_list:
                    pickup_node_heap.appendleft(curr)
                    p_node_idx_heap.appendleft(i)
                    if p_and_d_node_map:
                        OngoingSuperNodes[NodePairNum] = p_and_d_node_map
                        NodePairNum += 1
                        p_and_d_node_map.clear()
            
            if len(p_and_d_node_map) >= 2:
                OngoingSuperNodes[NodePairNum] = p_and_d_node_map
                NodePairNum += 1
        vehicleNo += 1
    
    return OngoingSuperNodes


def isFeasible(route_node_list : List[Node] , carrying_items : List[OrderItem] , capacity : float ):
    unload_item_list = carrying_items[::-1] if carrying_items else []

    for node in route_node_list:
        delivery_items : List[OrderItem] = node.delivery_item_list
        pickup_items : List[OrderItem]= node.pickup_item_list

        if delivery_items:
            for order_item in delivery_items:
                if not unload_item_list or unload_item_list[0] is None or unload_item_list[0].id != order_item.id:
                    print("Violate FILO" , file= sys.stderr)
                    return False
                unload_item_list.pop(0)

        if pickup_items:
            unload_item_list = pickup_items + unload_item_list

    if unload_item_list:
        print("Violate FILO" ,file= sys.stderr)
        return False

    left_capacity = capacity
    if carrying_items:
        for order_item in carrying_items:
            left_capacity -= order_item.demand

    for node in route_node_list:
        delivery_items = node.delivery_item_list
        pickup_items = node.pickup_item_list

        if delivery_items:
            for order_item in delivery_items:
                left_capacity += order_item.demand
                if left_capacity > capacity:
                    return False

        if pickup_items:
            for order_item in pickup_items:
                left_capacity -= order_item.demand
                if left_capacity < 0:
                    return False

    return True

def cost_of_a_route (temp_route_node_list : List[Node] , vehicle: Vehicle , id_to_vehicle: Dict[str , Vehicle] , route_map: Dict[tuple , tuple] , vehicleid_to_plan: Dict[str , list[Node]]) -> float:
    curr_factoryID = vehicle.cur_factory_id
    driving_dis  : float = 0.0
    overtime_Sum : float = 0.0
    objF : float = 0.0
    capacity = vehicle.board_capacity
    carrying_Items : List[OrderItem] = vehicle.carrying_items if vehicle.des else []
    if temp_route_node_list and not isFeasible(temp_route_node_list , carrying_Items , capacity):
        return math.inf
    
    dock_table: Dict[str, List[List[int]]] = {}
    n: int = 0
    vehicle_num: int = len(id_to_vehicle)

    curr_node: List[int] = [0] * vehicle_num
    curr_time: List[int] = [0] * vehicle_num
    leave_last_node_time: List[int] = [0] * vehicle_num

    update_time= id_to_vehicle["V_1"].gps_update_time
    n_node: List[int] = [0] * vehicle_num
    index = 0
    
    for vehicleID , otherVehicle in id_to_vehicle.items():
        distance = 0
        time  = 0
        
        if otherVehicle.cur_factory_id and len(otherVehicle.cur_factory_id) > 0:
            if otherVehicle.leave_time_at_current_factory > otherVehicle.gps_update_time:
                tw: List[int] = [
                    otherVehicle.arrive_time_at_current_factory,
                    otherVehicle.leave_time_at_current_factory
                ]
                tw_list: Optional[List[List[int]]] = dock_table.get(otherVehicle.cur_factory_id)
                if tw_list is None:
                    tw_list = []
                tw_list.append(tw)
                dock_table[otherVehicle.cur_factory_id] = tw_list
            leave_last_node_time[index] = otherVehicle.leave_time_at_current_factory
        else:
            leave_last_node_time[index] = otherVehicle.gps_update_time
        
        # Neu la xe dang xet
        if vehicleID == vehicle.id:
            if not temp_route_node_list or len(temp_route_node_list) == 0:
                curr_node[index] = math.inf
                curr_time[index] = math.inf
                n_node[index] = math.inf
            else:
                curr_node[index] = 0
                n_node[index] = len(temp_route_node_list)
                if vehicle.des == None:
                    if vehicle.cur_factory_id == "":
                        print("cur factory have no value" , file= sys.stderr)
                    if len(temp_route_node_list) == 0:
                        print("tempRouteNodeList have no length" , file= sys.stderr)
                        
                    if vehicle.cur_factory_id == temp_route_node_list[0].id:
                        curr_time[index] = vehicle.leave_time_at_current_factory
                    else:
                        dis_and_time = route_map.get((vehicle.cur_factory_id , temp_route_node_list[0].id))
                        distance = float(dis_and_time[0])
                        time = int(dis_and_time[1])
                        curr_time[index] = vehicle.leave_time_at_current_factory + time
                        driving_dis += distance
                else:
                    if curr_factoryID is not None and len(curr_factoryID) > 0:
                        if  curr_factoryID == temp_route_node_list[0].id:
                            curr_time[index] = vehicle.leave_time_at_current_factory
                        else:
                            curr_time[index] = vehicle.leave_time_at_current_factory
                            dis_and_time = route_map.get((curr_factoryID , temp_route_node_list[0].id))
                            distance = float(dis_and_time[0])
                            time = int(dis_and_time[1])
                            driving_dis += distance
                            curr_time[index] += time
                    else:
                        curr_time[index] = vehicle.des.arrive_time
                
                n += 1
        # Neu khong phai xe dang xet
        elif vehicleid_to_plan[vehicleID] and len(vehicleid_to_plan[vehicleID]) > 0:    
            curr_node[index] = 0
            n_node[index] = len(vehicleid_to_plan[vehicleID]) 
            
            if otherVehicle.des is None:
                if otherVehicle.cur_factory_id == vehicleid_to_plan[vehicleID][0].id:
                    curr_time[index] = otherVehicle.leave_time_at_current_factory
                else:
                    dis_and_time = route_map.get((otherVehicle.cur_factory_id , vehicleid_to_plan[vehicleID][0].id))
                    if dis_and_time is None:
                        print("no distance" , file= sys.stderr)
                    
                    distance = float(dis_and_time[0])
                    time = int(dis_and_time[1])
                    curr_time[index] = otherVehicle.leave_time_at_current_factory + time
                    driving_dis += distance
            else:
                if otherVehicle.cur_factory_id is not None and len(otherVehicle.cur_factory_id) > 0:
                    if otherVehicle.cur_factory_id == vehicleid_to_plan[vehicleID][0].id:
                        curr_time[index]  = otherVehicle.leave_time_at_current_factory
                    else:
                        curr_time[index] = otherVehicle.leave_time_at_current_factory
                        dis_and_time = route_map.get((otherVehicle.cur_factory_id , vehicleid_to_plan[vehicleID][0].id))
                        distance = float(dis_and_time[0])
                        time = int(dis_and_time[1])
                        curr_time[index] += time
                        driving_dis += distance
                else: 
                    curr_time[index] = otherVehicle.des.arrive_time
            n+=1
        else:
            curr_time[index] = math.inf
            curr_time[index] = math.inf
            n_node[index] = 0
        index += 1
        
    flag = False
    while n > 0:
        minT = math.inf
        minT2VehicleIndex = 0
        tTrue = minT
        idx = 0
        
        for i in range (len(vehicle_num)):
            if curr_time[i] < minT:
                minT = curr_time[i]
                minT2VehicleIndex = i
        
        minT2VehicleIndex += 1
        minT2VehicleID = "V_" + str(minT2VehicleIndex)
        minT2VehicleIndex -= 1
        
        minTNodeList: List[Node] = []
        if minT2VehicleID == vehicle.id:
            minTNodeList = temp_route_node_list
        else:
            minTNodeList = vehicleid_to_plan[minT2VehicleID]
        minTNode = minTNodeList[curr_node[minT2VehicleIndex]]
        
        if minTNode.delivery_item_list and len(minTNode.delivery_item_list) > 0:
            beforeOrderID = ""
            nextOrderID = ""
            for order_item in minTNode.delivery_item_list:
                nextOrderID = order_item.id
                if beforeOrderID != nextOrderID:
                    commitCompleteTime = order_item.committed_completion_time
                    overtime_Sum += max(0 , curr_time[minT2VehicleIndex] - commitCompleteTime)
                beforeOrderID = nextOrderID
        
        usedEndTime : List[int] = []
        timeSlots : List[List[int]] =  dock_table.get(minTNode.id, [])
        if timeSlots:
            i = 0
            while i < len(timeSlots):
                time_slot = timeSlots[i]
                if time_slot[1] <= minT:
                    timeSlots.pop(i)  # Xóa phần tử nếu end_time <= minT
                elif time_slot[0] <= minT < time_slot[1]:
                    usedEndTime.append(time_slot[1])
                    i += 1
                else:
                    print("------------ timeslot.start > minT --------------")
                    i += 1

        if len(usedEndTime) < 6:
            tTrue = minT
        else:
            flag = True
            idx = len(usedEndTime) - 6
            usedEndTime.sort()
            tTrue = usedEndTime[idx]
            
        is_same_address = False
        service_time = minTNodeList[curr_node[minT2VehicleIndex]].service_time
        cur_factory_id = minTNodeList[curr_node[minT2VehicleIndex]].id
        curr_node[minT2VehicleIndex] += 1

        while (curr_node[minT2VehicleIndex] < n_node[minT2VehicleIndex] and
            cur_factory_id == minTNodeList[curr_node[minT2VehicleIndex]].get_id()):

            delivery_item_list = minTNodeList[curr_node[minT2VehicleIndex]].delivery_item_list
            
            if delivery_item_list and len(delivery_item_list) > 0:
                before_order_id = ""
                next_order_id = ""

                for order_item in delivery_item_list:
                    next_order_id = order_item.order_id
                    if before_order_id != next_order_id:
                        commit_complete_time = order_item.committed_completion_time
                        overtime_Sum += max(0, curr_time[minT2VehicleIndex] - commit_complete_time)
                    before_order_id = next_order_id

            is_same_address = True
            service_time += minTNodeList[curr_node[minT2VehicleIndex]].service_time
            curr_node[minT2VehicleIndex] += 1
            
        if curr_node[minT2VehicleIndex] >= n_node[minT2VehicleIndex]:
            n -= 1
            curr_node[minT2VehicleIndex] = math.inf
            curr_time[minT2VehicleIndex] = math.inf
            n_node[minT2VehicleIndex] = 0
        else:
            dis_and_time = route_map.get((cur_factory_id , minTNodeList[curr_node[minT2VehicleIndex]].id))
            if dis_and_time:
                distance = float(dis_and_time[0])
                time = int(dis_and_time[1])

                curr_time[minT2VehicleID] = tTrue + APPROACHING_DOCK_TIME + service_time + time
                leave_last_node_time[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + service_time
                driving_dis += distance

        tw = [minT, tTrue + APPROACHING_DOCK_TIME + service_time]
        tw_list = dock_table.get(minTNode.id, [])

        tw_list.append(tw)
        dock_table[minTNode.id] = tw_list
    
    objF = Delta * overtime_Sum + driving_dis / float(len(id_to_vehicle))
    if objF < 0:
        print("the objective function less than 0" , file= sys.stderr)
    return objF


def total_cost(id_to_vehicle: Dict[str , Vehicle] , route_map: Dict[tuple , tuple] , vehicleid_to_plan: Dict[str , list[Node]]) -> float:
    driving_dis  : float = 0.0
    overtime_Sum : float = 0.0
    objF : float = 0.0
    dock_table: Dict[str, List[List[int]]] = {}
    n: int = 0
    vehicle_num: int = len(id_to_vehicle)

    curr_node: List[int] = [0] * vehicle_num
    curr_time: List[int] = [0] * vehicle_num
    leave_last_node_time: List[int] = [0] * vehicle_num

    update_time= id_to_vehicle["V_1"].gps_update_time
    n_node: List[int] = [0] * vehicle_num
    index = 0
    
    for vehicleID , otherVehicle in id_to_vehicle.items():
        distance = 0
        time  = 0
        
        if otherVehicle.cur_factory_id and len(otherVehicle.cur_factory_id) > 0:
            if otherVehicle.leave_time_at_current_factory > otherVehicle.gps_update_time:
                tw: List[int] = [
                    otherVehicle.arrive_time_at_current_factory,
                    otherVehicle.leave_time_at_current_factory
                ]
                tw_list: Optional[List[List[int]]] = dock_table.get(otherVehicle.cur_factory_id)
                if tw_list is None:
                    tw_list = []
                tw_list.append(tw)
                dock_table[otherVehicle.cur_factory_id] = tw_list
            leave_last_node_time[index] = otherVehicle.leave_time_at_current_factory
        else:
            leave_last_node_time[index] = otherVehicle.gps_update_time
        
        if vehicleid_to_plan.get(vehicleID) and len(vehicleid_to_plan.get(vehicleID)) > 0:    
            curr_node[index] = 0
            n_node[index] = len(vehicleid_to_plan[vehicleID]) 
            
            if otherVehicle.des is None:
                if otherVehicle.cur_factory_id == vehicleid_to_plan[vehicleID][0].id:
                    curr_time[index] = otherVehicle.leave_time_at_current_factory
                else:
                    dis_and_time = route_map.get((otherVehicle.cur_factory_id , vehicleid_to_plan[vehicleID][0].id))
                    if dis_and_time is None:
                        print("no distance" , file= sys.stderr)
                    
                    distance = float(dis_and_time[0])
                    time = int(dis_and_time[1])
                    curr_time[index] = otherVehicle.leave_time_at_current_factory + time
                    driving_dis += distance
            else:
                if otherVehicle.cur_factory_id is not None and len(otherVehicle.cur_factory_id) > 0:
                    if otherVehicle.cur_factory_id == vehicleid_to_plan[vehicleID][0].id:
                        curr_time[index]  = otherVehicle.leave_time_at_current_factory
                    else:
                        curr_time[index] = otherVehicle.leave_time_at_current_factory
                        dis_and_time = route_map.get((otherVehicle.cur_factory_id , vehicleid_to_plan[vehicleID][0].id))
                        distance = float(dis_and_time[0])
                        time = int(dis_and_time[1])
                        curr_time[index] += time
                        driving_dis += distance
                else: 
                    curr_time[index] = otherVehicle.des.arrive_time
            n+=1
        else:
            curr_time[index] = math.inf
            curr_time[index] = math.inf
            n_node[index] = 0
        index += 1
        
    flag = False
    while n > 0:
        minT = math.inf
        minT2VehicleIndex = 0
        tTrue = minT
        idx = 0
        
        for i in range (len(vehicle_num)):
            if curr_time[i] < minT:
                minT = curr_time[i]
                minT2VehicleIndex = i
        
        minT2VehicleIndex += 1
        minT2VehicleID = "V_" + str(minT2VehicleIndex)
        minT2VehicleIndex -= 1
        
        minTNodeList: List[Node] = []
        minTNodeList = vehicleid_to_plan.get(minT2VehicleID)
        minTNode = minTNodeList[curr_node[minT2VehicleIndex]]
        
        if minTNode.delivery_item_list and len(minTNode.delivery_item_list) > 0:
            beforeOrderID = ""
            nextOrderID = ""
            for order_item in minTNode.delivery_item_list:
                nextOrderID = order_item.id
                if beforeOrderID != nextOrderID:
                    commitCompleteTime = order_item.committed_completion_time
                    overtime_Sum += max(0 , curr_time[minT2VehicleIndex] - commitCompleteTime)
                beforeOrderID = nextOrderID
        
        usedEndTime : List[int] = []
        timeSlots : List[List[int]] =  dock_table.get(minTNode.id, [])
        if timeSlots:
            i = 0
            while i < len(timeSlots):
                time_slot = timeSlots[i]
                if time_slot[1] <= minT:
                    timeSlots.pop(i)  # Xóa phần tử nếu end_time <= minT
                elif time_slot[0] <= minT < time_slot[1]:
                    usedEndTime.append(time_slot[1])
                    i += 1
                else:
                    print("------------ timeslot.start > minT --------------")
                    i += 1

        if len(usedEndTime) < 6:
            tTrue = minT
        else:
            flag = True
            idx = len(usedEndTime) - 6
            usedEndTime.sort()
            tTrue = usedEndTime[idx]
            
        is_same_address = False
        service_time = minTNodeList[curr_node[minT2VehicleIndex]].service_time
        cur_factory_id = minTNodeList[curr_node[minT2VehicleIndex]].id
        curr_node[minT2VehicleIndex] += 1

        while (curr_node[minT2VehicleIndex] < n_node[minT2VehicleIndex] and
            cur_factory_id == minTNodeList[curr_node[minT2VehicleIndex]].get_id()):

            delivery_item_list = minTNodeList[curr_node[minT2VehicleIndex]].delivery_item_list
            
            if delivery_item_list and len(delivery_item_list) > 0:
                before_order_id = ""
                next_order_id = ""

                for order_item in delivery_item_list:
                    next_order_id = order_item.order_id
                    if before_order_id != next_order_id:
                        commit_complete_time = order_item.committed_completion_time
                        overtime_Sum += max(0, curr_time[minT2VehicleIndex] - commit_complete_time)
                    before_order_id = next_order_id

            is_same_address = True
            service_time += minTNodeList[curr_node[minT2VehicleIndex]].service_time
            curr_node[minT2VehicleIndex] += 1
            
        if curr_node[minT2VehicleIndex] >= n_node[minT2VehicleIndex]:
            n -= 1
            curr_node[minT2VehicleIndex] = math.inf
            curr_time[minT2VehicleIndex] = math.inf
            n_node[minT2VehicleIndex] = 0
        else:
            dis_and_time = route_map.get((cur_factory_id , minTNodeList[curr_node[minT2VehicleIndex]].id))
            if dis_and_time:
                distance = float(dis_and_time[0])
                time = int(dis_and_time[1])

                curr_time[minT2VehicleID] = tTrue + APPROACHING_DOCK_TIME + service_time + time
                leave_last_node_time[minT2VehicleIndex] = tTrue + APPROACHING_DOCK_TIME + service_time
                driving_dis += distance

        tw = [minT, tTrue + APPROACHING_DOCK_TIME + service_time]
        tw_list = dock_table.get(minTNode.id, [])

        tw_list.append(tw)
        dock_table[minTNode.id] = tw_list
    
    objF = Delta * overtime_Sum + driving_dis / float(len(id_to_vehicle))
    if objF < 0:
        print("the objective function less than 0" , file= sys.stderr)
    return objF


