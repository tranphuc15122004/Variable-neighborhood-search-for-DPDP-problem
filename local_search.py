import json
import os
import string
from typing import Dict
from Read_input import Input
from Object import Factory, Node, Vehicle, VehicleInfo, OrderItem, Destination
import copy
import numpy as np

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

def dispatch_nodePair(node_list: list[Node] , id_to_vehicle: Dict[str , Vehicle] , vehicleid_to_plan: Dict[str, list[Node]]):
    new_pickup_node = node_list[0]
    new_delivery_node = node_list[1]
    minCostDelta = float('inf')
    index = 0
    isExhausive  = False
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
        exhaustive_route_node_list = [] # chứa các node đầu tiến của plan (node giao)
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


