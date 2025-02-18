from datetime import datetime , timedelta
import json
import os
import string
import sys
from typing import Dict , List, Optional
from Read_input import Input
from Object import Factory, Node, Vehicle, VehicleInfo, OrderItem, Destination
import copy
import numpy as np
from constant import APPROACHING_DOCK_TIME , Delta , debugPeriod , SLACK_TIME_THRESHOLD 
import time

from local_search import *
from local_search2 import *


input_directory = ""
solution_json_path = "/solution.json"
delta_t = "0000-0010"
before_cost = 0.0
completeOrderItems: str = ""
newOrderItems: str = ""
onVehicleOrderItems : str = ""
unallocatedOrderItems: str = ""
routeBefore: str = ""
used_time = 0.0

## Cần có file JSON solution để debug lại
# CÓ vẻ đúng rồi
def restore_scene_with_single_node(vehicleid_to_plan: Dict[str , List[Node]], id_to_ongoing_items: Dict[str , OrderItem], id_to_unlocated_items: Dict[str , OrderItem], id_to_vehicle: Dict[str , Vehicle] , id_to_factory: Dict[str , Factory], id_to_allorder: Dict[str , OrderItem]) -> List[str]:
    global before_cost, delta_t, solution_json_path , completeOrderItems , newOrderItems , onVehicleOrderItems , unallocatedOrderItems , routeBefore

    vehicleid_to_plan = {vehicle_id: None for vehicle_id in id_to_vehicle}
    
    for key in id_to_ongoing_items:
        onVehicleOrderItems += f"{key} "
    onVehicleOrderItems.strip()
    
    for key in id_to_unlocated_items:
        unallocatedOrderItems += f"{key} "
    unallocatedOrderItems.strip()
    
    if os.path.exists(solution_json_path) and os.path.getsize(solution_json_path) > 0:
        try:
            with open(solution_json_path , 'r') as file:
                before_solution = json.loads(file)
                no = int(before_solution.get('no', 0))
                f = (no + 1) * 10
                t = (no + 1) * 10 + 10
                global delta_t
                delta_t = f"{f:04d}-{t:04d}"
                routeBefore = before_solution.get("route_after", "")
                splited_routeBefore : List[str] = routeBefore.split("V")
                
                last_on_vehicle_items: List[str] = before_solution.get("onvehicle_order_items", "").split(" ")
                curr_on_vehicle_items : List[str] = onVehicleOrderItems.split(" ")
                for key in last_on_vehicle_items:
                    if key not in curr_on_vehicle_items:
                        completeOrderItems += f"{key} "
                completeOrderItems.strip()
                completeItemsArray = completeOrderItems.split(" ")
                
                last_unallocated_items : List[str] = before_solution.get("unallocated_order_items", "").split(" ")
                curr_unallocated_items : List[str] = unallocatedOrderItems.split(" ")
                for key in last_on_vehicle_items:
                    if key not in curr_on_vehicle_items:
                        newOrderItems += f"{key} "
                newOrderItems.strip()
                
                for route in splited_routeBefore:
                    route.strip()
                    str_len : int = len(route.split(':')[1])
                    numstr = route.split(":")[0]
                    vehicleID = "V_" + numstr[1:]
                    if str_len < 3: 
                        vehicleid_to_plan[vehicleID] = None
                        continue
                    
                    route_nodes_str = route.split(":")[1]
                    route_nodes = route_nodes_str[1:len(route_nodes_str) - 1].split(" ")
                    node_list : List[str] = list(route_nodes)
                    
                    node_list = [
                        node for node in node_list
                        if not (
                            (node.startswith("d") and node.split("_")[1] in completeItemsArray) or
                            (node.startswith("p") and node.split("_")[1] in curr_on_vehicle_items)
                        )
                    ]
                    
                    if len(node_list) > 0:
                        planroute : List[Node] = []
                        for node in node_list:
                            deliveryItemList : List[OrderItem] = []
                            pickupItemList : List[OrderItem] = []
                            temp : OrderItem = None
                            op = node[0][0:1]
                            opNumstr = node.split("_")
                            opItemNum = int(opNumstr[0][1 , len(opNumstr)])
                            orderItemID = node.split("_")[1]
                            idEndNumber = int(orderItemID.split("-")[1])
                            
                            if op == 'd':
                                for i in range(opItemNum):
                                    temp = id_to_allorder[orderItemID]
                                    deliveryItemList.append(temp)
                                    idEndNumber -= 1
                                    orderItemID =  orderItemID.split("-")[0] + "-" + idEndNumber
                            else:
                                for i in range(opItemNum):
                                    temp = id_to_allorder[orderItemID]
                                    pickupItemList.append(temp)
                                    idEndNumber -= 1
                                    orderItemID =  orderItemID.split("-")[0] + "-" + idEndNumber
                                    
                            factoryID = ""
                            if op == 'd':
                                factoryID = temp.delivery_factory_id
                            else:
                                factoryID = temp.pickup_factory_id
                            factory = id_to_factory[factoryID]
                            
                            planroute.append(Node(factoryID , deliveryItemList , pickupItemList ,None ,None , factory.lng , factory.lat))
                        if len(planroute) > 0:
                            vehicleid_to_plan[vehicleID] = planroute
        except Exception as e:
            print(f"Error: {e}" , file= sys.stderr)
    else:
        newOrderItems  = unallocatedOrderItems
        completeOrderItems = ""
        routeBefore = ""
        delta_t = "0000-0010"
    
    new_order_itemIDs = newOrderItems.split(" ")
    return new_order_itemIDs


""" def Restore(vehicleid_to_plan: Dict[str , List[Node]], id_to_ongoing_items: Dict[str , OrderItem], id_to_unlocated_items: Dict[str , OrderItem], id_to_vehicle: Dict[str , Vehicle] , id_to_factory: Dict[str , Factory], id_to_allorder: Dict[str , OrderItem]):
    vehicleid_to_plan = {vehicle_id: None for vehicle_id in id_to_vehicle}
    route_before = ""
    complete_order_itemIDs = []
    new_order_itemIDs = list(id_to_unlocated_items.keys())

    global solution_json_path
    if os.path.exists(solution_json_path) and os.path.getsize(solution_json_path) > 0:
        with open(solution_json_path, 'r') as file:
            previous_sol  = json.load(file)
            no = int(previous_sol.get('no', 0))
            f = (no + 1) * 10
            t = (no + 1) * 10 + 10
            global delta_t
            delta_t = f"{f:04d}-{t:04d}"
            
            new_order_itemIDs = [] # mảng các string 
            complete_order_itemIDs = [] # mảng các string
            route_before = previous_sol.get("route_after", "")
            
            last_on_vehicle_items = previous_sol.get("onvehicle_order_items", "").split(" ")
            for item in last_on_vehicle_items:
                if item not in id_to_ongoing_items.keys():
                    complete_order_itemIDs.append(item)
            
            last_unallocated_items = previous_sol.get("unallocated_order_items", "").split(" ")
            for item in id_to_unlocated_items.keys():
                if item not in last_unallocated_items:
                    new_order_itemIDs.append(item)

            route_before_split = route_before.split("V")
            
            for route in route_before_split[1:]:
                route = route.strip()
                str_len = len(route.split(":")[1])
                num_str = route.split(":")[0]
                vehicle_id = "V_" + num_str[1:]

                if str_len < 3:
                    vehicleid_to_plan[vehicle_id] = None
                    continue
                
                route_nodes_str = route.split(":")[1]
                route_nodes = route_nodes_str[1:len(route_nodes_str) - 1].split(" ")
                node_list = list(route_nodes)

                # Bỏ những Node đã giao thành công
                node_list = [node for node in node_list if not (node.startswith("d") and node.split("_")[1] in complete_order_itemIDs)]

                # Bỏ những Node đã nhận hàng
                node_list = [node for node in node_list if not (node.startswith("p") and node.split("_")[1] in id_to_ongoing_items.keys())]

                if node_list:
                    plan_route = []
                    for node_str in node_list:
                        delivery_items_list = []
                        pickup_items_list = []
                        d_p_items = None
                        op = node_str[0][0] # đúng 
                        opNumStr = node_str.split("_")
                        opItemNum = int(opNumStr[0][1:]) # đúng
                        orderItemId = node_str.split("_")[1] # đúng
                        idEndnum = int(orderItemId.split('-')[1])  # đúng
                        
                        if op == "d":
                            for _ in range(opItemNum):
                                d_p_items = id_to_allorder.get(orderItemId)
                                delivery_items_list.append(d_p_items)
                                orderItemId = orderItemId.split("-")[0] + "-" + str(idEndnum - 1 )
                                idEndnum -= 1
                        else:
                            for _ in range(opItemNum):
                                d_p_items = id_to_allorder.get(orderItemId)
                                pickup_items_list.append(d_p_items)
                                orderItemId = orderItemId.split("-")[0] + "-" + str(idEndnum + 1)
                                idEndnum += 1

                        if op == "d":
                            factory_id = d_p_items.delivery_factory_id
                        else:
                            factory_id = d_p_items.pickup_factory_id
                        factory = id_to_factory[factory_id]

                        # Tạo lại các node hợp lệ
                        node = Node(factory_id, delivery_items_list, pickup_items_list, factory.lng, factory.lat)
                        plan_route.append(node)

                    if plan_route:
                        vehicleid_to_plan[vehicle_id] = plan_route
    return new_order_itemIDs , complete_order_itemIDs """


def dispatch_new_orders(vehicleid_to_plan: Dict[str , list[Node]] ,  id_to_factory:Dict[str , Factory] , route_map: Dict[tuple , tuple] ,  id_to_vehicle: Dict[str , Vehicle] , id_to_unlocated_items:Dict[str , OrderItem], new_order_itemIDs: list[str]):
    if new_order_itemIDs:

        orderId_to_Item : Dict[str , list[OrderItem]] = {}
        for new_order_item in new_order_itemIDs:
            new_item = id_to_unlocated_items[new_order_item]
            orderID  = new_item.order_id
            if orderID not in orderId_to_Item:
                orderId_to_Item[orderID] = []
            orderId_to_Item[orderID].append(new_item)
        
        tmp = []
        for vehicle in id_to_vehicle.values():
            tmp.append (vehicle.board_capacity)
        
        capacity = int(np.mean(np.array(tmp)))
        
        for orderID , orderID_items in orderId_to_Item.items():
            order_demand = 0
            for item in orderID_items:
                order_demand += item.demand
            
            if order_demand > capacity:
                tmp_demand = 0
                tmp_itemList: list[OrderItem] = []
                for item in orderID_items:
                    if (tmp_demand + item.demand) > capacity:
                        node_list: list[Node] = create_Pickup_Delivery_nodes(copy.deepcopy(tmp_itemList) , id_to_factory)
                        isExhausive , bestInsertVehicleID, bestInsertPosI, bestInsertPosJ , bestNodeList = dispatch_nodePair(node_list , id_to_vehicle , vehicleid_to_plan , route_map)
                        
                        route_node_list = vehicleid_to_plan.get(bestInsertVehicleID)

                        if isExhausive:
                            route_node_list = bestNodeList[:]
                        else:
                            if route_node_list is None:
                                route_node_list = []
                            
                            new_order_pickup_node = node_list[0]
                            new_order_delivery_node = node_list[1]
                            
                            route_node_list.insert(bestInsertPosI, new_order_pickup_node)
                            route_node_list.insert(bestInsertPosJ, new_order_delivery_node)

                        vehicleid_to_plan[bestInsertVehicleID] = route_node_list
                        
                        tmp_itemList.clear()
                        tmp_demand = 0
                    tmp_itemList.append(item)
                    tmp_demand += item.demand 

                if len(tmp_itemList) > 0:
                    node_list: list[Node] = create_Pickup_Delivery_nodes(copy.deepcopy(tmp_itemList) , id_to_factory)
                    isExhausive , bestInsertVehicleID, bestInsertPosI, bestInsertPosJ , bestNodeList =  dispatch_nodePair(node_list , id_to_vehicle , vehicleid_to_plan, route_map)
                    
                    if isExhausive:
                        route_node_list = bestNodeList[:]
                    else:
                        if route_node_list is None:
                            route_node_list = []
                        
                        new_order_pickup_node = node_list[0]
                        new_order_delivery_node = node_list[1]
                        
                        route_node_list.insert(bestInsertPosI, new_order_pickup_node)
                        route_node_list.insert(bestInsertPosJ, new_order_delivery_node)

                    vehicleid_to_plan[bestInsertVehicleID] = route_node_list
            else:
                node_list: list[Node] = create_Pickup_Delivery_nodes(copy.deepcopy(tmp_itemList) , id_to_factory)
                isExhausive , bestInsertVehicleID, bestInsertPosI, bestInsertPosJ , bestNodeList = dispatch_nodePair(node_list , id_to_vehicle , vehicleid_to_plan , route_map)
                
                if isExhausive:
                    route_node_list = bestNodeList[:]
                else:
                    if route_node_list is None:
                        route_node_list = []
                    
                    new_order_pickup_node = node_list[0]
                    new_order_delivery_node = node_list[1]
                    
                    route_node_list.insert(bestInsertPosI, new_order_pickup_node)
                    route_node_list.insert(bestInsertPosJ, new_order_delivery_node)

                vehicleid_to_plan[bestInsertVehicleID] = route_node_list


def variable_neighbourhood_search(begintime: float , vehicleid_to_plan: Dict[str , list[Node]]  , route_map: Dict[tuple , tuple] ,  id_to_vehicle: Dict[str , Vehicle] ):
    global used_time
    n1, n2, n3, n4, n5 = 0, 0, 0, 0, 0
    endtime = time.time()
    used_time = endtime - begintime

    while True:
        if inter_couple_exchange(vehicleid_to_plan , id_to_vehicle , route_map):
            n1 += 1
            continue

        endtime = time.time()
        used_time = endtime - begintime
        if used_time > 9 * 60:
            print("TimeOut!!", file=sys.stderr)
            break

        if block_exchange(vehicleid_to_plan , id_to_vehicle , route_map):
            n2 += 1
            continue

        endtime = time.time()
        used_time = endtime - begintime
        if used_time > 9 * 60:
            print("TimeOut!!", file=sys.stderr)
            break

        if block_relocate(vehicleid_to_plan , id_to_vehicle , route_map):
            n3 += 1
            continue

        endtime = time.time()
        used_time = endtime - begintime
        if used_time > 9 * 60:
            print("TimeOut!!", file=sys.stderr)
            break

        if multi_pd_group_relocate(vehicleid_to_plan , id_to_vehicle , route_map):
            n4 += 1
        elif not improve_ci_path_by_2_opt(vehicleid_to_plan , id_to_vehicle , route_map , begintime):
            break
        else:
            n5 = 0

        endtime = time.time()
        used_time = endtime - begintime
        if used_time > 9 * 60:
            print("TimeOut!!", file=sys.stderr)
            break

    print(
        f"PDPairExchange:{n1}; BlockExchange:{n2}; BlockRelocate:{n3}; mPDG:{n4}; usedTime:{used_time:.2f} seconds; cost:{total_cost(id_to_vehicle , route_map , vehicleid_to_plan):.2f}",
        file=sys.stdout
    )


def get_route_after(vehicleid_to_plan: Dict[str , list[Node]], vehicleid_to_destination : Dict[str , Node]):
    route_str = ""
    vehicle_num = len(vehicleid_to_plan)
    vehicle_routes = [""] * vehicle_num
    index = 0
    
    if vehicleid_to_destination is None or len(vehicleid_to_destination) == 0:
        for i in range(vehicle_num):
            vehicle_routes[i] = "["
    for vehicle_id, first_node in vehicleid_to_destination.items():
        if first_node is not None:
            pickup_size = len(first_node.pickup_item_list) if first_node.pickup_item_list else 0
            delivery_size = len(first_node.delivery_item_list) if first_node.delivery_item_list else 0
            
            if delivery_size > 0:
                vehicle_routes[index] = f"[d{delivery_size}_{first_node.delivery_item_list[0].id} "
            if pickup_size > 0:
                if delivery_size == 0:
                    vehicle_routes[index] = f"[p{pickup_size}_{first_node.pickup_item_list[0].id} "
                else:
                    vehicle_routes[index] = vehicle_routes[index].strip()
                    vehicle_routes[index] += f"p{pickup_size}_{first_node.pickup_item_list[0].id} "
        else:
            vehicle_routes[index] = "["
        index += 1
    
    index = 0
    for vehicle_id, id2_node_list in vehicleid_to_plan.items():
        if id2_node_list and len(id2_node_list) > 0:
            for node in id2_node_list:
                pickup_size = len(node.pickup_item_list)
                delivery_size = len(node.delivery_item_list)
                
                if delivery_size > 0:
                    vehicle_routes[index] += f"d{delivery_size}_{node.delivery_item_list[0].id} "
                if pickup_size > 0:
                    if delivery_size > 0:
                        vehicle_routes[index] = vehicle_routes[index].strip()
                    vehicle_routes[index] += f"p{pickup_size}_{node.pickup_item_list[0].id} "
            
            vehicle_routes[index] = vehicle_routes[index].strip()
        vehicle_routes[index] += "]"
        index += 1

    for i in range(vehicle_num):
        car_id = f"V_{i + 1}"
        route_str += f"{car_id}:{vehicle_routes[i]} "
    
    route_str = route_str.strip()
    return route_str


def update_solution_json (id_to_ongoing_items: Dict[str , OrderItem] , id_to_unlocated_items: Dict[str , OrderItem] , id_to_vehicle: Dict[str , Vehicle] , vehicleid_to_plan: Dict[str , list[Node]] , vehicleid_to_destination : Dict[str , Node] , route_map: Dict[tuple , tuple]):
    global solution_json_path , input_directory ,delta_t , used_time
    order_items_json_path = os.path.join(input_directory, "solution.json")
    complete_order_items = ""
    on_vehicle_order_items = ""
    ongoing_order_items = ""
    unongoing_order_items = ""
    unallocated_order_items = ""
    new_order_items = ""
    route_before = ""
    route_after = ""

    on_vehicle_order_items = " ".join(id_to_ongoing_items.keys()).strip()

    # Process unallocatedOrderItems
    unallocated_order_items = " ".join(id_to_unlocated_items.keys()).strip()

    # Ensure the directory exists
    if not os.path.exists(input_directory):
        os.mkdir(input_directory)
    
    pre_matching_item_ids = []
    for vehicle in id_to_vehicle.values():
        if vehicle.carrying_items is None and vehicle.des is not None:
            pickup_item_list = vehicle.des.pickup_item_list
            pre_matching_item_ids.extend([order_item.id for order_item in pickup_item_list])
    ongoing_order_items = " ".join(pre_matching_item_ids).strip()

    # Process unongoingOrderItems
    unallocated_items = unallocated_order_items.split()
    unongoing_order_items = " ".join([item for item in unallocated_items if item not in pre_matching_item_ids]).strip()

    if not os.path.exists(order_items_json_path):
        delta_t = "0000-0010"
        vehicle_num = len(vehicleid_to_plan)
        for i in range(vehicle_num):
            car_id = f"V_{i + 1}"
            route_before += f"{car_id}:[] "
        route_before = route_before.strip()

        route_after = get_route_after(vehicleid_to_plan , vehicleid_to_destination)
        
        solution_json_obj = {
            "no.": "0",
            "deltaT": delta_t,
            "complete_order_items": complete_order_items,
            "onvehicle_order_items": on_vehicle_order_items,
            "ongoing_order_items": ongoing_order_items,
            "unongoing_order_items": unongoing_order_items,
            "unallocated_order_items": unallocated_order_items,
            "new_order_items": unallocated_order_items,
            "used_time": used_time,
            "finalCost": total_cost(id_to_vehicle , route_map , vehicleid_to_plan),
            "route_before": route_before,
            "route_after": route_after
        }
        
        with open(order_items_json_path, 'w') as file:
            json.dump(solution_json_obj, file, indent=4)
    else:
        with open(order_items_json_path, 'r') as file:
            before_solution = json.load(file)

        no = int(before_solution["no."]) + 1

        from_t = (no + 1) * 10
        to_t = (no + 1) * 10 + 10
        from_t_str = f"{from_t:04d}"
        to_t_str = f"{to_t:04d}"
        delta_t = f"{from_t_str}-{to_t_str}"

        complete_order_items_list = before_solution["onvehicle_order_items"].split()
        current_on_vehicle_items = on_vehicle_order_items.split()

        # Update completeOrderItems
        complete_order_items = " ".join([item for item in complete_order_items_list if item not in current_on_vehicle_items])

        unallocated_items_list = before_solution["unallocated_order_items"].split()
        current_unallocated_items = unallocated_order_items.split()

        # Update newOrderItems
        new_order_items = " ".join([item for item in current_unallocated_items if item not in unallocated_items_list])

        route_before = before_solution["route_after"]
        route_after = get_route_after(vehicleid_to_plan , vehicleid_to_destination)

        solution_json_obj = {
            "no.": str(no),
            "deltaT": delta_t,
            "complete_order_items": complete_order_items,
            "onvehicle_order_items": on_vehicle_order_items,
            "ongoing_order_items": ongoing_order_items,
            "unongoing_order_items": unongoing_order_items,
            "unallocated_order_items": unallocated_order_items,
            "new_order_items": new_order_items,
            "used_time": used_time,
            "finalCost": total_cost(id_to_vehicle , route_map , vehicleid_to_plan),
            "route_before": route_before,
            "route_after": route_after
        }

        with open(order_items_json_path, 'w') as file:
            json.dump(solution_json_obj, file, indent=4)


def get_output_solution(id_to_vehicle: Dict[str , Vehicle] , vehicleid_to_plan: Dict[str , list[Node]] , vehicleid_to_destination : Dict[str , Node]):
    for vehicleID , vehicle in id_to_vehicle.items():
        origin_plan : List[Node]= vehicleid_to_plan.get(vehicleID , [])
        planned_route : List[Node] = []
        destination : Node = None
        if vehicle.des:
            if not origin_plan:
                print(f"Planned route of vehicle {vehicleID} is wrong", file=sys.stderr)
            else:
                destination = origin_plan[0]
                destination.arrive_time = vehicle.des.arrive_time
                origin_plan.pop(0)
            
            if destination and vehicle.des.id != destination.id:
                print(f"Vehicle {vehicleID} returned destination id is {vehicle.des.id} "
                    f"however the origin destination id is {destination.id}", file=sys.stderr)
        elif (origin_plan):
            destination = origin_plan[0]
            origin_plan.pop(0)
        if origin_plan and len(origin_plan) == 0:
            origin_plan = None
        vehicleid_to_plan[vehicleID] = origin_plan
        vehicleid_to_destination[vehicleID] = destination


def over24hours(id_to_vehicle : Dict[str , Vehicle]):
    global newOrderItems
    now = datetime.now()
    midnight = datetime(now.year, now.month, now.day)

    initial_time = int(midnight.timestamp())  

    # Kiểm tra điều kiện
    if (id_to_vehicle.get("V_1").gps_update_time - 600 - 86400 >= initial_time and len(newOrderItems) == 0):
        return True
    return False


def redispatch_process(id_to_vehicle: Dict[str , Vehicle] , route_map: Dict[tuple , tuple] , vehicleid_to_plan: Dict[str , list[Node]] ,  id_to_factory:Dict[str , Factory] , id_to_unlocated_items:Dict[str , OrderItem] ):
    global newOrderItems
    cost0 = total_cost(id_to_vehicle , route_map , vehicleid_to_plan)
    order_item_ids = []
    backup_restore_solution : Dict[str,List[Node]]= {}
    for idx , (vehicleID , nodelist) in enumerate(vehicleid_to_plan.items()):
        vehicle = id_to_vehicle.get(vehicleID)
        route_size = len(nodelist) if nodelist else 0
        begin_pos = 1 if vehicle.des else 0
        
        if route_size == 0:
            backup_restore_solution[vehicleID] = None
            continue
        
        cp_nodelist = copy.deepcopy(nodelist)
        backup_restore_solution[vehicleID] = cp_nodelist
        
        i = begin_pos
        while i < len(nodelist):
            temp_node1 = nodelist[i]
            if temp_node1.pickup_item_list:
                begin_itemID = temp_node1.pickup_item_list[0].id
                
                for j in range(i +1 , route_size):
                    temp_node2 = nodelist[j]
                    if temp_node2.delivery_item_list and temp_node2.delivery_item_list[-1].id == begin_itemID:
                        order_item_ids.extend(orderitem.id for orderitem in temp_node1.pickup_item_list)
                        del nodelist[i]
                        del nodelist[j-  1]
                        i -=1
                        break
            i += 1

    if order_item_ids:
        order_item_ids.sort(key=lambda x: (re.split(r"-", x)[0], int(re.split(r"-", x)[1])))
        newOrderItems : str =" " + " ".join(order_item_ids)
        newOrderItems = newOrderItems.strip()
        
    new_order_itemIDs = newOrderItems.split(" ")
    dispatch_new_orders(vehicleid_to_plan , id_to_factory , route_map , id_to_vehicle , id_to_unlocated_items , new_order_itemIDs)
    
    cost1 = total_cost(id_to_vehicle , route_map , vehicleid_to_plan)
    if cost0 - 0.01 < cost1:
        vehicleid_to_plan = backup_restore_solution
    else:
        print(f"After 24h,redispatch valid.originCost:{cost0} newCost:{cost1}improve value: {(cost0 - cost1)}" , file= sys.stderr)

def main():
    global before_cost, delta_t, solution_json_path , completeOrderItems , newOrderItems , onVehicleOrderItems , unallocatedOrderItems , routeBefore , used_time

    id_to_factory , route_map ,  id_to_vehicle , id_to_unlocated_items ,  id_to_ongoing_items , id_to_allorder = Input()

    vehicleid_to_plan: Dict[str , List[Node]]= {}
    vehicleid_to_destination : Dict[str , Node] = {}
    
    begintime = time.time()

    new_order_itemIDs = restore_scene_with_single_node(vehicleid_to_plan , id_to_ongoing_items, id_to_unlocated_items  , id_to_vehicle , id_to_factory ,id_to_allorder)
    
    if over24hours():
        redispatch_process(id_to_vehicle , route_map , vehicleid_to_plan , id_to_factory , id_to_unlocated_items)
    else:
        dispatch_new_orders(vehicleid_to_plan , id_to_factory , route_map , id_to_vehicle , id_to_unlocated_items , new_order_itemIDs)
        
    variable_neighbourhood_search(begintime , vehicleid_to_plan , route_map , id_to_vehicle)
    
    merge_node(id_to_vehicle , vehicleid_to_plan)
    
    emergency_index =  Delaydispatch(id_to_vehicle , vehicleid_to_plan , route_map)
    
    get_output_solution(id_to_vehicle , vehicleid_to_plan , vehicleid_to_destination)
    
    update_solution_json(id_to_ongoing_items , id_to_unlocated_items , id_to_vehicle , vehicleid_to_plan , vehicleid_to_destination , route_map)
    
    write_destination_json_to_file_with_delay_timme(vehicleid_to_destination , emergency_index , id_to_vehicle , input_directory)
    
    write_route_json_to_file_with_delay_time(vehicleid_to_plan , emergency_index , id_to_vehicle , input_directory)
    
    end_time = time.time()
    used_time = (end_time - begintime)
    print ("SUCCESS !!!!!!!!!!!!!"  , file= sys.stdout)


if __name__ == '__main__':
    main()