import json
import os
import string
from typing import Dict
from Read_input import Input
from Object import Factory, Node, Vehicle, VehicleInfo, OrderItem, Destination
import copy
import numpy as np

from local_search import create_Pickup_Delivery_nodes, dispatch_nodePair

solution_json_path = "./solution.json"
delta_t = "0000-0010"


## Cần có file JSON solution để debug lại
# CÓ vẻ đúng rồi
def Restore(vehicleid_to_plan: dict, id_to_ongoing_items: dict, id_to_unlocated_items: dict, id_to_vehicle: dict , id_to_factory:dict, id_to_allorder: dict):
    vehicleid_to_plan = {vehicle_id: None for vehicle_id in id_to_vehicle}
    route_before = ""
    complete_order_itemIDs = []
    new_order_itemIDs = list(id_to_unlocated_items.keys())
    

    global solution_json_path
    if os.path.exists(solution_json_path) and os.path.getsize(solution_json_path) > 0:
        with open(solution_json_path, 'r') as file:
            previous_sol = json.load(file)
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
    return new_order_itemIDs , complete_order_itemIDs


def dispatch_new_orders(vehicleid_to_plan: Dict[str , list[Node]] ,  id_to_factory:dict , route_map: dict ,  id_to_vehicle: Dict[str , Vehicle] , id_to_unlocated_items:Dict[str , OrderItem] ,  id_to_ongoing_items: dict , id_to_allorder:dict , new_order_itemIDs: list[str]):
    if new_order_itemIDs:
        # Mapping orderID to its list of items 
        # str - list[OrderItem]
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
                    if tmp_demand + item.demand > capacity:
                        node_list: list[Node] = create_Pickup_Delivery_nodes(copy.deepcopy(tmp_itemList) , id_to_factory)
                        dispatch_nodePair(node_list , id_to_vehicle , vehicleid_to_plan)
                        
                        tmp_itemList.clear()
                        tmp_demand = 0
                    tmp_itemList.append(item)
                    tmp_demand += item.demand


def main():
    # Tất cả đều là Dict
    id_to_factory , route_map ,  id_to_vehicle , id_to_unlocated_items ,  id_to_ongoing_items , id_to_allorder = Input()
    # Dict
    vehicleid_to_plan: Dict[str , list[Node]]= {}
    vehicleid_to_destination = {}
    
    # Mảng các string
    new_order_itemIDs , complete_order_itemIDs = Restore(vehicleid_to_plan , id_to_ongoing_items, id_to_unlocated_items  , id_to_vehicle , id_to_factory ,id_to_allorder)
    
    dispatch_new_orders(vehicleid_to_plan= vehicleid_to_plan , id_to_factory= id_to_factory, route_map= route_map , id_to_vehicle= id_to_vehicle,
                        id_to_unlocated_items= id_to_unlocated_items , id_to_ongoing_items= id_to_ongoing_items , id_to_allorder= id_to_allorder , new_order_itemIDs= new_order_itemIDs)


if __name__ == '__main__':
    main()