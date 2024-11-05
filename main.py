import json
import os
from Read_input import Input
from Object import Factory, Node, Vehicle, VehicleInfo, OrderItem, Destination
import copy

solution_json_path = "./solution.json"
delta_t = "0000-0010"

def Restore(vehicleid_to_plan, id_to_ongoing_items, id_to_unlocated_items, id_to_vehicle, id_to_factory, id_to_allorder):
    vehicleid_to_plan = {vehicle_id: None for vehicle_id in id_to_vehicle}
    new_order_items = list(id_to_unlocated_items.keys())
    route_before = ""

    global solution_json_path
    if os.path.exists(solution_json_path) and os.path.getsize(solution_json_path) > 0:
        with open(solution_json_path, 'r') as file:
            previous_sol = json.load(file)
            no = int(previous_sol.get('no', 0))
            previous_cost = previous_sol.get('finalCost', 0)
            f = (no + 1) * 10
            t = (no + 1) * 10 + 10
            global delta_t
            delta_t = f"{f:04d}-{t:04d}"
            
            completeOrderItems = []
            route_before = previous_sol.get("route_after", "")
            
            last_on_vehicle_items = previous_sol.get("onvehicle_order_items", "").split(" ")
            for item in last_on_vehicle_items:
                if item not in id_to_ongoing_items:
                    completeOrderItems.append(item)
            
            last_unallocated_items = previous_sol.get("unallocated_order_items", "").split(" ")
            for item in id_to_unlocated_items.keys():
                if item not in last_unallocated_items:
                    new_order_items.append(item)

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
                node_list = [node for node in node_list if not (node.startswith("d") and node.split("_")[1] in completeOrderItems)]

                # Bỏ những Node đã nhận hàng
                node_list = [node for node in node_list if not (node.startswith("p") and node.split("_")[1] in id_to_ongoing_items.keys())]

                if node_list:
                    plan_route = []
                    for node_str in node_list:
                        delivery_items_list = []
                        pickup_items_list = []
                        d_p_items = None
                        op = node_str[0]
                        opNumStr = node_str.split("_")
                        opItemNum = int(opNumStr[0][1:])
                        orderItemId = node_str.split("_")[1]
                        idEndnum = int(orderItemId.split('-')[1])
                        
                        if op == "d":
                            for _ in range(opItemNum):
                                d_p_items = id_to_allorder.get(orderItemId)
                                delivery_items_list.append(d_p_items)
                                orderItemId = orderItemId.split("-")[0] + "-" + str(idEndnum - 1)
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




def main():
    id_to_factory , route_map ,  id_to_vehicle , id_to_unlocated_items ,  id_to_ongoing_items , id_to_allorder = Input()
    
    vehicleid_to_plan= {}
    vehicleid_to_destination = {}
    Restore(vehicleid_to_plan , copy.deepcopy(id_to_ongoing_items), copy.deepcopy(id_to_unlocated_items)  , copy.deepcopy(id_to_vehicle) , copy.deepcopy(id_to_factory) , copy.deepcopy(id_to_allorder))
    for vehicle_id, plan in vehicleid_to_plan.items():
        print(f"Vehicle ID: {vehicle_id}")
        if plan:
            for node in plan:
                print(f"  Node: {node.factory_id}, Delivery Items: {node.delivery_items}, Pickup Items: {node.pickup_items}")
        else:
            print("  No plan available")


if __name__ == '__main__':
    main()