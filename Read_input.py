import csv
import json
import string
from typing import Dict
from Object import Factory, Node, Vehicle, VehicleInfo, OrderItem, Destination
import traceback
import copy


def read_input_Factory_CSV():
    file_path = 'benchmark/factory.csv'
    id_to_vehicle = {}
    with open(file_path, mode='r', encoding='utf-8-sig') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            factory_id = row[0]
            lng = float(row[1])
            lat = float(row[2])
            dock_num = int(row[3])
            id_to_vehicle[factory_id] = Factory(factory_id, lng, lat, dock_num)
    return id_to_vehicle  


def read_input_Routemap_CSV():
    file_path = 'benchmark/route.csv'
    Route_map = {}
    with open(file_path , mode= 'r' , encoding= 'utf-8-sig') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            u = row[1]
            v = row[2]
            dis = row[3]
            t = row[4]
            Route_map[(u, v)] = (dis, t)
    return Route_map

def read_csv() :
    return read_input_Factory_CSV() , read_input_Routemap_CSV()

def read_json():
    id_to_unlocated_item = read_unlocated_item()
    id_to_ongoing_item = read_ongoing_item()
    id_allorder = {}
    id_allorder.update(id_to_unlocated_item)
    id_allorder.update(id_to_ongoing_item)
    id_to_vehicle = read_vehicleinfor(id_allorder)
    return id_to_vehicle , id_to_unlocated_item ,  id_to_ongoing_item , id_allorder

def read_unlocated_item() -> Dict[string , OrderItem]:
    path = 'benchmark/unlocated_item.json'
    id_to_unlocated_item = {}
    with open(f"./{path}" , mode= 'r' , encoding= 'utf-8-sig') as file:
        data = json.load(file)
        for item_json in data:
            id = item_json.get("id")
            type = item_json.get("type")
            order_id = item_json.get("order_id")
            pickup_factory_id = item_json.get("pickup_factory_id")
            delivery_factory_id = item_json.get("delivery_factory_id")
            creation_time = int(item_json.get("creation_time"))
            committed_completion_time = int(item_json.get("committed_completion_time"))
            load_time = int(item_json.get("load_time"))
            unload_time = int(item_json.get("unload_time"))
            delivery_state = int(item_json.get("delivery_state"))
            demand = float(item_json.get("demand"))
            
            temp = OrderItem(id=id , type= type , order_id= order_id , pickup_factory_id= pickup_factory_id , delivery_factory_id= delivery_factory_id , creation_time= creation_time , committed_completion_time= committed_completion_time , load_time= load_time , unload_time= unload_time , delivery_state= delivery_state , demand= demand)
            
            id_to_unlocated_item[id] = temp
    return id_to_unlocated_item
        
def read_ongoing_item() -> Dict[string , OrderItem]:
    path = 'benchmark/ongoing_item.json'
    id_to_ongoing_item = {}
    with open(f"./{path}" , mode= 'r' , encoding= 'utf-8-sig') as file:
        data = json.load(file)
        for item_json in data:
            id  = item_json.get("id")
            type = item_json.get("type")
            order_id = item_json.get("order_id")
            pickup_factory_id = item_json.get("pickup_factory_id")
            delivery_factory_id = item_json.get("delivery_factory_id")
            creation_time = int(item_json.get("creation_time"))
            committed_completion_time = int(item_json.get("committed_completion_time"))
            load_time = int(item_json.get("load_time"))
            unload_time = int(item_json.get("unload_time"))
            delivery_state = int(item_json.get("delivery_state"))
            demand = float(item_json.get("demand"))
            temp = OrderItem(id=id , type= type , order_id= order_id , pickup_factory_id= pickup_factory_id , delivery_factory_id= delivery_factory_id , creation_time= creation_time , committed_completion_time= committed_completion_time , load_time= load_time , unload_time= unload_time , delivery_state= delivery_state , demand= demand)
            id_to_ongoing_item[id] = temp
    return id_to_ongoing_item

def read_vehicleinfor(id_allorder: Dict[string ,OrderItem]) -> Dict[string , Vehicle]:
    path  = 'benchmark/vehicle.json'
    id_to_vehicle = {}
    with open(f"./{path}" , mode= 'r' , encoding= 'utf-8-sig') as file:
        data = json.load(file)
        for v_json in data:
            id = v_json.get("id")
            gps_id = v_json.get("gps_id")
            cur_factory_id = v_json.get("cur_factory_id")
            operation_time = int(v_json.get("operation_time"))
            capacity = float(v_json.get("capacity"))
            update_time = int(v_json.get("update_time"))
            arrive_time_at_current_factory = int(v_json.get("arrive_time_at_current_factory"))
            leave_time_at_current_factory = int(v_json.get("leave_time_at_current_factory"))
            
            carrying_items_json = v_json.get("carrying_items", [])
            carrying_items_list = [item for item in carrying_items_json] 
            carrying_items = []
            for item_id in carrying_items_list:
                if item_id in id_allorder:
                    carrying_items.append(id_allorder[item_id])

            destination_json = v_json.get("des")
            if destination_json is not None:
                factory_id = destination_json.get("factory_id")
                arrive_time = int(destination_json.get("arrive_time"))
                leave_time = int(destination_json.get("leave_time"))

                # Thêm thông tin về các ITEM trả tại điểm đến
                pickup_items_json = destination_json.get("pickup_item_list", [])
                pickup_items_list = [item for item in pickup_items_json]
                pickup_items = []
                for item_id in pickup_items_list:
                    if item_id in id_allorder:
                        pickup_items.append(id_allorder[item_id])

                # Thêm các ITEM nhận tại điểm đến
                delivery_items_json = destination_json.get("delivery_item_list", [])
                delivery_items_list = [item for item in delivery_items_json]
                delivery_items = []
                for item_id in delivery_items_list:
                    if item_id in id_allorder:
                        delivery_items.append(id_allorder[item_id])

                des = Node(factory_id, delivery_items  ,pickup_items , arrive_time, leave_time)
            temp = Vehicle(id=id, gps_id=gps_id, operation_time=operation_time, board_capacity=capacity, carrying_items=carrying_items, des=des)
            temp.set_cur_position_info(cur_factory_id , update_time , arrive_time_at_current_factory , leave_time_at_current_factory)
            id_to_vehicle[id] = temp
    return id_to_vehicle

def Input():
        id_to_factory , route_map =  read_csv()
        id_to_vehicle , id_to_unlocated_item ,  id_to_ongoing_item  , id_allorder = read_json()
        return id_to_factory , route_map ,  id_to_vehicle , id_to_unlocated_item ,  id_to_ongoing_item , id_allorder
