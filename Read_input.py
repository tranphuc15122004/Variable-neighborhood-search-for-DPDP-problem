import csv
import json
import string
import sys
from typing import Dict , List , Tuple
from Object import Factory, Node, Vehicle, VehicleInfo, OrderItem, Destination
import traceback
import copy


def read_input_Factory_CSV() -> Dict[str , Factory]:
    file_path = 'benchmark/factory.csv'
    id2factory_map: Dict[str, Factory] = {}
    try:
        with open(file_path, "r", encoding="utf-8") as file:
            next(file)  # Bỏ qua dòng đầu tiên
            for line in file:
                item = line.strip().split(",")
                factory_id = item[0]
                lng = float(item[1])
                lat = float(item[2])
                dock_num = int(item[3])
                factory = Factory(factory_id, lng, lat, dock_num)
                if factory_id not in id2factory_map:
                    id2factory_map[factory_id] = factory
    except Exception as e:
        print(f"Error: {e}")
    return id2factory_map


def read_input_Routemap_CSV() -> Dict[Tuple[str , str] , Tuple[str , str]]:
    file_path = 'benchmark/route.csv'
    Route_map : Dict[Tuple[str , str] , Tuple[str , str]] = {}
    try:
        with open(file_path , mode= 'r' , encoding= 'utf-8-sig') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                u : str = row[1]
                v :str = row[2]
                if (u , v) not in Route_map:
                    dis : str = row[3]
                    t : str = row[4]
                    Route_map[(u, v)] = (dis, t)
    except Exception as e:
        print (f"Error: {e}" , file= sys.stderr)
    return Route_map

def read_csv() -> Tuple[Dict[str, Factory], Dict[Tuple[str , str], Tuple[str , str]]]:
    return read_input_Factory_CSV() , read_input_Routemap_CSV()

def read_json() -> Tuple[Dict[str , Vehicle], Dict[str , OrderItem], Dict[str , OrderItem], Dict[str , OrderItem]]:
    id_to_unlocated_item = read_unlocated_item()
    id_to_ongoing_item = read_ongoing_item()
    id_allorder : Dict[str , OrderItem] = {}
    id_allorder.update(id_to_unlocated_item)
    id_allorder.update(id_to_ongoing_item)
    id_to_vehicle = read_vehicleinfor(id_allorder)
    return id_to_vehicle , id_to_unlocated_item ,  id_to_ongoing_item , id_allorder

def read_unlocated_item() -> Dict[str , OrderItem]:
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
        
def read_ongoing_item() -> Dict[str , OrderItem]:
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

def read_vehicleinfor(id_allorder: Dict[str ,OrderItem]) -> Dict[str , Vehicle]:
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

def Input() ->Tuple[Dict[str, Factory], Dict[Tuple[str , str], Tuple[str , str]] ,Dict[str , Vehicle], Dict[str , OrderItem], Dict[str , OrderItem], Dict[str , OrderItem] ]:
        id_to_factory , route_map =  read_csv()
        id_to_vehicle , id_to_unlocated_item ,  id_to_ongoing_item  , id_allorder = read_json()
        return id_to_factory , route_map ,  id_to_vehicle , id_to_unlocated_item ,  id_to_ongoing_item , id_allorder
