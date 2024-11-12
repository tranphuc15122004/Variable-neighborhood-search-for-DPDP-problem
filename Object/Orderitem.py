class OrderItem:
    def __init__(self, id: str, type: str, order_id: str, demand: float, pickup_factory_id: str, delivery_factory_id: str, creation_time, committed_completion_time, load_time: int, unload_time: int, delivery_state: int):
        self.id = id
        self.type = type
        self.order_id = order_id
        self.demand = demand  
        self.pickup_factory_id = pickup_factory_id
        self.delivery_factory_id = delivery_factory_id
        self.creation_time = creation_time
        self.committed_completion_time = committed_completion_time
        self.load_time = load_time
        self.unload_time = unload_time
        self.delivery_state = delivery_state

    def __str__(self):
        return f"OrderItem {self.id} from {self.pickup_factory_id} to {self.delivery_factory_id} , demand: {self.demand} , creation_time: {self.creation_time} , committed_completion_time: {self.committed_completion_time} delivery_state: {self.delivery_state}"