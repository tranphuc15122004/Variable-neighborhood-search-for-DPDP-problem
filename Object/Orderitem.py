class OrderItem:
    def __init__(self, id, type, order_id, demand, pickup_factory_id, delivery_factory_id, creation_time, committed_completion_time, load_time, unload_time, delivery_state):
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