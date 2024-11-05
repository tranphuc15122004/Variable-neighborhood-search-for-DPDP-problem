from typing import List

class Destination:
    def __init__(self, factory_id: str, delivery_item_list: List[str], pickup_item_list: List[str],
                arrive_time: int, leave_time: int):
        self.factory_id = factory_id
        self.delivery_item_list = delivery_item_list
        self.pickup_item_list = pickup_item_list
        self.arrive_time = arrive_time
        self.leave_time = leave_time
