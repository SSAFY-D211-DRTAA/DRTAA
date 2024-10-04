from .api_client import APIClient

class RentCarAPI:
    def __init__(self, base_url, token):
        self.api_client = APIClient(base_url, token)

    def send_alarm(self, rent_car_id, contents):
        payload = {
            "rentCarId": rent_car_id,
            "contents": contents
        }
        return self.api_client.post("/rent-car/alarm", payload)

    def send_arrival_info(self, rent_car_id, contents):
        payload = {
            "rentCarId": rent_car_id,
            "contents": contents,
        }
        return self.api_client.post("/rent-car/arrival", payload)
