import requests
import logging

class APIClient:
    def __init__(self, base_url, auth_token):
        self.base_url = base_url
        self.auth_token = auth_token
        self.headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.auth_token}"
        }

    def post(self, endpoint, data):
        url = f"{self.base_url}{endpoint}"
        try:
            response = requests.post(url, json=data, headers=self.headers)
            response.raise_for_status()

            return response
        except requests.RequestException as e:
            logging.error(f"API 요청 중 오류 발생: {e}")
            return None