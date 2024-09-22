import json
from utils.logger import setup_logger

logger = setup_logger(__name__)

class MessageHandler:
    def __init__(self):
        # 여기에 차량 관리자 등 필요한 객체를 초기화합니다.
        pass

    def handle_message(self, message, source):
        try:
            data = json.loads(message)
            action = data.get('action')
            response = self.process_action(action, data)
            logger.info(f"Processed message from {source}: {action}")
            return response
        except json.JSONDecodeError:
            logger.error(f"Invalid JSON received from {source}")
            return {"error": "Invalid JSON"}
    
    def process_action(self, action, data):
        if action == 'Spring Boot Connect':
            return self.handle_connect(data)
        elif action == 'vehicle_info':
            return self.handle_vehicle_info(data)
        elif action == 'vehicle_dispatch':
            return self.handle_vehicle_dispatch(data)
        elif action == 'vehicle_return':
            return self.handle_vehicle_return(data)
        elif action == 'vehicle_drive':
            return self.handle_vehicle_drive(data)
        else:
            logger.warning(f"Unknown action received: {action}")
            return {"error": "Unknown action"}

    def handle_connect(self, data):
        return {"status": "success", "echo": data}

    def handle_vehicle_info(self, data):
        # 차량 정보 조회 로직
        return {"status": "success", "info": "Vehicle info placeholder"}

    def handle_vehicle_dispatch(self, data):
        # 차량 호출 로직
        return {"status": "success", "message": "Vehicle dispatched", "rentCarLat": 30.576760, "rentCarLon": 100.898863}

    def handle_vehicle_return(self, data):
        # 차량 반납 로직
        return {"status": "success", "message": "Vehicle returned"}

    def handle_vehicle_drive(self, data):
        # 차량 주행 로직
        return {"status": "success", "message": "Vehicle driving"}
