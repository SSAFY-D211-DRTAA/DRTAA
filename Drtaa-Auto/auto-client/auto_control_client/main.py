import websocket
import json
import logging
import signal
import sys
import time

from websocket import WebSocketApp
from typing import Tuple, Dict, Any, Union
from pyproj import CRS, Transformer

logging.debug(websocket.WebSocketApp)

# 상수 정의
CONFIG_FILE_PATH = 'config.json'
GPS_TOPIC = "/gps"
COMPLETE_DRIVE_TOPIC = "/complete_drive"
MOVE_BASE_GOAL_TOPIC = "/move_base_simple/goal"


# 설정 파일 로드
def load_config() -> Dict[str, Union[str, float, int]]:
    try:
        with open(CONFIG_FILE_PATH) as f:
            return json.load(f)
    except FileNotFoundError:
        logging.error("설정 파일을 찾을 수 없습니다.")
        sys.exit(1)
    except json.JSONDecodeError:
        logging.error("설정 파일 형식이 잘못되었습니다.")
        sys.exit(1)

def validate_config(config: Dict[str, Any]) -> None:
    required_keys = ['websocket_url', 'utm_zone', 'east_offset', 'north_offset', 'next_goal_lat', 'next_goal_lon']
    for key in required_keys:
        if key not in config:
            raise ValueError(f"설정 파일에 '{key}' 값이 없습니다.")

config = load_config()
validate_config(config)

# 로깅 설정
logging.basicConfig(level=config.get('log_level', 'INFO'), format='%(asctime)s - %(levelname)s - %(message)s')

def calc_pose_from_gps(latitude: float, longitude: float) -> Tuple[float, float]:
    """
    GPS 좌표를 로컬 좌표계로 변환합니다.

    :param latitude: 위도
    :param longitude: 경도
    :return: (x, y) 로컬 좌표
    """
    crs_utm = CRS(proj='utm', zone=config['utm_zone'], ellps='WGS84')
    transformer = Transformer.from_crs("EPSG:4326", crs_utm)

    xy_zone = transformer.transform(latitude, longitude)

    x = xy_zone[0] - config['east_offset']
    y = xy_zone[1] - config['north_offset']

    return x, y

def publish_pose_from_gps(ws: WebSocketApp, lat: float, lon: float) -> None:
    """
    GPS 좌표를 이용하여 로봇의 목표 위치를 발행합니다.

    :param ws: WebSocket 연결
    :param lat: 위도
    :param lon: 경도
    """
    current_time = time.time()
    secs = int(current_time)
    nsecs = int((current_time - secs) * 1e9)

    x, y = calc_pose_from_gps(lat, lon)

    msg = {
        "op": "publish",
        "topic": MOVE_BASE_GOAL_TOPIC,
        "type": "geometry_msgs/PoseStamped",
        "msg": {
            "header": {
                "frame_id": "map",
                "stamp": {
                    "secs": secs,
                    "nsecs": nsecs
                }
            },
            "pose": {
                "position": {
                    "x": x,
                    "y": y,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1.0
                }
            }
        }
    }
    ws.send(json.dumps(msg))

def subscribe(ws: WebSocketApp, topic: str, type: str) -> None:
    """
    지정된 토픽을 구독합니다.

    :param ws: WebSocket 연결
    :param topic: 구독할 토픽
    :param type: 메시지 타입
    """
    msg = {
        "op": "subscribe",
        "topic": topic,
        "type": type
    }
    ws.send(json.dumps(msg))

def unsubscribe(ws: WebSocketApp, topic: str) -> None:
    """
    지정된 토픽의 구독을 해제합니다.

    :param ws: WebSocket 연결
    :param topic: 구독 해제할 토픽
    """
    msg = {
        "op": "unsubscribe",
        "topic": topic,
    }
    ws.send(json.dumps(msg))

def on_message(ws: WebSocketApp, message: str) -> None:
    try:
        data: Dict[str, Any] = json.loads(message)
        if data['op'] == 'publish':
            if data['topic'] == GPS_TOPIC:
                """
                {
                    "op": "publish",
                    "topic": "/gps",
                    "msg": {
                        "header": {
                            "seq": 2719,
                            "stamp": {
                                "secs": 1726724743, 
                                "nsecs": 968000000
                                },
                            "frame_id": "gps"
                        },
                        "latitude": 37.58258292262119,
                        "longitude": 126.88956050473249,
                        "altitude": 36.98945840126462,
                        "eastOffset": 313008.55819800857,
                        "northOffset": 4161698.628368007,
                        "status": 1
                    }
                }
                """

                gps_data = data['msg']

                # GPS 데이터를 파일에 저장
                try:
                    with open('gps_data.json', 'w') as f:
                        json.dump(gps_data, f)
                except IOError as e:
                    logging.error(f"GPS 데이터를 파일에 저장하는 중 오류 발생: {e}")

            elif data['topic'] == COMPLETE_DRIVE_TOPIC:
                logging.info("도착지에 도착했습니다.")
                """
                {
                    "op": "publish",
                    "topic": "/complete_drive",
                    "msg": {
                        "data": true
                    }
                }
                """
                
                # GPS 데이터를 파일에 저장
                try:
                    with open('complete_drive_data.json', 'w') as f:
                        json.dump(data, f)
                except IOError as e:
                    logging.error(f"완료 데이터를 파일에 저장하는 중 오류 발생: {e}")

                publish_pose_from_gps(ws, config['next_goal_lat'], config['next_goal_lon'])
    except json.JSONDecodeError:
        logging.error("잘못된 JSON 형식의 메시지를 받았습니다.")
    except KeyError as e:
        logging.error(f"메시지에서 필요한 키를 찾을 수 없습니다: {e}")
    except Exception as e:
        logging.error(f"메시지 처리 중 오류 발생: {e}")

def on_error(ws: WebSocketApp, error: Exception) -> None:
    logging.error(f"에러 발생: {error}")

def on_close(ws: WebSocketApp, close_status_code: int, close_msg: str) -> None:
    logging.info("WebSocket 연결 종료")

def on_open(ws: WebSocketApp) -> None:
    logging.info("WebSocket 연결 성공")

    subscribe(ws, "/gps", "morai_msgs/GPSMessage")
    subscribe(ws, "/complete_drive", "std_msgs/Bool")

    # publish_pose_from_gps(ws, config['test_lat'], config['test_lon'])

def signal_handler(sig: int, frame: Any) -> None:
    logging.info("프로그램 종료 중...")

    unsubscribe(ws, GPS_TOPIC)
    unsubscribe(ws, COMPLETE_DRIVE_TOPIC)

    time.sleep(0.01)

    ws.close()

    # WebSocket 연결이 종료될 때까지 대기
    timeout = time.time() + 5  # 5초 타임아웃
    while ws.keep_running and time.time() < timeout:
        time.sleep(0.1)

    if ws.keep_running:
        logging.warning("WebSocket 연결을 강제로 종료합니다.")
        ws.keep_running = False

    sys.exit(0)

if __name__ == "__main__":
    websocket.enableTrace(False)
    logging.info(f"Target {config['websocket_url']}")

    ws: WebSocketApp = WebSocketApp(config['websocket_url'],
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)

    signal.signal(signal.SIGINT, signal_handler)
    ws.run_forever()
