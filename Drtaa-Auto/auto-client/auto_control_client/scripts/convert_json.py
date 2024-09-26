import json

def convert_points_to_json(points):
    """
    좌표 리스트를 JSON 문자열로 변환합니다.
    
    :param points: (lat, lon) 튜플의 리스트
    :return: JSON 문자열
    """
    data = {
        "tag": "global_path",
        "msg": {
            "path": [{"idx": i, "lat": lat, "lon": lon} for i, (lat, lon) in enumerate(points)]
        }
    }
    return json.dumps(data)
