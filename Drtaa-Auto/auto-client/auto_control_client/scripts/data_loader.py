import json
from typing import Dict

def load_json(file_path: str) -> Dict:
    try:
        with open(file_path) as f:
            return json.load(f)
    except FileNotFoundError:
        return {"status": "fail", "message": "파일을 찾을 수 없습니다."}
    except json.JSONDecodeError:
        return {"status": "fail", "message": "파일 형식이 잘못되었습니다."}
