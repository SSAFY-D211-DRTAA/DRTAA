import json
import logging
import sys
from typing import Dict, Union

CONFIG_FILE_PATH = 'config.json'

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
