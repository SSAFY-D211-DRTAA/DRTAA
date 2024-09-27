import logging
import os

from datetime import time

# from logging.handlers import RotatingFileHandler
from logging.handlers import TimedRotatingFileHandler

class ColoredFormatter(logging.Formatter):
    COLORS = {
        'DEBUG': '\033[94m',  # 파란색
        'INFO': '\033[92m',   # 초록색
        'WARNING': '\033[93m',  # 노란색
        'ERROR': '\033[91m',  # 빨간색
        'CRITICAL': '\033[95m',  # 보라색
        'RESET': '\033[0m'  # 색상 리셋
    }

#region 색상 전체 적용 포맷
    # def format(self, record):
    #     log_message = super().format(record)
    #     return f"{self.COLORS.get(record.levelname, self.COLORS['RESET'])}{log_message}{self.COLORS['RESET']}"
#endregion

    def format(self, record):
        # 원본 로그 메시지 포맷
        log_message = super().format(record)
        
        # 로그 레벨에 해당하는 색상 코드 가져오기
        level_color = self.COLORS.get(record.levelname, self.COLORS['RESET'])
        
        # 로그 메시지에서 레벨 부분만 찾아 색상 적용
        colored_level = f"{level_color}{record.levelname}{self.COLORS['RESET']}"
        return log_message.replace(record.levelname, colored_level)


def setup_logger(name, log_file='main.log', level=logging.INFO):
    """로거를 설정하고 반환하는 함수"""
    
    # 로거 생성
    logger = logging.getLogger(name)
    logger.setLevel(level)

#region 기존 로그 포맷
    # # 로그 포맷 정의
    # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    # # 스트림 핸들러 (콘솔 출력)
    # stream_handler = logging.StreamHandler()
    # stream_handler.setFormatter(formatter)
    # logger.addHandler(stream_handler)
#endregion

    handler = logging.StreamHandler()
    formatter = ColoredFormatter(
        "%(asctime)s.%(msecs)03d | %(levelname)s | %(filename)s:%(lineno)d | %(process)d >>> %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    # 파일 핸들러
    log_dir = 'logs'
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # 특정 시간 설정
    rotation_time = time(hour=0, minute=0, second=0)
    
    # file_handler = RotatingFileHandler(os.path.join(log_dir, log_file), maxBytes=10*1024*1024, backupCount=5)
    file_handler = TimedRotatingFileHandler(
        os.path.join(log_dir, log_file),
        when='midnight',  # 매일 자정에 새 파일 생성
        interval=1,  # 1일 간격
        backupCount=30,  # 최대 30일치 로그 파일 유지
        encoding='utf-8',
        atTime=rotation_time  # 특정 시간 지정
    )
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    return logger

# 전역 로거 설정 (선택적)
global_logger = setup_logger('global')

def get_logger(name):
    """이미 설정된 로거를 반환하거나 새 로거를 설정하는 함수"""
    if name in logging.Logger.manager.loggerDict:
        return logging.getLogger(name)
    else:
        return setup_logger(name)
