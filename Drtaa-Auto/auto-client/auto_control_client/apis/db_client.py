import pymysql
from typing import List, Dict, Any

class DBClient:
    def __init__(self, host: str, port: int, user: str, password: str, database: str):
        self.connection = None

        self.host=host
        self.port=port
        self.user=user
        self.password=password
        self.database=database

    def connect(self):
        self.connection = pymysql.connect(
            host=self.host,
            port=self.port,
            user=self.user,
            password=self.password,
            database=self.database,
            charset='utf8mb4',
            cursorclass=pymysql.cursors.DictCursor
        )

    def close(self):
        if self.connection:
            self.connection.close()

    def execute_query(self, sql: str, params: tuple = None) -> List[Dict[str, Any]]:
        try:
            with self.connection.cursor() as cursor:
                cursor.execute(sql, params)
                return cursor.fetchall()
        except pymysql.Error as e:
            print(f"쿼리 실행 중 에러 발생: {e}")
            return []

    def execute_update(self, sql: str, params: tuple = None) -> bool:
        try:
            with self.connection.cursor() as cursor:
                cursor.execute(sql, params)
            self.connection.commit()
            return True
        except pymysql.Error as e:
            print(f"업데이트 중 에러 발생: {e}")
            self.connection.rollback()
            return False


    def update_rent_car_status(self, car_id: int, status: str) -> bool:
        self.connect()
        sql = "UPDATE rent_car SET RENT_CAR_DRIVING_STATUS = %s WHERE RENT_CAR_ID = %s"
        response = self.execute_update(sql, (status, car_id))
        self.close()
        return response
    
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
