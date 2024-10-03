# gps_converter.py

import numpy as np
from pyproj import CRS, Transformer
from typing import List, Tuple

class GPSConverter:
    def __init__(self, utm_zone: int, east_offset: float, north_offset: float):
        self.utm_zone = utm_zone
        self.east_offset = east_offset
        self.north_offset = north_offset
        self.crs_utm = CRS(proj='utm', zone=self.utm_zone, ellps='WGS84')
        self.transformer = Transformer.from_crs(self.crs_utm, "EPSG:4326", always_xy=True)
        self.transformer_inv = Transformer.from_crs("EPSG:4326", self.crs_utm)
        

    def calc_gps_from_pose(self, x: float, y: float) -> Tuple[float, float]:
        """
        로컬 좌표계를 GPS 좌표로 일괄 변환합니다.

        :param coordinates: (x, y) 좌표 리스트
        :return: (latitude, longitude) GPS 좌표 리스트
        """
        utm_x = x + self.east_offset
        utm_y = y + self.north_offset

        longitudes, latitudes = self.transformer.transform(utm_x, utm_y)
        return latitudes, longitudes

    def calc_gps_from_pose_batch(self, coordinates: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        로컬 좌표계를 GPS 좌표로 일괄 변환합니다.

        :param coordinates: (x, y) 좌표 리스트
        :return: (latitude, longitude) GPS 좌표 리스트
        """
        x, y = np.array(coordinates).T
        utm_x = x + self.east_offset
        utm_y = y + self.north_offset

        longitudes, latitudes = self.transformer.transform(utm_x, utm_y)
        return list(zip(latitudes, longitudes))
    
    def calc_pose_from_gps(self, latitude: float, longitude: float) -> Tuple[float, float]:
        """
        GPS 좌표를 로컬 좌표계로 변환합니다.

        :param latitude: 위도
        :param longitude: 경도
        :return: (x, y) 로컬 좌표
        """
        xy_zone = self.transformer_inv.transform(latitude, longitude)
        
        x = xy_zone[0] - self.east_offset
        y = xy_zone[1] - self.north_offset

        return x, y
