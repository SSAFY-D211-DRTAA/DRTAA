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
