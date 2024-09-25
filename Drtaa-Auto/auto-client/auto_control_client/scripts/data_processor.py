from typing import Dict, List, Tuple
from data_loader import load_json
from curve_simplification import simplify_curve

def process_data(file_path: str, epsilon: float) -> Dict:
    global_path_data = load_json(file_path)
    original_points = [(pose["pose"]["position"]["x"], pose["pose"]["position"]["y"]) for pose in global_path_data["poses"]]
    optimized_path = simplify_curve(original_points, epsilon)
    
    # print(f"{file_path}:")
    # print(f"  원본 데이터 포인트 수: {len(original_points)}")
    # print(f"  최적화된 데이터 포인트 수: {len(optimized_path)}")
    
    return {
        'original': original_points,
        'optimized': optimized_path
    }
