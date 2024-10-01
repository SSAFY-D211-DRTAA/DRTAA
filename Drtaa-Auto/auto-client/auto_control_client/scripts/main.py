from config import load_config
from data_processor import process_data
from plotter import plot_coordinates
from gps_converter import GPSConverter
from convert_json import convert_points_to_json

def main():
    epsilon = 0.5  # 이 값을 조절하여 단순화 정도를 제어할 수 있습니다

    file_paths = [
        './path/global_path_data.json',
        './path/global_path_data (1).json',
        './path/global_path_data (2).json'
    ]

    data_list = [process_data(file_path, epsilon) for file_path in file_paths]

    # GPSConverter 초기화
    config = {'utm_zone': 52, 'east_offset': 313008.55819800857, 'north_offset': 4161698.628368007}
    converter = GPSConverter(config['utm_zone'], config['east_offset'], config['north_offset'])

    gps_coordinates = converter.calc_gps_from_pose_batch(data_list[0]["optimized"])
    
    json_data = convert_points_to_json(gps_coordinates)
    print(json_data)
    plot_coordinates(data_list)

if __name__ == "__main__":
    main()
