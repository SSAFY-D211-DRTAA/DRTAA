import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from typing import List, Dict

def plot_coordinates(data_list: List[Dict]):
    font_path = '/usr/share/fonts/truetype/nanum/NanumGothicCoding.ttf'
    font_prop = fm.FontProperties(fname=font_path)

    fig, axs = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('원본 및 최적화된 X, Y 좌표 비교', fontproperties=font_prop)

    colors = ['blue', 'red', 'green']
    labels = ['원본 데이터', '최적화된 경로', '원본 데이터']

    for i, (ax, data) in enumerate(zip(axs, data_list)):
        original_x, original_y = zip(*data['original'])
        optimized_x, optimized_y = zip(*data['optimized'])

        ax.scatter(original_x, original_y, color=colors[0], alpha=0.6, label=labels[0])
        ax.scatter(optimized_x, optimized_y, color=colors[1], alpha=0.8, label=labels[1])
        
        ax.set_title(f'데이터 {i+1}', fontproperties=font_prop)
        ax.set_xlabel('X 좌표', fontproperties=font_prop)
        ax.set_ylabel('Y 좌표', fontproperties=font_prop)
        ax.grid(True)
        ax.legend(prop=font_prop)

        all_x = original_x + optimized_x
        all_y = original_y + optimized_y
        x_min, x_max = min(all_x), max(all_x)
        y_min, y_max = min(all_y), max(all_y)
        
        x_margin = (x_max - x_min) * 0.20
        y_margin = (y_max - y_min) * 0.20
        
        ax.set_xlim(x_min - x_margin, x_max + x_margin)
        ax.set_ylim(y_min - y_margin, y_max + y_margin)

        ax.text(0.05, 0.95, f'원본: {len(data["original"])}개\n최적화: {len(data["optimized"])}개', 
                transform=ax.transAxes, verticalalignment='top', 
                fontproperties=font_prop, bbox=dict(facecolor='white', alpha=0.7))

    plt.tight_layout()
    plt.show()
