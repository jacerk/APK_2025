from PyQt6.QtCore import QPointF
from PyQt6.QtGui import QPolygonF
from Algorithms import Algorithms
import matplotlib.pyplot as plt

def plot_point_categories(ax, points, results, polygon):
    poly_x = [point.x() for point in polygon] + [polygon[0].x()]
    poly_y = [point.y() for point in polygon] + [polygon[0].y()]
    
    ax.plot(poly_x, poly_y, color='blue', linewidth=2, label='Polygon')
    
    inside_pts = [p for p, r in zip(points, results) if r == 1]
    edge_pts = [p for p, r in zip(points, results) if r == -1]
    outside_pts = [p for p, r in zip(points, results) if r == 0]
    
    if inside_pts:
        ax.scatter([p.x() for p in inside_pts], [p.y() for p in inside_pts], 
                  color='green', s=60, label='Inside (1)', edgecolor='black', linewidth=0.5, zorder=10)
    if edge_pts:
        ax.scatter([p.x() for p in edge_pts], [p.y() for p in edge_pts], 
                  color='red', s=60, label='On Edge/Vertex (-1)', edgecolor='black', linewidth=0.5, zorder=10)
    if outside_pts:
        ax.scatter([p.x() for p in outside_pts], [p.y() for p in outside_pts], 
                  color='orange', s=60, label='Outside (0)', edgecolor='black', linewidth=0.5, zorder=10)
    
    ax.legend(framealpha=0.7)
    ax.grid(True, color='gray', alpha=0.3)
    ax.set_aspect('equal')
    ax.tick_params(colors='black')

def run_tests():
    alg = Algorithms()
    
    irregular_star = QPolygonF([
        QPointF(180, 120),
        QPointF(210, 160),
        QPointF(280, 140),
        QPointF(240, 180),
        QPointF(300, 220),
        QPointF(220, 230),
        QPointF(240, 300),
        QPointF(190, 260),
        QPointF(140, 310),
        QPointF(150, 250),
        QPointF(80, 260),
        QPointF(130, 210),
        QPointF(70, 180),
        QPointF(130, 180),
        QPointF(100, 120)
    ])
    
    test_points = [
        QPointF(180, 200),
        QPointF(200, 220),
        QPointF(160, 190),
        QPointF(180, 120),
        QPointF(300, 220),
        QPointF(195, 140),
        QPointF(270, 200),
        QPointF(165, 285),
        QPointF(100, 195),
        QPointF(50, 50),
        QPointF(350, 200),
        QPointF(180, 350),
    ]
    
    ray_results = [alg.ray_crossing(p, irregular_star) for p in test_points]
    winding_results = [alg.winding_number(p, irregular_star) for p in test_points]
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
    
    ax1.set_title('Ray Crossing Algorithm', fontsize=14, pad=10)
    plot_point_categories(ax1, test_points, ray_results, irregular_star)
    
    ax2.set_title('Winding Number Algorithm', fontsize=14, pad=10)
    plot_point_categories(ax2, test_points, winding_results, irregular_star)
    
    plt.tight_layout()
    plt.subplots_adjust(top=0.9)
    
    plt.savefig("Algorithm_Comparison.png", facecolor='white', bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    run_tests() 