import heapq

class Node:
    def __init__(self, x, y, walkable=True):
        self.x = x
        self.y = y
        self.walkable = walkable
        self.g_cost = float('inf')  # maliyet başlangıçta sonsuz olarak ayarlanır
        self.h_cost = 0
        self.parent = None

    @property
    def f_cost(self):
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        return self.f_cost < other.f_cost


def heuristic(a, b):
    """Manhattan mesafesi"""
    return abs(a.x - b.x) + abs(a.y - b.y)


def a_star_algorithm(start, end, grid, traffic_lights):
    open_set = []
    heapq.heappush(open_set, (0, start))
    start.g_cost = 0
    start.h_cost = heuristic(start, end)
    
    while open_set:
        _, current = heapq.heappop(open_set)

        # Eğer hedefe ulaştıysak, yolu geri takip ederek döndür
        if current == end:
            return reconstruct_path(end)

        # Komşu düğümleri incele
        for neighbor in get_neighbors(current, grid, traffic_lights):
            if not neighbor.walkable:
                continue

            # Yeni maliyetin hesaplanması
            tentative_g_cost = current.g_cost + 1  # her hareket maliyeti 1 birim olarak ayarlanır
            if tentative_g_cost < neighbor.g_cost:
                neighbor.parent = current
                neighbor.g_cost = tentative_g_cost
                neighbor.h_cost = heuristic(neighbor, end)
                heapq.heappush(open_set, (neighbor.f_cost, neighbor))

    return None  # Yol bulunamazsa


def get_neighbors(node, grid, traffic_lights):
    neighbors = []
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Yukarı, sağ, aşağı, sol

    for dx, dy in directions:
        x, y = node.x + dx, node.y + dy
        if 0 <= x < len(grid) and 0 <= y < len(grid[0]):
            neighbor = grid[x][y]
            update_walkable_status(neighbor, traffic_lights)  # Komşu düğümlerin geçilebilirliğini güncelle
            neighbors.append(neighbor)
    return neighbors


def update_walkable_status(node, traffic_lights):
    """Trafik ışıklarını dikkate alarak düğüm geçilebilirliğini günceller"""
    for light in traffic_lights:
        if (node.x, node.y) == (light.x, light.y):
            node.walkable = light.is_green  # ışık yeşilse geçilebilir, kırmızıysa geçilemez
            return
    node.walkable = True  # ışık yoksa düğüm geçilebilir


def reconstruct_path(end_node):
    path = []
    current = end_node
    while current is not None:
        path.append((current.x, current.y))
        current = current.parent
    path.reverse()
    return path


class TrafficLight:
    def __init__(self, x, y, green_duration, red_duration):
        self.x = x
        self.y = y
        self.green_duration = green_duration
        self.red_duration = red_duration
        self.current_time = 0
        self.is_green = True

    def update_light(self, delta_time):
        self.current_time += delta_time
        if self.is_green and self.current_time >= self.green_duration:
            self.is_green = False
            self.current_time = 0
        elif not self.is_green and self.current_time >= self.red_duration:
            self.is_green = True
            self.current_time = 0


# Örnek kullanım:
# 10x10 grid oluşturulması
grid_size = 10
grid = [[Node(x, y) for y in range(grid_size)] for x in range(grid_size)]

# Trafik ışığı oluşturulması
traffic_lights = [
    TrafficLight(2, 2, green_duration=5, red_duration=5),
    TrafficLight(5, 5, green_duration=3, red_duration=7)
]

# Trafik ışıklarının durumlarını güncelle
for light in traffic_lights:
    light.update_light(delta_time=1)

start = grid[0][0]
end = grid[7][7]

path = a_star_algorithm(start, end, grid, traffic_lights)

print("Path found:", path)
total_cost = len(path) - 1  # Her hareket için sabit maliyet
print("Toplam maliyet:", total_cost)

