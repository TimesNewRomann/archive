import heapq

def dijkstra(graph, start, end):
    # 그래프의 모든 노드에 대해 초기값을 무한대로 설정하고 시작 노드는 0으로 설정합니다.
    distances = {node: float('inf') for node in graph}
    distances[start] = 0

    # 각 노드까지의 최단 경로를 저장하는 딕셔너리
    previous_nodes = {}

    # 우선순위 큐를 사용하여 방문할 노드를 관리합니다.
    priority_queue = [(0, start)]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        # 현재 노드가 이미 더 짧은 경로로 방문된 경우 건너뜁니다.
        if current_distance > distances[current_node]:
            continue

        # 현재 노드에 인접한 노드들을 탐색합니다.
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight

            # 새로 계산된 거리가 기존의 거리보다 짧은 경우 업데이트합니다.
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    # 경로 추적
    path = []
    current = end
    while current in previous_nodes:
        path.insert(0, current)
        current = previous_nodes[current]
    path.insert(0, start)

    return path

# 그래프와 시작 노드, 목적지 노드 설정
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'C': 2, 'D': 5},
    'C': {'A': 4, 'B': 2, 'D': 1},
    'D': {'B': 5, 'C': 1}
}
start_node = 'A'
end_node = 'D'

# 최단 경로 찾기
shortest_path = dijkstra(graph, start_node, end_node)
print("Shortest path:", shortest_path)
