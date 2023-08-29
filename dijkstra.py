import heapq

def dijkstra(graph, start, end):
    # 그래프의 모든 노드에 대해 초기값을 무한대로 설정하고 시작 노드는 0으로 설정
    distances = {node: float('inf') for node in graph}
    distances[start] = 0

    # 각 노드까지의 최단 경로를 저장하는 딕셔너리
    previous_nodes = {}

    # 우선순위 큐를 사용하여 방문할 노드 관리
    priority_queue = [(0, start)]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        # 현재 노드가 이미 더 짧은 경로로 방문된 경우 건너뛴다
        if current_distance > distances[current_node]:
            continue

        # 현재 노드에 인접한 노드들을 탐색한다
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight

            # 새로 계산된 거리가 기존의 거리보다 짧은 경우 업데이트한다.
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
    '154': {'55': 1},
    '55': {'129': 1},
    '129': {'61': 1},
    '61': {'135': 1},
    '135': {'139': 1},
    '139': {'144': 1},
    '144': {'16': 2, '17' : 1},
    '16': {},
    '17': {'203': 1},
    '203': {'204': 1},
    '204': {'205': 1},
    '205': {'39': 1},
    '39': {'216': 1},
    '216': {'147': 1},
    '147': {'149': 1},
    '149': {'151': 1},
    '151': {'153': 1},
    '153': {}
}
start_node = '17' # 시작노드. 외부에서 실시간으로 받아온 현재 위치(노드)를 넣으면, 현재 위치부터 목적노드까지의 경로를 계속 생성해 줄 수 있다.
end_node = '153' # 목표 노드

# 최단 경로 찾기
shortest_path = dijkstra(graph, start_node, end_node)
print("Shortest path:", shortest_path) # 전체 경로 리스트
print("Oncoming_node:", shortest_path[1:4]) # 앞 3노드
#포인트스템프드 형식에 넣어서 데이터를 퍼블리시하는 코드 구현해보기