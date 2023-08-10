# 여러개의 노드가 필요함
# 음의 간선(edge)이 존재하지 않는이상 즉, edge >= 0 양수일때, 동작

# 1. 출발노드 설정
# 2. 최단 거리 테이블 초기화(최단거리를 기록할 테이블을 정의해야 함)
# 3. 방문하지 않은 노드 중에서 최단 거리가 가장 짧은 노드를 선택(방문하지 않은 노드를 체크해야 하므로 이를 위한 테이블을 정의해야 함)
# 4. 해당 노트를 거쳐 다른 노드로 가는 간선 비용을 계산하여 최단 거리 테이블을 업테이트
# 5. 3~4번 반복

# 최단 거리를 기록할 테이블(리스트) 정의하기

# 시간복잡도를 고려하여 우선순위 큐(heap)을 사용하여 개선된 다익스트라 알고리즘으로 구현하기

import heapq  # 우선순위 큐 구현


def dijkstra(graph, start):
    distances = {node: float('inf')
                 for node in graph}  # start로 부터의 거리 값을 저장하기 위함. 모든 노드에 대해 최기값을 무한대로 설정
    distances[start] = 0  # 시작 값은 0

    priority_queue = [(0, start)]  # 우선순위 큐 방문노드 관리
    while priority_queue:
        current_distance, current_node = heapq.heappop(
            priority_queue)  # 탐색할 노드, 거리를 가져옴

        if current_distance > distances[current_node]:  # 이전 거리보다 크다면 무시
            continue

        for new_destination, new_distance in graph[current_node].items():
            distance = current_distance + new_distance  # 해당 노드를 거쳐 갈 때 거리

            if distance < distance[new_destination]:  # 알고있는 거리보다 작으면 업데이트
                distances[new_destination] = distance
                # 다음 인접 거리를 계산하기 위해 큐에 삽입
                heapq.heappush(priority_queue, (distance, new_destination))
    return distances


graph = {}

result = dijkstra(graph, 'A')
print(result)
