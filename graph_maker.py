import pandas as pd
csv_path = '/home/seonwoo/Desktop/workspace/near_polygon_data.csv'
df = pd.read_csv(csv_path)
df = df.iloc[:, [6, 7, 8, 9, 10, 11]] 
#"lane_polyg = 1", "next = 2", "turn_left = 3", "turn_right = 4", "left_polyg = 5", "rigt_polyg = 6"
# df의 행 인덱스는 0부터 247까지 있고, 폴리곤은 1번부터 248번까지 있다.
df.fillna(False, inplace=True)

graph = {}
near = {}

for i in range(0, 248):
    for j in range(2, 7): # 2 ~ 6
        if df[f'{j}'][i] == False:
            pass
        else:
            near[int(df[f'{j}'][i])] = 0
            # print(near)
    graph[df[f'{1}'][i]] = near
    # print(near)
    near = {}
for k, v in graph.items():
    print("{} : {},".format(k, v))
# print(graph)