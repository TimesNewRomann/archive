import pandas as pd

data_col = ["UTM_lat", "UTM_lon"]
data = {"UTM_lat": [], "UTM_lon": []}
user_df = pd.DataFrame(data, columns=data_col)

user_df.to_csv('/home/seonwoo/ros2_ws/coordinates.csv',
               index=False, encoding='cp949')
