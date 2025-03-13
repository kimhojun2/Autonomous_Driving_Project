# 2024/03/22 ~ 2024/04/04
- Object Detection
    - HOG    
        "Histogram of Oriented Gradients"의 약자로, 컴퓨터 비전 및 객체 검출 분야에서 사용되는 특징 추출 기술 중 하나이다. HOG는 이미지의 지역적 그래디언트 정보를 히스토그램으로 표현하여 이미지에서 객체의 모양과 윤곽을 설명하는 데 사용된다. HOG 특징은 객체 인식, 객체 검출 및 이미지 분류와 같은 응용 분야에서 특히 유용하다.
    - SVM  
        SVM은 인공지능의 기계학습 분야 중 하나로, 패턴인식, 자료분석을 2개의 범주를 분류하는 이진 분류기이다.주로 분류와 회귀 분석을 위해 사용되며, SVM 알고리즘은 주어진 데이터 집합을 바탕으로 하여 새로운 데이터가 어느 카테고리에 속할 것인지 판단하는 비확률적 이진 선형 분류 모델을 만들게 된다.

    - YOLO  
        YOLO 모델은 이미지를 한 번만 보고 물체를 검출하는 딥러닝 기술을 이용한 물체 검출 모델입니다. 학습된 딥러닝 모델을 기반으로 특징을 추출한 뒤 이를 이용해서 물체의 종류와 위치를 예측합니다. 1장의 이미지에 대한 그리드쉘당 2개의 bounding box를 예측하고 마지막으로 NMS 과정을 통해 최종적으로 확률 값이 높은 예측 결과를 도출합니다.  
        하지만 YoloV5의 아키텍쳐는 최대한 일반적인 환경에서 최소한의 라벨리으로 detection을 할수 있어 선택하였으며 특히 정확도는 떨어지나 속도가 가장 빠른 Yolov5s 모델을 사용. 객체 인식률은 매우 높았지만 gpu의 속도가 시뮬레이터의 통신 속도보다 늦어 실제 사용은 불가능 하였습니다.
    - HSV 색상 기반 객체 인식  
        HSV 모델
        인간의 색인지에 기반을 둔 색상 모델로서 Hue(색조), Saturation(채도), Value(명도), 3가지 성분의 조합으로 표현합니다.

        - Hue(색조) : 색의 종류. 0º~360º의 범위를 갖는다.
        - Saturation(채도) : 색의 선명도, 진함의 정도(가장 진한 상태를 100%로 한다)
        - Value(명도) : 색의 밝기, 밝은 정도(가장 밝은 상태를 100%로 한다)

        RGB 이미지에서 색 정보를 검출하기 위해서는 R, G, B 세가지 속성을 모두 참고하지만, HSV 이미지에서는 H(Hue)가 일정한 범위를 갖는 순수한 색 정보를 가지고 있기 때문에 RGB 이미지보다 쉽게 색을 분류할 수 있다.

- LiDAR-Camera Fusion  
    라이다는 3차원 데이터를 출력해주고, 카메라는 프로젝션된 2차원 데이터를 출력해주기 때문에 카메라와 라이다를 퓨전하기 위해서 calibration 진행이 필수적이다.

    1. **Sensor Calibration을 먼저 진행한다.** parameters_cam, parameters_lidar 정보를 회전 행렬과 변환 행렬을 통해 센서간의 위치정보를 이해(일치 시킨다)
    2. 라이다 센서로부터 얻은 포인트 클라우드를 필요 부분을 제외하고 제거 후 센서 간의 상대적인 위치 및 방향을 나타내는 회전변환 행렬을 사용하여 다른 센서의 좌표계로 변환
    3. 변환된 포인트 클라우드를 카메라의 투영 행렬을 사용하여 2차원 이미지로 투영
    4. 변환된 포인트 클라우드와 투영된 이미지를 결합하여 센서 데이터를 통합
    5. 라이다 센서 클라우드 좌표가 있는 2D 카메라 이미지에서 객체 인식을 진행
    
# 2024/03/01 ~ 2024/03/22
- 자율주행 인지(카메라) 개발
scripts/camera_pedes_detector.py 코드 개발
yolov3 활용 코드 작성
객체 인식 모델 개선을 위해 yolov5s를 통한 커스텀 학습을 통해 모델 생성


# 2024/02/27
### dijkstra 알고리즘 공부

#### - dijkstra 알고리즘이란?
다익스트라(dijkstra) 알고리즘은 그래프에서 한 정점(노드)에서 다른 정점까지의 최단 경로를 구하는 알고리즘 중 하나이다. 이 과정에서 도착 정점 뿐만 아니라 모든 다른 정점까지 최단 경로로 방문하며 각 정점까지의 최단 경로를 모두 찾게 된다. 매번 최단 경로의 정점을 선택해 탐색을 반복하는 것이다.

참고로 그래프 알고리즘 중 최소 비용을 구하는 데는 다익스트라 알고리즘 외에도 벨만-포드 알고리즘, 프로이드 워샬 알고리즘 등이 있다.

#### - dijkstra 알고리즘 문제 풀이 

```python
import sys
import heapq
f = sys.stdin.readline

N = int(f())
M = int(f())
graph = {_:{} for _ in range(1, N+1)}


for m in range(M):
    s, e, w = map(int, f().split())
    if e in graph[s] and graph[s][e] < w:
        continue
    graph[s][e] = w

start, end = map(int, f().split())


def dijkstra(graph, start, end):
    distances = {node : float('inf') for node in range(N+1)}
    distances[start] = 0
    q = []
    heapq.heappush(q, [distances[start], start])

    while q:
        now_distance, now_destination = heapq.heappop(q)

        if distances[now_destination] < now_distance:
            continue

        for new_destination, new_distance in graph[now_destination].items():
            distance = now_distance + new_distance
            if distance < distances[new_destination]:
                distances[new_destination] = distance
                heapq.heappush(q, [distance, new_destination])


    return distances[end]


print(dijkstra(graph,start,end))
```

```python
import heapq
import sys
f = sys.stdin.readline

N, M, K, X = map(int, f().split())
graph = {_:{} for _ in range(1, N+1)}

for i in range(M):
    A, B = map(int, f().split())
    graph[A][B] = 1


def dijkstra(graph, start):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    queue = []
    heapq.heappush(queue, [distances[start], start])

    while queue:
        now_distance, now_destination = heapq.heappop(queue)

        if distances[now_destination] < now_distance:
            continue

        for new_destination, new_distance in graph[now_destination].items():
            distance = now_distance + new_distance
            if distance < distances[new_destination]:
                distances[new_destination] = distance
                heapq.heappush(queue, [distance, new_destination])

    return distances.values()


ans = dijkstra(graph, X)
lst = []
for i, value in enumerate(ans):
    if value == K:
        lst.append(i+1)
if len(lst) !=0:
    for i in lst:
        print(i)
else:
    print(-1)
```

# 2024/02/28
## dijkstra 알고리즘 문제풀이
```python
from collections import deque

def BFS(N,K):
    visited = [0] * (100001)
    distance = [0] * (100001)
    queue = deque()
    queue.append(N)
    result = 0
    if N == K:
        result = 1
    while queue:
        if result == 1:
            break
        t = queue.popleft()
        for i in (t-1, t+1, t*2):
            if 0 <= i < 100001:
                if visited[i] == 0:
                    if i == t*2:
                        distance[i] = distance[t]
                    else:
                        distance[i] = distance[t] + 1
                    visited[i] = 1
                    queue.append(i)
                else:
                    if distance[i] >= distance[t]+1:
                        if i == t*2:
                            distance[i] = distance[t]
                        else:
                            distance[i] = distance[t] + 1
                        queue.append(i)

    print(distance[K])


N,K=map(int,input().split())
BFS(N,K)
```

### ROS 통신 공부
#### - 메시지(message)

노드와 노드가 서로 데이터를 주고 받을때 사용되는 것이 바로 메시지이다. 메시지는 int, float, point, boolean과 같은 변수 형태를 띄고 있으며, 단방향 메시지 송수신 방식의 토픽(topic)과 양방향 메시지 요청(request)/응답(response) 방식의 서비스를 이용한다

 

#### -  토픽(topic)

토픽은 단어 뜻 그대로 주제, 이야깃거리를 의미한다. 퍼블리셔(publisher)와 서브스크라이버(subscriber)가 데이터를 주고받으려면 상호가 연결될 주제가 필요하다. 퍼블리셔가 하나의 토픽을 마스터에 등록한 후 토픽내용을 송신하면, 서브스크라이버가 마스터에 등록된 토픽에 정보를 받아 퍼블리셔와 직접 연결하여 데이터를 주고 받는 것이다.