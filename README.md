Pure Pursuit Algorithm

Author : Dae Jong Jin

Reference :

Implementation of the Pure Pursuit Path Tracking Algorithm(R.Craig Coulter)

https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/pure_pursuit/pure_pursuit.py

Algorithm to follow the path by calculating the curvature for the vehicle to move from the current position to the target position

## 1. Pursuit Algorithm

1. Determine the current location of the vehicle.
2. Find the path point closet to the vehicle.
3. Find the goal point.
4. Transform the goal point to vehicle coordinates.
5. Calculate the curvature and request the vehicle to set the steering to that curvature.
6. Update the vehicle's position

## 2. Equation

<img src="https://user-images.githubusercontent.com/35681273/98942326-2d62b180-2531-11eb-8468-d7b8aa5dfd0f.png" width="400" height = "300" />

## 3. Detailed Description

1. Determine the current location of the vehicle.

   - the vehicle’s current position as (x,y,heading)
   - This original position is the global reference frame for the run
2. Find the path point closet to the vehicle.

   - The vehicle should steer toward the closest point one lookahead distance from its current location.

     Therefore, the path point closest to the vehicle will first be found, and the search for a point 1 lookahead distance away from the vehicle will start at this point and commence up the path.
3. Find the goal point.

   - The goal point is found by moving up the path and calculating the distance between that path point and the vehicle’s current location.

   Path point locations are recorded in the global frame; this calculation is done in global coordinates
4. Transform the goal point to vehicle coordinates.

   - Once the goal point has been found, it must be transformed to the vehicle’s local coordinates. The geometric derivation for the curvature was done in vehicle coordinates and curvature commands to the vehicle make sense in vehicle. coordinates
5. Calculate the curvature.

   - Using the curvature equation derived in the last section, calculate the desired vehicle curvature. The curvature is transformed into steering wheel angle by the vehicle’s on board controller.
6. Update the vehicle’s position.

   - During simulation, it is necessary to determine what effects the command has upon the vehicle’s position and heading.

~~~
풀어 설명하면,
1. 차량의 현재위치(x, y, heading)를 구하고 이동해야 할 경로(이미 저장되어 있는 waypoint) 중 현재 위치와 가장 가까운 점을 첫 번째 인덱스로 정한다.

2. 차량이 이동하는 중 현재 위치에서 target point 까지의 거리가 LD(Look ahead distance) 길이보다 작을 때 target point의 인덱스를 1 증가시킨다. 
(이때, 차량이 뒤로 가지 않으려면
현재위치와 목표점과의 길이는 현재위치와 다음 목표점과의 길이보다 작아야 한다는 조건을 세우고 작지 않다면 인덱스를 1 증가시킨다.)

3. target index에 해당하는 목표 점과 LD를 알고 있으니 차량의 조향각을 구하는 공식에 대입한다.

4. 차량의 위치 (x, y, heading)를 계속 업데이트 한다. 
~~~

## 4. Package

### 1) PID_lib

- pid_control.py
  - 차량의 속도를 제어하기 위한 패키지

### 2) PurePursuit_lib

- pure_pursuit.py
  - 차량의 현재 위치 상태, 목표 점 , 조향각을 업데이트 및 계산하는 패키지

### 3) PLT_lib

- plotlib.py
  - 데이터 시각화를 위해 만든 패키지인 pyplot을 이용한 패키지

## 5. Use

~~~
$ python3 pursuit_test.py
~~~

![Alt Text](https://user-images.githubusercontent.com/35681273/117776770-dbe47680-b276-11eb-9626-6111d1f832b4.gif)

