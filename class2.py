import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt64

ackermann_data = AckermannDriveStamped()

estimated_previous_val_right = [0] * 720 # 한 사이클 전의 실제 우측 벽과의거리
estimated_previous_val_left = [0] * 720 # 한 사이클 전의 실제 좌츨 벽과의거리
estimated_current_val_right = [0] * 720 # 실제 우측 벽과의거리를 저장해둠
estimated_current_val_left = [0] * 720 # 실제 좌측 벽과의거리를 저장해둠
current_val_right = [0] * 720 # 실제 우측 벽과의 거리
current_val_left = [0] * 720 # 실제 좌측 벽과의 거리
group_right_x = [0] * 720 # 우측기준 각 센서들의 x값
group_right_y = [0] * 720 # 우측기준 각 센서르의 y값
group_left_x = [0] * 720 # 좌측기준 각 센서들의 x값
group_left_y = [0] * 720 # 좌측기준 각 센서들의 y값
distance_data_within_15m = [0] * 720 # 15m 보다 작은 값들 저장
right_largest_value_next = [0] * 720 # 우측 가장 큰 값을 구하기 위한 리스트
right_largest_degree_next = [0] * 720 # 우측 가장 큰 값의 각도를 구하기 위한 리스트
left_largest_value_next = [0] * 720 # 좌측 가장 큰 값을 구하기 위한 리스트
left_largest_degree_next [0] * 720 # 좌측 가장 큰 값의 각도를 구하기 위한 리스트

left_return = 0
right_return = 0
main_wall = 1 # 1: 우측 0: 좌측


class Race_car:    # 클래스함수로 정의
    def __init__(self):
        self.publisher = rospy.Publisher("/servo", UInt64, queue_size = 10 )
        self.subscriber = rospy.Subscriber("/scan", LaserScan, call_back)
        self.subscriber = rospy.Subscriber("/imu", Imu, call_back2)
        # callback2

    def call_back(self, scan_data):    # 센서값
        self.scan = scan_data.ranges
        
        for i in range(720):   # 현재값과의 비교를 위해 이전값 저장
            estimated_previous_val_right[i] = estimated_current_val_right[i] 
            estimated_previous_val_left[i] = estimated_current_val_left[i] 

        for i in range(720):
            current_val_right[i] = scan[ 291 + i ]  # 실제 우측 벽과의 거리
            current_val_left[i] = scan[ 840 - i ]   # 실제 좌측 벽과의 거리

        for i in range(720):
            estimated_current_val_right[i] = current_val_right[i] # 현재값 갱신
            estimated_current_val_left[i] = current_val_left[i]   # 현재값 갱신

        laserscan_and_obtain_coordinate_data() # 실제 벽면의 x,y 좌표 계산 함수

        val_forward = scan[560] # 정면의 벽과의 거리

        collect_distance_data_within_15m() # 15를 기준으로 잡고 작은값들을 리스트에 넣어주는 함수

        find_largest_distance_data_on_left_right_respectively() # 좌, 우측 코너 확인 함수

        change_angle_base_2() # 각도변환 함수 

        ################## 직진 ###################
        if left_corner_exist  == 0 and right_corner_exist == 0:

            distance_now = 0.3

            if left_return == 0 and right_return == 0:
                main_wall = 1
            elif left_return == 1 and right_return == 0:
                main_wall = 1
            elif left_return == 0 and right_return == 1:
                main_wall = 0

            if val_forward > 3:
                select_wall_and_driving_mode() # 가상벽 만드는 함수
                select_path_and_PD_control() # PID 부분

            elif val_forward < 3 and val_forward > 1:
                select_middle_value_between_left_right() # 좌, 우측 최대값의 좌표 
                select_theta_now_Low_Pass_Filter() # PID부분
                Steering_PID_Control_Using_Gyto() # PID부분
            
            elif val_forward < 1:
                if main_wall == 1:
                    distance_now = 3
                if main_wall == 0:
                    distance_now = 6

                select_wall_and_driving_mode()
                select_path_and_PD_control()
        ############################################


        ################ 좌측 코너 #################
        elif left_corner_exist == 1 and right_corner_exist == 0:

            distance_now = 0.3

            left_return = 1
            right_return = 0
            main_wall = 0

            if val_forward > 1.5:
                select_wall_and_driving_mode()
                select_path_and_PD_control()

            elif val_forward < 1.5 and val_forward > 1:
                select_middle_value_between_left_right()
                select_theta_now_Low_Pass_Filter()
                Steering_PID_Control_Using_Gyto()

            elif val_forward < 1:
                if main_wall == 1:
                    distance_now = 3
                if main_wall == 0:
                    distance_now = 6

                select_wall_and_driving_mode()
                select_path_and_PD_control()
        ############################################### 


        ############### 우측 코너 ######################
        elif left_corner_exist == 0 and right_corner_exist == 1:

            distance_now = 0.3

            left_return = 0
            right_return = 1
            main_wall = 1

            if val_forward > 1.5:
                select_wall_and_driving_mode()
                select_path_and_PD_control()

            elif val_forward < 1.5 and val_forward < 1:
                select_middle_value_between_left_right()
                select_theta_now_Low_Pass_Filter()
                Steering_PID_Control_Using_Gyto()  

            elif val_forward < 1:
                if main_wall == 1:
                    distance_now = 3
                if main_wall == 0:
                    distance_now = 6

                select_wall_and_driving_mode()
                select_path_and_PD_control()  
        ################################################


        ################## 세갈래길 ####################
        elif left_corner_exist == 1 and right_corner_exist == 1:
            if left_largest_value > right_largest_value:

                distance_now = 0.3

                left_return = 1
                right_return = 0
                main_wall = 0

                if val_forward > 1.5:
                    select_wall_and_driving_mode()
                    select_path_and_PD_control()

                elif val_forward < 1.5 and val_forward < 1:
                    select_middle_value_between_left_right()
                    select_theta_now_Low_Pass_Filter()
                    Steering_PID_Control_Using_Gyto()  

                elif val_forward < 1:
                    if main_wall == 1:
                        distance_now = 3
                    if main_wall == 0:
                        distance_now = 6

                    select_wall_and_driving_mode()
                    select_path_and_PD_control()  

            if left_largest_value < right_largest_value:

                distance_now = 0.3

                left_return = 0
                right_return = 1
                main_wall = 1

                if val_forward > 1.5:
                    select_wall_and_driving_mode()
                    select_path_and_PD_control()

                 elif val_forward < 1.5 and val_forward < 1:
                    select_middle_value_between_left_right()
                    select_theta_now_Low_Pass_Filter()
                    Steering_PID_Control_Using_Gyto()  

                elif val_forward < 1:
                    if main_wall == 1:
                        distance_now = 3
                    if main_wall == 0:
                        distance_now = 6

                    select_wall_and_driving_mode()
                    select_path_and_PD_control()
        #####################################################

        speed_control() # 

        #servo.data_servo_data

        #ros_tutorial_pub.publish(servo)

        #dt = clock() - start




def laserscan_and_obtain_coordinate_data():

    for i in range(720):    # 720개의 센서들의 우측기준 좌표와 좌측기준 좌표
        group_right_x[i] = estimated_current_val_right[i] * cos( i * 3.14 / 720 ) #밑변
        group_right_y[i] = estimated_current_val_right[i] * sin( i * 3.14 / 720 ) #높이
        group_left_x[i] = estimated_current_val_left[i] * cos( 3.14 - i * 3.14 / 720 ) #밑변
        group_left_y[i] = estimated_current_val_left[i] * sin( 3.14 - i * 3.14 / 720 ) #높이


def collect_distance_data_within_15m():

    for i in range(720):
        if estimated_current_val_right[i] < 15:
            distance_data_within_15m[i] = estimated_current_right[i] # 15m보다 작은 값들을 저장

        else:
            distance_data_within_15m[i] = 0 # 15m보다 큰값의 자리에는 0


def find_largest_distance_data_on_left_right_respectively(): # 좌, 우측 코너 확인 함수

    right_largest_value = distance_data_within_15m[0] # 우측 가장 큰 값을 임의로 설정
    left_largest_value = distance_data_within_15m[361] # 좌측 가장 큰 값을 임의로 설정

    right_corner_exist = 0 # 우측코너 X
    left_corner_exist = 0 # 좌측코너 X

    for i in ragne(360):
        if distance_data_within_15m[i] >= right_largest_value: # 임의로 지정한 우측 가장큰값보다 큰 수 찾기
            right_largest_value = distance_data_within_15m[i]  # 우측 0도부터 정면 90도 까지의 가장 큰값을 저장 할 수 있음
            right_largest_degree = i/4 #  0.25도 마다 센서 1개 , 가장 큰값의 i번째 센서 / 4 를 해주면 가장 큰 값의 각도

    for i in range(right_largest_degree * 4 + 1, 0, -1): # 가장 큰값의 센서에서 0번째 센서까지 -1 
        if distance_data_within_15m[i] == 0: # 15m보다 큰 값들이면
            right_largest_value_next[i] = 0  # i번째 자리에 0을 넣어줌
            right_largest_degree_next[i] = 0 # i번째 자리에 0을 넣어줌

        elif distance_data_within_15m[i] !=0: # 15m보다 작은 값들이면
            right_largest_value_next[i] = distance_data_within_15m[i] # 15m보다 작은 실제 거리값을 넣어줌
            right_largest_degree_next[i] = i / 4 # 각 i번째 센서들의 각도를 넣어줌

            if right_largest_value_next[i + 1] > 2 * right_largest_value_next[i]: # i + 1 번째 센서의 길이가 , i 번째 센서의 길이의 2배 이상이면
                right_largest_value = right_largest_value_next[i + 1] # i + 1 번째 길이를 최대값으로 넣어줌
                right_largest_degree = (i+1) / 4 # i + 1 번째 센서의 각도를 넣어줌
                right_corner_previous_value = right_largest_value_next[i] # i 번째 길이를 넣어줌
                right_corner_previous_degree = i / 4 # i 번째 센서의 각도를 넣어줌

                right_corner_exist = 1 # 오른쪽 코너가 존재 확인

                break # 확인이 됐으면 for문을 나가서 함수 종료

    for i in range(360,720):
        if distance_data_within_15m[i] >= left_largest_value:
            left_largest_value = distance_data_within_15m[i] # 정면 360번째 센서부터 
            left_largest_degree = i / 4 # 0.25도 마다 센서 1개 , 가장 큰값의 i번째 센서 / 4 를 해주면 가장 큰 값의 각도

    for i in ragne(left_largest_degree * 4 + 1, 720): # 가장 큰 값의 센서에서 719번째 센서까지
        if distance_data_within_15m[i] == 0: # 15m보다 큰 값들이면
            left_largest_value_next[i] = 0 # i 번째 자리에 0을 넣어줌
            left_largest_degree_next[i] = 0 # i번째 자리에 0을 넣어줌

        elif distance_data_within_15m[i] != 0: # 15m보다 작은 값들이면
            left_largest_value_next[i] = distance_data_within_15m[i] # 15m보다 작은 실제 거리값을 넣어줌
            left_largest_degree_next[i] = i / 4 # 각 i번째 센서들의 각도를 넣어줌

            if left_largest_value_next[i - 1] > 2 * left_largest_value_next[i]: # i - 1 번째 센서의 길이가 , i 번째 센서의 길이의 2배 이상이면
                left_largest_value = left_largest_value_next[i - 1] # i - 1 번째 길이를 최대값으로 넣어줌
                left_largest_degree = (i-1) / 4 # i - 1 번째 센서의 각도를 넣어줌
                left_corner_previous_value = left_largest_value_next[i] # i번째 길이를 넣어줌
                left_corner_previous_degree = i / 4 # i번째 센서의 각도를 넣어줌

                left_corner_exist = 1 # 왼쪽 코너가 존재 확인

                break # 확인이 됐으면 for문을 나가서 함수 종료 

def change_angle_base_2():
    right_largest_degree_scaling = -(right_largest_degree - 90)  # x축 기준 각도를 y축 기준 각도로 변환해줌
    left_largest_degree_scaling = -(left_largest_degree - 90) # x축 기준 각도를 y축 기준 각도로 변환해줌
    right_corner_previous_degree_scaling = -(right_corner_previous_degree - 90) # 우측 최대길이와 2배차이가 났던 바로 옆 센서도 위와같이 각도 변환 
    left_corner_previous_degree_scaling = -(left_corner_previous_degree - 90) #  좌측 최대길이와 2배차이가 났던 바로 옆 센서도 위와같이 각도 변환

def select_middle_value_between_left_right():
    x_left_largest_value = left_largest_value * sin(left_largest_degree_scaling * 3.14 / 180) # 변환한 각도에서의 좌측 최대값의 x좌표
    y_left_largest_value = left_largest_value * cos(left_largest_degree_scaling * 3.14 / 180) # 변환한 각도에서의 좌측 최대값의 y좌표

    x_right_largest_value = right_largest_value * sin(right_largest_degree_scaling * 3.14 / 180) # 변환한 각도에서의 우측 최대값의 x좌표
    y_right_largest_value = right_largest_value * cos(right_largest_degree_scaling * 3.14 / 180) # 변환한 각도에서의 우측 최대값의 y좌표


def select_wall_and_driving_mode(): 

    if main_wall == 1:
        
        if val_forward > 1.5:
            x_total = 0
            y_total = 0
            a_nominator = 0 # 분자값 초기화
            a_denominator = 0 # 분모값 초기화

            for i in range(180):
                x_total = x_total + group_right_x[i]
                y_total = y_total + group_right_y[i]

            x_avg = x_total / 180
            y_avg = x_total / 180

            for i in range(180):
                a_nominator = a_nominator + ( (group_right_x[i] - x_avg) * (group_right_y[i] - y_avg) )
                a_denominator = a_denominator + ( (group_right_x[i] - x_avg) * (group_right_x[i] - x_avg) )

            a = a_nominator / a_denominator
            b = y_avg - a * x_avg

        if val_forward < 1.5:
            a = -val_forward / estimated_current_val_right[0]
            b = val_forward


    if main_wall == 0:
        
        if val_forward > 1.5:
            x_total = 0
            y_total = 0
            a_nominator = 0 # 분자값 초기화
            a_denominator = 0 # 분모값 초기화

            for i in range(180):
                x_total = x_total + group_left_x[i]
                y_total = y_total + group_left_y[i]

            x_avg = x_total / 180
            y_avg = x_total / 180

            for i in range(180):
                a_nominator = a_nominator + ( (group_left_x[i] - x_avg) * (group_left_y[i] - y_avg) )
                a_denominator = a_denominator + ( (group_left_x[i] - x_avg) * (group_left_x[i] - x_avg) )

            a = a_nominator / a_denominator
            b = y_avg - a * x_avg

        if val_forward < 1.5:
            a = -val_forward / estimated_current_val_right[0]
            b = val_forward


if __name__ == '__main__':
    rospy.init_node('/scan')
    rospy.spin()