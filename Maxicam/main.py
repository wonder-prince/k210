from maix import image, camera, display, app, nn, app, uart, time
import numpy as np
import cv2
import math
import struct

def calculate_xy_offset(image_point):
    z_distance = 310
    # 定义相机内参矩阵和畸变系数（请替换为您的相机参数）
    camera_matrix = np.array([[294.1824, 0, 112],
                              [0, 294.7686, 112],
                              [0, 0, 1]], dtype="double")
    dist_coeffs = np.zeros((4, 1))  # 如果镜头没有畸变，使用零值
    # 提取相机内参矩阵中的参数
    f_x = camera_matrix[0, 0]
    f_y = camera_matrix[1, 1]
    c_x = camera_matrix[0, 2]
    c_y = camera_matrix[1, 2]
    
    # 获取图像中的像素坐标 (u, v)
    u, v = image_point
    
    # 计算 X 和 Y 的偏移量
    x_offset = (u - c_x) * z_distance / f_x
    y_offset = (v - c_y) * z_distance / f_y
    y_offset=-y_offset

    x_offset=round(x_offset,2)
    y_offset=round(y_offset,2)

    print(f"十字中心的相对于图像中心的偏移 - x: {x_offset}, y: {y_offset}")
    serial.write(f'[({x_offset},{y_offset})]\n'.encode())
    
    # 返回实际的 X 和 Y 偏移量
    return x_offset, y_offset





def cv_PnP(image_points, num):
    # 定义相机内参矩阵和畸变系数（请替换为您的相机参数）
    camera_matrix = np.array([[294.1824, 0, 159.6151],
                              [0, 294.7686, 117.1914],
                              [0, 0, 1]], dtype="double")
    dist_coeffs = np.zeros((4, 1))  # 如果镜头没有畸变，使用零值

    # 根据物体类别 `num` 设置相应的3D世界坐标点（object_points）
    if num == 0:  # 例如，类别0的物体是南瓜
        w, h = 85, 85  # 物体宽度和高度（mm）
        object_points = np.array([
            [-w/2, -h/2, 0],
            [w/2, -h/2, 0],
            [w/2, h/2, 0],
            [-w/2, h/2, 0]
        ], dtype="double")
    
    elif num == 1:  # 类别1的物体是辣椒
        w, h = 80, 65  # 十字中心小方块的宽高(mm)
        object_points = np.array([
            [-w/2, -h/2, 0],
            [w/2, -h/2, 0],
            [w/2, h/2, 0],
            [-w/2, h/2, 0]
        ], dtype="double")

    elif num == 2:  # 类别2的物体是苹果
        w, h = 80, 65  # 十字中心小方块的宽高(mm)
        object_points = np.array([
            [-w/2, -h/2, 0],
            [w/2, -h/2, 0],
            [w/2, h/2, 0],
            [-w/2, h/2, 0]
        ], dtype="double")


    # elif num == 3:  # 类别3的物体是十字
    #     w, h ,a= 24, 24, 40  # 实验室十字中心小方块的宽高(mm)
    #     # w,h,a=32,32,90# 实际十字中心小方块的宽高(mm)
    #     object_points = np.array([
    #         [-w/2, -h/2, 0],#十字中心左下
    #         [w/2, -h/2, 0],#十字中心右下
    #         [w/2, h/2, 0],#十字中心右上
    #         [-w/2, h/2, 0]#十字中心左上
    #         # [-w/2, a+h/2, 0],  # 十字的上左
    #         # [w/2, a+h/2, 0],   # 十字的上右
    #         # [-w/2, -a-h/2, 0],  # 十字的下左
    #         # [w/2, -a-h/2, 0],   # 十字的下右
    #         # [-a-w/2, h/2, 0],#十字左上
    #         # [-a-w/2, -h/2, 0],#十字左下
    #         # [a+w/2, h/2, 0],#十字右上
    #         # [a+w/2, -h/2, 0]#十字右下
    #     ], dtype="double")

    
    else:
        print("未知物体类别")
        return None

    # 使用solvePnP计算物体的位姿
    success, rotation_vector, translation_vector = cv2.solvePnP(
        object_points, image_points, camera_matrix, dist_coeffs
    )

    if success:
        if num == 0 or num == 1 or num == 2:
            # 提取相对于主点 (cx, cy) 的偏移量
            cx = camera_matrix[0, 2]
            cy = camera_matrix[1, 2]
            error = 40
            x_offset = translation_vector[0][0] + error
            y_offset = -translation_vector[1][0]

            x_offset=round(x_offset,2)
            y_offset=round(y_offset,2)
            

            # 返回相对于图像中心的 x 和 y 偏移值
            print(f"物体类别 {num} 的相对于图像中心的偏移 - x: {x_offset}, y: {y_offset}")

            serial.write(f'[({x_offset},{y_offset}),{num}]\n'.encode())

            return x_offset, y_offset
        
        # if num == 3:
        #     # 提取相对于主点 (cx, cy) 的偏移量
        #     cx = camera_matrix[0, 2]
        #     cy = camera_matrix[1, 2]
        #     x_offset = translation_vector[0][0]
        #     y_offset = translation_vector[1][0]


        #     x_offset=round(x_offset,2)
        #     y_offset=round(y_offset,2)

        #     # 返回相对于图像中心的 x 和 y 偏移值
        #     print(f"物体类别 {num} 的相对于图像中心的偏移 - x: {x_offset}, y: {y_offset}")
        #     # 使用struct打包中心坐标数据并通过串口发送

        #     return x_offset, y_offset

    else:
        print("位姿计算失败")
        return None


def point_to_line_distance(point, slope, intercept):
    x, y = point
    return abs(slope * x - y + intercept) / np.sqrt(slope**2 + 1)


def find_two_closest_points(contour_points, slope, intercept):
    distances = [(point[0], point_to_line_distance(point[0], slope, intercept)) for point in contour_points]
    # 按距离排序
    distances.sort(key=lambda x: x[1])
    # 提取距离最小的两个点
    closest_points = [distances[0][0], distances[1][0]]
    

    return closest_points


def fit_line(points):
    # 提取 x 和 y 坐标
    x = np.array([p[0] for p in points])
    y = np.array([p[1] for p in points])

    # 最小二乘法拟合直线：y = Bx + A
    A = np.vstack([x, np.ones(len(x))]).T
    parameterB, parameterA = np.linalg.lstsq(A, y, rcond=None)[0]
    return parameterB, parameterA



# 实验室测试版本
# def cross_detect():
#     img = cam.read()  # 从相机读取一帧图像

#     img = img.lens_corr(strength=1.5)  # 应用镜头校正
#     img_raw = image.image2cv(img)  # 将图像转换为OpenCV格式

#     img = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)  # 转为灰度图
#     img = cv2.GaussianBlur(img, (5, 5), 0)  # 高斯模糊
#     img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)  # 闭运算
#     _, img = cv2.threshold(img, 143, 255, cv2.THRESH_BINARY)  # 二值化
#     img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)  # 开运算

#     edged = cv2.Canny(img, 200, 600)  # Canny边缘检测
#     contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 找到轮廓


def cross_detect():
    img = cam.read()  # 从相机读取一帧图像
    img = img.lens_corr(strength=1.5)  # 应用镜头校正
    img_raw = image.image2cv(img)  # 将图像转换为OpenCV格式

    img = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)  # 转为灰度图
    img = cv2.GaussianBlur(img, (5, 5), 0)  # 高斯模糊
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)  # 开运算

    # 使用自适应阈值进行二值化处理
    img = cv2.adaptiveThreshold(
        img, 
        255,  # 二值化后的最大值
        # cv2.ADAPTIVE_THRESH_GAUSSIAN_C, # 自适应方法，基于高斯加权均值
        cv2.ADAPTIVE_THRESH_MEAN_C, # 自适应方法，基于相邻像素均值
        cv2.THRESH_BINARY,  # 阈值类型
        27,  # 邻域大小（必须为奇数）
        2  # 常数C, 从局部均值中减去
    )

    # # 使用闭运算去除噪点（腐蚀+膨胀）
    # img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

    # 使用膨胀操作来加深和连通十字
    img = cv2.dilate(img, kernel, iterations=1)

    img = cv2.erode(img, kernel, iterations=1)

    # # 使用闭运算去除噪点（腐蚀+膨胀）
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel,iterations=2)


    edged = cv2.Canny(img, 10,20)  # Canny边缘检测
    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 找到轮廓

    
    for contour in contours:
        epsilon = 0.01 * cv2.arcLength(contour, True)  # 计算轮廓的周长
        approx = cv2.approxPolyDP(contour, epsilon, True)  # 多边形逼近

        if len(approx) ==12:
            # print("这是一个" + str(len(approx)) + "边形")

            # 计算多边形的质心
            sum_x = sum(point[0][0] for point in approx)
            sum_y = sum(point[0][1] for point in approx)
            # k_x=detect_best_fit_slope(approx)
            center_x = int(sum_x / len(approx))
            center_y = int(sum_y / len(approx))
            print(f"质心坐标: ({center_x}, {center_y})")


            # 找到距离质心最近的四个点
            distances = [(point[0], (point[0][0] - center_x)**2 + (point[0][1] - center_y)**2) for point in approx]
            distances.sort(key=lambda x: x[1])
            closest_points = [point for point, _ in distances[:4]]
            
            #定义四个点
            # image_points = np.array([
            #     approx[0][0],  # 第一个点 (x1, y1)
            #     approx[1][0],  # 第二个点 (x2, y2)
            #     approx[2][0],  # 第三个点 (x3, y3)
            #     approx[3][0]  # 第四个点 (x4, y4)
            #     # approx[4][0],  # 第五个点 (x5, y5)
            #     # approx[5][0],  # 第六个点 (x6, y6)
            #     # approx[6][0],  # 第七个点 (x7, y7)
            #     # approx[7][0],  # 第八个点 (x8, y8)
            #     # approx[8][0],  # 第九个点 (x9, y9)
            #     # approx[9][0],  # 第十个点 (x10, y10)
            #     # approx[10][0], # 第十一个点 (x11, y11)
            #     # approx[11][0]  # 第十二个点 (x12, y12)
            # ], dtype="double")

            calculate_xy_offset((center_x,center_y))

            # 将四个点分为左右两组（假设靠左x值更小）
            closest_points.sort(key=lambda p: p[0])  # 按 x 坐标排序
            left_points = closest_points[:2]
            right_points = closest_points[2:]

            # 将点从 numpy 数组转换为元组，以便进行比较
            right_points_tuple = [tuple(p) for p in right_points]
            left_points_tuple = [tuple(p) for p in left_points]

            # 去除已有的 left_points 中的点
            remaining_left_points = [point for point in approx if tuple(point[0]) not in left_points_tuple]

            remaining_right_points = [point for point in approx if tuple(point[0]) not in right_points_tuple]

            # 计算左线斜率和截距
            left_slope, left_intercept = fit_line(left_points)

            # 离左线最近的两个点
            closest_left_points=find_two_closest_points(remaining_left_points,left_slope, left_intercept)

            for point in closest_left_points:
                cv2.circle(img_raw, (point[0], point[1]), 7, (0, 255, 255), -1)  # 黄色标记

            left_points=left_points+closest_left_points

            left_slope,left_intercept = fit_line(left_points)
            # print(f"左线斜率: {left_slope}, 截距: {left_intercept}")


            # 计算右线斜率和截距
            right_slope, right_intercept = fit_line(right_points)
            
            # 离右线最近的两个点
            closest_right_points=find_two_closest_points(remaining_right_points,right_slope, right_intercept)

            for point in closest_right_points:
                cv2.circle(img_raw, (point[0], point[1]), 7, (0, 255, 255), -1)  # 黄色标记

            right_points=right_points+closest_right_points

            right_slope,right_intercept = fit_line(right_points)
            # print(f"右线斜率: {right_slope}")

            # 计算中线斜率和截距（可以取平均值）
            center_slope = (left_slope + right_slope) / 2
            # center_intercept = (left_intercept + right_intercept) / 2
            # print(f"中线斜率: {center_slope}")

            angle = math.degrees(math.atan(center_slope))
            angle = angle+90
            if angle>90:
                angle-=180
            print(f"中线角度: {angle:.2f} 度")


            # 绘制左右线
            for x in range(img_raw.shape[1]):
                y_left = int(left_slope * x + left_intercept)
                y_right = int(right_slope * x + right_intercept)
                
                if 0 <= y_left < img_raw.shape[0]:
                    img_raw[y_left, x] = (255, 0, 0)  # 左线为蓝色
                if 0 <= y_right < img_raw.shape[0]:
                    img_raw[y_right, x] = (0, 255, 0)  # 右线为绿色

            serial_data = f'[({center_x},{center_y}), {center_slope}]\n'
            serial.write(serial_data.encode())
            # 在图像上绘制质心
            cv2.circle(img_raw, (center_x, center_y), 7, (0, 0, 255), -1)

            # 绘制多边形的顶点和逼近的多边形
            for point in approx:
                x, y = point.ravel()
                cv2.circle(img_raw, (x, y), 5, (255, 0, 0), 2)
            # cv2.drawContours(img_raw, [approx], 0, (0, 255, 0), 3)

    img_show = image.cv2image(img_raw)
    disp.show(img_show)



def fruit_detect():
    img = cam.read()  # 从相机读取一帧图像

    # 使用检测器检测图像中的物体，置信度阈值为0.5，IOU阈值为0.45
    objs = detector.detect(img, conf_th=0.5, iou_th=0.45)

    # 遍历检测到的每个物体
    for obj in objs:
        # 在图像上绘制检测框
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_RED)

        # 计算检测框的中心坐标
        center_x = obj.x + obj.w // 2
        center_y = obj.y + obj.h // 2

        # 在图像上绘制中心点
        img.draw_circle(center_x, center_y, 5, color=image.COLOR_RED)

        # 创建显示标签，包含物体类别名称和置信度
        msg = f'{detector.labels[obj.class_id]}: {obj.score:.2f}'


        if obj.class_id==1 or obj.class_id==2 or obj.class_id==4:
            # 在检测框上方显示标签信息
            img.draw_string(obj.x, obj.y, msg, color=image.COLOR_RED)
            # green_pumpkin:0 orange_pumpkin:1 red_pepper:2  green_pepper:3 apple:4

            # 在控制台输出物体类别和中心坐标
            print(f'物体: {detector.labels[obj.class_id]}, 中心坐标: ({center_x},{center_y})')
        

            # 图像中的2D点对应的世界坐标点
            image_points = np.array([
                (obj.x, obj.y),
                (obj.x + obj.w, obj.y),
                (obj.x + obj.w, obj.y + obj.h),
                (obj.x, obj.y + obj.h)
            ], dtype="double")
            #调用pnp解算
            if obj.class_id==1:
                cv_PnP(image_points,0)
            elif obj.class_id==2:
                cv_PnP(image_points,1)
            else:
                cv_PnP(image_points,2)

    # 显示图像
    disp.show(img)
 



#主程序
devices = uart.list_devices()
serial = uart.UART(devices[0], 115200)
kernel = np.ones((3, 3), np.uint8)  # 创建用于形态学操作的核
detector = nn.YOLOv5(model="/root/models/maixhub/155316/model_155316.mud")  # 初始化YOLOv5检测模型
detector_cross = nn.YOLOv5(model="/root/models/maixhub/156034/model_156034.mud")
cam = camera.Camera(224, 224)
# cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())  # 初始化相机
disp = display.Display()  # 初始化显示器
kernel = np.ones((3, 3), np.uint8)  # 创建用于形态学操作的核
flag=1 #初始化
while not app.need_exit():
    data = serial.read() #data should be 1 or 0
    if data:
        data = int(data.decode('utf-8'))
        print(f"Received data: {data}")
        flag=data
    if (flag==1):
        cross_detect()
    elif(flag==0):
        fruit_detect()
    time.sleep_ms(1)
        