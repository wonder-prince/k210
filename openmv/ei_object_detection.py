# Edge Impulse - OpenMV Object Detection Example with Line and Cross Detection

import sensor, image, time, tf, math, uos, gc

# 初始化传感器
sensor.reset()                         # 重置并初始化传感器
sensor.set_pixformat(sensor.RGB565)    # 设置像素格式为RGB565
sensor.set_framesize(sensor.QVGA)      # 设置帧大小为QVGA (320x240)
sensor.set_windowing((240, 240))       # 设置窗口大小为240x240
sensor.skip_frames(time=2000)          # 让摄像头调整

clock = time.clock()

# 颜色设置（如检测多类物体时可扩展）
colors = [
    (255, 0, 0), (0, 255, 0), (255, 255, 0),
    (0, 0, 255), (255, 0, 255), (0, 255, 255),
    (255, 255, 255)
]

# 直线检测参数
threshold = 1000
theta_margin = 25
rho_margin = 25

# 二值化阈值
binary_threshold = (165, 255)  # 假设白色在这个范围内

def detect_lines_in_roi(img, roi):
    """
    仅在指定的ROI区域内检测直线，并返回检测到的最大角度。
    """
    max_angle = None
    displayed_lines = []

    for l in img.find_lines(roi=roi, threshold=threshold, theta_margin=theta_margin, rho_margin=rho_margin):
        angle = l.theta()
        if angle > 90:
            angle -= 180
        angle = -angle  # 将逆时针方向的角度定义为正

        if -30 <= angle <= 30:
            show_line = True
            for displayed_angle in displayed_lines:
                if abs(displayed_angle - angle) < 5:
                    show_line = False
                    break

            if show_line:
                img.draw_line(l.line(), color=(255, 0, 0))
                displayed_lines.append(angle)

                # 更新最大角度
                if max_angle is None or abs(angle) > abs(max_angle):
                    max_angle = angle

    return max_angle

while True:
    clock.tick()
    img = sensor.snapshot()

    # 转换为灰度图像
    img = img.to_grayscale()

    # 二值化处理，分离黄色背景和白色十字
    img.binary([binary_threshold])

    # 应用高斯滤波去噪
    img.gaussian(1)  # 使用高斯滤波，参数为 1 表示 3x3 高斯滤波窗口

    # 寻找最大轮廓
    blobs = img.find_blobs([binary_threshold], pixels_threshold=500, area_threshold=500, merge=True)

    if blobs:
        # 获取最大的十字轮廓
        largest_blob = max(blobs, key=lambda b: b.pixels())

        # 绘制边框和中心十字
        img.draw_rectangle(largest_blob.rect())
        img.draw_cross(largest_blob.cx(), largest_blob.cy())

        # 打印十字中心坐标
        print("十字中心坐标: x=%d, y=%d" % (largest_blob.cx(), largest_blob.cy()))

        # 获取最大轮廓的ROI区域
        roi = largest_blob.rect()

        # 在最大轮廓内的图像区域进行直线检测
        max_angle = detect_lines_in_roi(img, roi)
        if max_angle is not None:
            print("ROI区域内检测到的最大角度: {:.2f}".format(max_angle))

    # 打印 FPS 信息
    print("帧率: {:.2f} fps".format(clock.fps()), end="\n\n")
