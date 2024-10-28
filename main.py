import sensor, image, time, lcd, gc, cmath
from maix import KPU
import machine
from modules import ybserial

lcd.init()                          # Init lcd display
lcd.clear(lcd.RED)                  # Clear lcd screen.

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 1000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

# Initialize serial
serial = ybserial()                 # Create serial object
serial.send_byte(0x31)
serial.send_byte(0x0D)
array = [0x30, 0x31, 0x32, 0x33, 0x0D]
serial.send_bytearray(array)        # Send byte array

labels = ["gr_pumpkin", "gr_apple", "red_pumpkin", "re_apple", "gr_pepper", "re_pepper", "crossing"] # Class names
anchor = (2.91, 1.62, 3.77, 2.17, 4.56, 2.69, 5.45, 3.05, 6.36, 3.83) # Anchors

kpu = KPU()
# Load model from sd or flash
kpu.load_kmodel('/sd/fruit.kmodel')
# kpu.load_kmodel(0x300000, 584744)
kpu.init_yolo2(anchor, anchor_num=(int)(len(anchor)/2), img_w=320, img_h=240, net_w=320, net_h=240, layer_w=10, layer_h=8, threshold=0.6, nms_value=0.3, classes=len(labels))

# Initialize UART
uart = machine.UART(machine.UART.UART1, 115200, timeout=1000, read_buf_len=4096)

while True:
    gc.collect()

    clock.tick()
    img = sensor.snapshot()

    kpu.run_with_output(img)
    dect = kpu.regionlayer_yolo2()

    fps = clock.fps()

    if len(dect) > 0:
        for l in dect:
            x, y, w, h, class_id, score = l
            center_x = x + w // 2
            center_y = y + h // 2
            class_id = int(class_id)  # 确保 class_id 是整数
            class_name = labels[class_id]  # 获取类名
            info = "%s %.3f" % (labels[class_id], score)
            img.draw_rectangle(x, y, w, h, color=(0, 255, 0))
            img.draw_string(x, y, info, color=(255, 0, 0), scale=2.0)

            # 根据类名设置 labeling
            if class_name == "red_pumpkin":
                labeling = 1
            elif class_name == "re_pepper":
                labeling = 2
            elif class_name == "crossing":
                labeling = 3
            else:
                labeling = 0


            # Print and send center coordinates and label info
            #output_str = "x={}, y={}, w={}, h={}, center_x={}, center_y={}, id={}".format(x, y, w, h, center_x, center_y, labels[class_id])
            formatted_center_x = "%03d" % center_x
            formatted_center_y = "%03d" % center_y
            output = "5555{}0{}0{}8888".format(formatted_center_x, formatted_center_y, labeling)
            print(output)

            try:
                serial.send((output + "\n").encode())  # Send all information via serial
            except Exception as e:
                print("UART write error:", e)

            del info

    # Display received serial data
    img.draw_string(0, 0, "%2.1ffps" % (fps), color=(0, 60, 255), scale=2.0)

    lcd.display(img)
    time.sleep_ms(100)
