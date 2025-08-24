'''
    2025电赛E题找A4纸，定位激光点，串口通信
    有多种设置和算法，根据实际情况选择。
    控制云台可以基于中心点误差 err_center 进行 PID 控制
    
    新增功能：
    - 检测到黑框时自动发送串口命令帧
    - 激光点和圆心重合时自动发送串口命令帧
    - 命令帧格式：帧头1(0xAA) + 帧头2(0x55) + 数据(4字节) + 校验位 + 帧尾(0xFF)
    - 命令帧长度：8字节
    
    @author Neucrack@sipeed & lxo@sieed 协助
    @license MIT
    @date 2025.7.30
    @modified 2025.8.1 - 添加串口通信功能
'''
from enum import auto
from maix import camera, display, image, nn, app, time, uart
import cv2
import numpy as np

DEBUG=False
PRINT_TIME = False

################################ config #########################################

# DEBUG=True                 # 打开调试模式，取消注释即可
# PRINT_TIME = True          # 打印每一步消耗的时间，取消注释即可
debug_draw_err_line = False   # 画出圆心和画面中心的误差线，需要消耗1ms左右时间
debug_draw_err_msg = False    # 画出圆心和画面中心的误差值和 FPS 信息，需要消耗7ms左右时间，慎用
debug_draw_circle = False     # 画出圆圈，实际是画点，需要再打开变量, debug 模式都会画，耗费时间比较多，慎用
debug_draw_rect = False       # 画出矩形框
debug_show_hires = False      # 显示结果在高分辨率图上，而不是小分辨率图上， 开启了 hires_mode 才生效
debug_draw_crosshair = True   # 在画面中心画十字标记
print_fps_terminal = True     # 在终端实时打印FPS信息


crop_padding = 12            # 裁切图时的外扩距离，调试到保证最近和最远位置整个黑框在检测框里，可以打开 DEBUG 模式看
rect_min_limit = 12          # 找到的大黑边框四个点最小距离必须大于这个值才有效，防止找到错误的值，可以放到最远位置测试
std_from_white_rect = True   # 裁切标准图是裁切自A4纸内部白色部分（更精准），False则是带黑框的外围框（整个A4纸）（更快一点点）
circle_num_points = 50       # 生成的第三个圆圈的点数量，控制圆边的平滑程度，可以用来巡迹
std_res = [int(29.7 / 21 * 80), 80]        # 找中心点和圆圈的分辨率，越大越精确，更慢，A4 29.7 x 21cm
hires_mode = True           # 高分辨模式，适合 find_circle 模式使用，帧率会更低但是找圆圈更精准
                             # 不 find_circle 也可以使用，找4个角点更精准，需要配合设置合理的 std_res
                             # 注意开启了这个模式，输出的误差值也是基于大图的分辨率
high_res = 448               # 高分辨率模式宽高,越高越清晰但是帧率越低，注意 std_res 也要跟着改大点
model_path = "/root/yolo25e/best001.mud" # 检测黑框模型路径，从 https://maixhub.com/model/zoo/1159 下载并传到开发板的 /root/models 目录


find_circle = False          # 在找到黑框以内白框后是否继续找圆，如果圆圈画得标准，在纸正中心则不用找，如果画点不在纸正中心则需要找。
                             # 建议把A4纸制作正确就不用找了，帧率更高。
                             # 可以用hires_mode 更清晰才能识别到，另外设置合理的 std_res
cam_buff_num = 1             # 摄像头缓冲， 1 延迟更低帧率慢一点点， 2延迟更高帧率高一点点
find_laser = False           # 找激光点（未测试），实际使用时直接把摄像头中心和激光点保持移植就好了，不需要找激光点

auto_awb = True                            # 自动白平衡或者手动白平衡
awb_gain = [0.134, 0.0625, 0.0625, 0.1139]  # 手动白平衡，auto_awb为False才生效， R GR GB B 的值，调 R 和 B 即可
contrast = 80                               # 对比度，会影响到检测，阴影和圆圈痕迹都会更重

# 串口通信配置
enable_serial_communication = True        # 是否启用串口通信

# 工作模式配置
WORK_MODE_IDLE = 0                        # 待机模式 - 等待指令
WORK_MODE_DETECTION = 1                   # 检测模式

# 模式切换命令配置 - 简化为单字节命令
MODE1_TRIGGER_BYTE = 0xA1  # 📝 模式一触发命令：单字节 0xA1

# 检测结果发送数据配置
BLACKLINE_DETECTED_DATA = [0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88]            # 📝 检测到黑线时发送的16进制数据

# 命令帧配置（保留原有配置用于复杂通信）
FRAME_HEADER1 = 0xAA                      # 帧头1
FRAME_HEADER2 = 0x55                      # 帧头2  
FRAME_TAIL = 0xFF                         # 帧尾
BLACK_RECT_COMMAND = [0x11, 0x11, 0x11, 0x11]      # 检测到黑框的命令数据
LASER_CENTER_COMMAND = [0x22, 0x22, 0x22, 0x22]    # 激光点和圆心重合的命令数据
CENTER_ORIGIN_COMMAND = [0x33, 0x33, 0x33, 0x33]   # 圆心接近坐标系原点的命令数据

#############################################################

device = "/dev/ttyS0"
# ports = uart.list_devices() # 列出当前可用的串口

serial = uart.UART(device, 115200)

# 打印串口通信初始化信息
print("=" * 60)
print("MaixCAM 视觉检测系统初始化")
print("=" * 60)
print(f"串口设备: {device}")
print(f"波特率: 115200")
print(f"串口通信: {'启用' if enable_serial_communication else '禁用'}")
print(f"工作模式: 待机模式 (等待上位机指令)")
# 简化配置信息，移除圆心相关参数
print("📝 模式切换命令配置 (单片机发送，MaixCAM接收):")
print(f"  模式一触发: 0x{MODE1_TRIGGER_BYTE:02X} (单字节)")
print("📝 检测结果数据配置 (请根据实际需求修改):")
print(f"  黑线检测: {[hex(x) for x in BLACKLINE_DETECTED_DATA]} (长度: {len(BLACKLINE_DETECTED_DATA)}字节)")
print("🔧 系统说明:")
print("  - 系统启动时处于待机模式，串口不主动发送数据")
print("  - 单片机发送 0xA1 字节，MaixCAM自动切换到检测模式")
print("  - 模式一：视觉检测模式，检测黑线并发送检测信号")
# 移除中心标定相关信息
print("=" * 60)

print("=" * 60)

###################################################################################
# 初始化摄像头
detector = nn.YOLO11(model=model_path, dual_buff = True)

# 初始化摄像头
if hires_mode:
    cam = camera.Camera(high_res, high_res, detector.input_format(), buff_num=cam_buff_num)
else:
    cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format(), buff_num=cam_buff_num)
if not auto_awb:
    cam.awb_mode(camera.AwbMode.Manual)	
    cam.set_wb_gain(awb_gain)
cam.constrast(contrast)
# cam.set_windowing([448, 448])

disp = display.Display()

def calculate_checksum(data):
    '''
    计算校验位：简单的累加和校验
    '''
    return sum(data) & 0xFF

def send_simple_data(data_byte):
    '''
    发送简单的16进制数据
    '''
    if not enable_serial_communication:
        return
    
    try:
        serial.write(bytes([data_byte]))
    except Exception as e:
        pass

def send_blackline_detected():
    '''
    发送检测到黑线的数据 - 发送完整的8字节数据帧
    '''
    if not enable_serial_communication:
        print("[串口发送] 串口通信已禁用，跳过发送")
        return
    
    try:
        # 确保数据是8字节
        if len(BLACKLINE_DETECTED_DATA) != 8:
            print(f"[串口发送] 错误：数据长度不是8字节，当前长度: {len(BLACKLINE_DETECTED_DATA)}")
            return
        
        
        # 转换为字节数组并发送
        data_bytes = bytes(BLACKLINE_DETECTED_DATA)
        print(f"  字节数组: {data_bytes}")
        
        # 发送全部8字节数据
        bytes_sent = serial.write(data_bytes)
        print(f"[串口发送] 实际发送字节数: {bytes_sent}")
        
        # 验证发送的字节数是否正确
        if bytes_sent == 8:
            print(f"[串口发送] ✓ 成功发送完整8字节数据帧")
        
        # 强制刷新串口缓冲区，确保数据立即发送
        try:
            serial.flush()
            print(f"[串口发送] ✓ 串口缓冲区已刷新，数据已发送到硬件")
        except Exception as flush_error:
            print(f"[串口发送] ⚠️ 缓冲区刷新失败: {flush_error}")
            
    except Exception as e:
        print(f"[串口发送] ❌ 发送失败: {e}")
        print(f"[串口发送] 错误详情: {type(e).__name__}: {str(e)}")



def check_serial_commands():
    '''
    串口命令检查函数 - 简化为单字节接收，收到命令后发送确认消息
    '''
    if not enable_serial_communication:
        return WORK_MODE_IDLE
    
    try:
        # 有数据才读取
        data = serial.read(16)  # 无超时参数
        if not data or len(data) == 0:
            return WORK_MODE_IDLE
        
        data_list = list(data)
        
        # 检查是否包含 0xA1 字节
        for byte_val in data_list:
            if byte_val == MODE1_TRIGGER_BYTE:
                # 发送确认消息
                confirmation_message = b'ACK:DETECTION_MODE'
                serial.write(confirmation_message)
                print("串口收到检测命令，已发送确认消息")
                return WORK_MODE_DETECTION
        
        return WORK_MODE_IDLE
        
    except:
        return WORK_MODE_IDLE

def create_command_frame(command_data):
    '''
    创建命令帧格式：帧头1 + 帧头2 + 数据 + 校验位 + 帧尾
    '''
    # 构建数据部分
    data_bytes = [FRAME_HEADER1, FRAME_HEADER2] + command_data
    
    # 计算校验位（对帧头和数据的校验）
    checksum = calculate_checksum(data_bytes)
    
    # 组装完整命令帧：帧头+数据+校验位+帧尾
    command_frame = data_bytes + [checksum, FRAME_TAIL]
    
    # 检查命令帧长度是否符合预期（2字节帧头 + 4字节数据 + 1字节校验 + 1字节帧尾 = 8字节）
    expected_length = 8
    if len(command_frame) != expected_length:
        pass  # 长度异常时不输出警告
    
    return bytes(command_frame)


def draw_crosshair(img, center_x, center_y, size=20, color=image.COLOR_GREEN, thickness=2):
    '''
    在指定位置画十字标记
    '''
    # 水平线
    img.draw_line(center_x - size, center_y, center_x + size, center_y, color, thickness)
    # 垂直线
    img.draw_line(center_x, center_y - size, center_x, center_y + size, color, thickness)

_t = time.ticks_ms()
def debug_time(msg):
    if PRINT_TIME:
        global _t
        print(f"t: {time.ticks_ms() - _t:4d} {msg}")
        _t = time.ticks_ms()

# 画面中心点
center_pos = [cam.width() // 2, cam.height() // 2] # 画面的中心

# 串口通信状态标志
last_black_rect_detected = False

# 工作模式状态
current_work_mode = WORK_MODE_IDLE         # 初始状态：待机模式

# FPS统计变量
fps_counter = 0
fps_last_time = time.ticks_ms()
fps_interval = 1000  # 每1秒打印一次FPS

# 在主循环开始时添加模式状态打印（仅在模式切换时打印）
last_printed_mode = None

while not app.need_exit():
    debug_time("start")
    img = cam.read()
    debug_time("cam read")
    
    # 检查串口命令，更新工作模式
    received_mode = check_serial_commands()
    
    # 只有在明确接收到模式切换命令时才改变工作模式
    if received_mode != WORK_MODE_IDLE or current_work_mode == WORK_MODE_IDLE:
        if received_mode != current_work_mode:
            current_work_mode = received_mode
            
            # 打印模式切换信息
            mode_names = {
                WORK_MODE_IDLE: "待机模式",
                WORK_MODE_DETECTION: "检测模式"
            }
            print(f"[系统] 当前工作模式: {mode_names.get(current_work_mode, '未知模式')}")
    
    # 只在检测模式下进行AI检测
    if current_work_mode == WORK_MODE_DETECTION:
        # 初始化变量，避免NameError
        approx = None
        crop_ai_cv = None
        binary = None
        crop_rect = None
        
        # AI 检测外框（模式一的逻辑）
        if hires_mode:
            img_ai = img.resize(detector.input_width(), detector.input_height())
        else:
            img_ai = img # new copy
        debug_time("resize")
        objs = detector.detect(img_ai, conf_th = 0.5, iou_th = 0.45)
        max_idx = -1
        max_s = 0
        for i, obj in enumerate(objs):
            s = obj.w * obj.h
            if s > max_s:
                max_s = s
                max_idx = i
        debug_time("detect")
        
        # 串口通信：检测到黑框（黑线）状态变化
        black_rect_detected = max_idx >= 0
        if black_rect_detected and not last_black_rect_detected:
            print(f"[检测] 检测到黑线，准备发送串口信号")
            send_blackline_detected()
        last_black_rect_detected = black_rect_detected
        
        if max_idx >= 0 and debug_draw_rect:
            obj = objs[max_idx]
            # 简化显示，只在需要时画出矩形框
            crop_rect = [int(obj.x * img.width() / img_ai.width()), 
                        int(obj.y * img.height() / img_ai.height()), 
                        int(obj.w * img.width() / img_ai.width()), 
                        int(obj.h * img.height() / img_ai.height())]
            img.draw_rect(crop_rect[0], crop_rect[1], crop_rect[2], crop_rect[3], 
                        color=image.COLOR_RED, thickness=2)
    
    # 显示图像（所有模式下都显示摄像头画面）
    if DEBUG or debug_show_hires:
        # 在画面中心画十字标记
        if debug_draw_crosshair:
            draw_crosshair(img, center_pos[0], center_pos[1])
        disp.show(img)
    else:
        # 根据不同模式显示不同内容
        if current_work_mode == WORK_MODE_DETECTION:
            # 检测模式：显示AI检测结果
            # 在画面中心画十字标记
            if debug_draw_crosshair:
                ai_center_x = detector.input_width() // 2
                ai_center_y = detector.input_height() // 2
                draw_crosshair(img_ai, ai_center_x, ai_center_y, size=15)
            disp.show(img_ai)
        else:
            # 待机模式：仅显示摄像头画面和十字标记
            if debug_draw_crosshair:
                draw_crosshair(img, center_pos[0], center_pos[1])
            disp.show(img)
        debug_time("display img")
        
        # 终端实时打印FPS
        if print_fps_terminal:
            fps_counter += 1
            current_time = time.ticks_ms()
            if current_time - fps_last_time >= fps_interval:
                current_fps = time.fps()
                print(f"[FPS] 当前帧率: {current_fps:5.1f} fps | 帧计数: {fps_counter}")
                fps_counter = 0
                fps_last_time = current_time

def test_serial_communication():
    '''
    测试串口通信 - 发送8字节测试数据
    '''
    if not enable_serial_communication:
        print("[串口测试] 串口通信已禁用")
        return
    
    # 发送8字节测试数据帧
    test_data = [0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x09, 0xFF]  # 8字节测试帧
    
    try:
        print(f"[串口测试] 准备发送8字节测试数据:")
        print(f"  数据帧: {[hex(x) for x in test_data]}")
        print(f"  字节数: {len(test_data)}")
        
        # 确保是8字节
        if len(test_data) != 8:
            print(f"[串口测试] 错误：测试数据不是8字节")
            return
            
        data_bytes = bytes(test_data)
        bytes_sent = serial.write(data_bytes)
        print(f"[串口测试] 实际发送字节数: {bytes_sent}")
        
        if bytes_sent == 8:
            print(f"[串口测试] ✓ 成功发送完整8字节测试帧")
        else:
            print(f"[串口测试] ⚠️ 警告：期望发送8字节，实际发送{bytes_sent}字节")
            
        serial.flush()
        print(f"[串口测试] ✓ 测试数据发送完成")
    except Exception as e:
        print(f"[串口测试] ❌ 发送失败: {e}")

# 添加手动测试函数，可以在主循环外调用
def manual_test_send():
    '''
    手动测试发送函数 - 可以在程序中随时调用
    '''
    print("\n" + "="*50)
    print("手动测试串口发送")
    print("="*50)
    
    # 测试1：发送测试数据
    print("\n1. 发送测试数据:")
    test_serial_communication()
    
    # 测试2：发送黑线检测数据  
    print("\n2. 发送黑线检测数据:")
    send_blackline_detected()
    
    # 移除中心标定相关测试
    
    print("\n" + "="*50)