import cv2
import numpy as np
import sys
import csv
import time
import can
from ultralytics import YOLO
from datetime import datetime

#离线写法，只写模型名称如果本地路径检测不到会拉网络下载
model=YOLO(r"D:\embedded-ai\scripts\YOLO_Learning\yolov8n.pt")
cap=cv2.VideoCapture(r'D:\embedded-ai\练手数据集\videos\pedestrians.avi')
log_file = open(r"D:\embedded-ai\练手数据集\logs\events.csv",
                "w", newline="", encoding="utf-8")
writer = csv.writer(log_file)
writer.writerow(["time", "frame", "event", "confidence", "bbox"])
# 注意这个表达 if not (True/False)
if not cap.isOpened():print("打开失败");sys.exit()

#划区域
roi = [(300,150), (500,150), (500,310), (300,310)]  # 示例四个角点

#报警和解除标志
intrusion_frames=0    #连续有人在区域内帧数量
leave_frames=0          #连续无人在区域内帧数量
alarm_state = "NORMAL"  #报警状态

#报警截图编号
alarm_frame_id=0
#切换报警那一刻标志位，用来截图
just_alarm = False
#全局帧索引
frame_index=0
# 循环外累计：
precd_total=0.0
infer_total = 0.0
frame_count = 0
#配置can
bus=can.Bus(interface="pcan",channel="PCAN_USBBUS1",bitrate=500000)
last_cmd=0
cmd_seq=0
#心跳时间初始定义
next_hb_time = time.time() + 1.0   # 1秒后发第一次心跳
last_hb_recv = time.time()     # 启动先当在线，不然一启动就误报离线
while(True):
    ret,frame=cap.read()
    if not ret:print("读取失败");break #
    #cv2.polylines支持一次画多个多边形，所以第二个参数传numpy数组列表（对比判断函数cv2.pointPolygonTest）
    t1 = time.time()
    cv2.polylines(frame, [np.array(roi, np.int32)], True, (0,0,255), 2)
    t0 = time.time()
    results = model(frame, verbose=False)
    infer_total += time.time() - t0
    frame_count += 1
    res=results[0]
    persons = []
    for box in res.boxes:
        cls = int(box.cls)
        conf = float(box.conf)
        if cls == 0 and conf >= 0.5:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            cx=(x1+x2)/2
            cy=(y1+y2)/2
            persons.append((x1, y1, x2, y2, conf,cx,cy))
    #用来储存每帧在危险区域内的人数（box数）
    inside_num=0
    persons_inside=[]
    if persons:
        for x1, y1, x2, y2, conf, cx, cy in persons:
            scan_result = cv2.pointPolygonTest(np.array(roi, np.int32), (cx, cy), False)
            if scan_result > 0:
                persons_inside.append((x1, y1, x2, y2, conf))
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
                cv2.putText(frame, f"person {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
                inside_num+=1
            else:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
                cv2.putText(frame, f"person {conf:.2f}", (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)

    #报警状态参数累计与清零
    if inside_num: intrusion_frames+=1; leave_frames=0
    else: leave_frames+=1; intrusion_frames=0
    if alarm_state == "NORMAL" and intrusion_frames >= 5:
        alarm_state = "ALARM"
        just_alarm = True
    if alarm_state == "ALARM" and leave_frames >= 10:
        alarm_state = "NORMAL"
        writer.writerow([datetime.now().strftime("%H:%M:%S"), frame_index, "CLEAR", "", ""])
    #发送CAN
    current_cmd = last_cmd
    if alarm_state== "NORMAL":
        current_cmd=0
    elif alarm_state== "ALARM" and intrusion_frames < 30:
        current_cmd=1
    elif alarm_state== "ALARM" and intrusion_frames >= 30:
        current_cmd=2
    if current_cmd!=last_cmd:
        msg=can.Message(
            arbitration_id=0x202,
            data=[current_cmd,inside_num,cmd_seq,0,0,0,0,0],
            is_extended_id=False
            )

        send_ok=False
        for attempt in range(3):
            while bus.recv(timeout=0) is not None:
                pass
            this_seq = cmd_seq
            bus.send(msg)

            deadline = time.time() + 1.0      # 超时1.0秒
            got = False
            while time.time() < deadline:
                ack = bus.recv(timeout=0.1)   # 短轮询，先填0.1秒
                if ack and ack.arbitration_id == 0x301 and ack.data[0] == this_seq:
                    got = True
                    break
                # 不匹配的旧帧/别的帧，什么都不做，继续循环等
            if got:
                send_ok=True
                last_cmd=current_cmd
                print("序号匹配，命令被应答","第",attempt+1,"次", datetime.now().strftime("%H:%M:%S"))
                print("ACK ID:",hex(ack.arbitration_id),"序号：",this_seq,"状态：",ack.data[1],"错误码",ack.data[2])
                break

        if not send_ok:
            print("尝试3次仍失败，序号为：", this_seq,datetime.now().strftime("%H:%M:%S"))
        cmd_seq+=1
    #显示状态
    if alarm_state == "ALARM":
        color = (0, 0, 255)      # 红
    else:
        color = (0, 255, 0)      # 绿（NORMAL）
    cv2.putText(frame, alarm_state, (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 2)
    
    #截图切换报警状态帧
    if just_alarm:
        #遍历persons_inside列表的每个元组，比较[4]索引值取最大，返回整个元组
        best_person = max(persons_inside, key=lambda p: p[4])
        best_conf = best_person[4]
        bbox = f"({best_person[0]},{best_person[1]},{best_person[2]},{best_person[3]})"

        cv2.imwrite(rf"D:\embedded-ai\练手数据集\shortcuts\whole\alarm_{alarm_frame_id}.jpg",frame)
        alarm_frame_id+=1
        writer.writerow([
                    datetime.now().strftime("%H:%M:%S"),
                    frame_index,
                    "ALARM",
                    best_conf,          # 这一帧在内人员的置信度（取最高那个）
                    bbox           # 位置，比如 "(x1,y1,x2,y2)"
                ])
        just_alarm = False     # 立刻清掉，下一帧就不再存

    precd_total += time.time()-t1
    #定时1秒发心跳
    PC_id=1
    STM32_id=2
    off_line=1
    on_line=0
    if time.time()>=next_hb_time:
        hb=can.Message(
            arbitration_id=0x7F0,
            data=[PC_id,on_line,0,0,0,0,0,0],
            is_extended_id=False
            )
        bus.send(hb)
        next_hb_time+=1
    #循环接收心跳回包
    while True:
        hb_rx=bus.recv(timeout=0)
        if hb_rx is None:
            break
        if hb_rx.arbitration_id == 0x7F0:
            last_hb_recv=time.time()
    offline_printed = False
    if time.time() - last_hb_recv > 3.0:
        text = "设备离线"
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale, thick = 1.5, 3
        (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
        x = (frame.shape[1] - tw) // 2
        y = (frame.shape[0] + th) // 2+200
        cv2.putText(frame, text, (x, y), font, scale, (0,0,255), thick)
        cv2.imshow("player", frame)      # 画完立刻刷新一次画面
        while True:
            hb_rx=bus.recv(timeout=0)
            if hb_rx and hb_rx.arbitration_id==0x7F0:
                last_hb_recv=time.time()
            if time.time()-last_hb_recv < 1.0:
                break
            if cv2.waitKey(100) & 0xFF == ord('q'):   # 卡住时也能按q退出
                break
            print("设备连接异常，请检查！")
            
    #显示画面
    cv2.namedWindow("player", cv2.WINDOW_NORMAL)
    cv2.imshow("player",frame)

    # 1000ms/视频帧率=帧间时间设定，但是实际程序循环跑也需要毫秒时间
    # &0xFF是跨平台写法，只取低八位进行按键检测，去除高八位冗余干扰信息
    if cv2.waitKey(100) & 0xFF == ord('q'):break
    frame_index += 1
bus.shutdown()
avg_ms = infer_total / frame_count * 1000   # 毫秒/帧
pre_avg_ms = precd_total / frame_count * 1000   # 毫秒/帧
print(f"平均推理 {avg_ms:.5f} ms/帧，理论上限 {1000/avg_ms:.3f} FPS")
print(f"单帧总耗时 {pre_avg_ms:.5f} ms/帧，理论上限 {1000/pre_avg_ms:.3f} FPS")
cap.release()
log_file.close()
cv2.destroyAllWindows()
