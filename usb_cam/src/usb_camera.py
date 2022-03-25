#!/usr/bin/env python
#-*- coding:utf-8   -*-
import numpy as np
import cv2
import time
import rospy

cap = cv2.VideoCapture("/dev/video0")
weight=640
height=480
cap.set(3, weight)  # 设置分辨率 3和4 分别代表摄像头的属性值。你可以使用函数 cap.get(propId) 来获得视频的一些参数信息。这里propId 可以是 0 到 18 之间的任何整数。每一个数代表视频的一个属性,见表其中的一些值可以使用cap.set(propId,value) 来修改,value 就是你想要设置成的新值。例如,我可以使用 cap.get(3) 和 cap.get(4) 来查看每一帧的宽和高。默认情况下得到的值是 640X480。但是我可以使用 ret=cap.set(3,320)和 ret=cap.set(4,240) 来把宽和高改成 320X240。
cap.set(4, height)
cap.set(15,0.1)
codec = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')
#print(codec)

cap.set(cv2.CAP_PROP_FOURCC, codec)
fps =cap.get(cv2.CAP_PROP_FPS) #获取视频帧数
cap.set(cv2.CAP_PROP_AUTOFOCUS, False)  # 禁止自动对焦
cap.set(cv2.CAP_PROP_SETTINGS, 1) 
b_fps=time.time()  #后帧时间全局变量赋值
image_path  = '/home/bo/ucar_cam/image_data/data9/'
count = 0

#rate = rospy.Rate(20)
while(True):
    # 读取一帧
    f_fps=time.time() #前帧时间
    fps_now=str(round(1/(f_fps-b_fps),2))   #求当前帧率
    b_fps=f_fps #后帧时间
    ret, frame = cap.read()
    frame = cv2.flip(frame,0)   ##图像左右颠倒
    cv2.putText(frame, 'FPS:' + ' ' + fps_now, (10, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 2, cv2.LINE_AA)
    # h, w = frame.shape[:2]
    # print(h, w)
    # print("获得的帧率:", fps)
    # cv2.imshow('Camera_USB', frame)


    # cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    count += 1
    timestr = "%d" % count
    # %.6f表示小数点后带有6位，可根据精确度需要修改；
    image_name = timestr + ".jpg"  # 图像命名：时间戳.jpg
    print("图片保存成功:" + image_name)
    cv2.imwrite(image_path + image_name, frame)  # 保存；
    cv2.imshow('Camera_USB', frame)
    cv2.waitKey(3)
    # rate.sleep()
    if cv2.waitKey(1) & 0xFF == 27:
        cap.release()
        cv2.destroyAllWindows()
        break


