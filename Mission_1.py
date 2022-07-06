#Daire Görevi

##### Mission_1 icin kamera açısı 315
import cv2
import numpy as np
import math
import motor_function
import control
import time
import support

# image_process() returns the last version of the frame
# red_filter() filters red color, returns frame
# line() # returns 0 - no line or 1 - line
# pre_ellipse() # pre-process of ellipse, returns ellipse_check()
# ellipse_check() # returns 0 - no ellipse or 1 - ellipse
# gudumlenme
#############################################------------------------------------Görüntü işleme--------------------------------------###################################################
def image_process(cap,out_i,fps_i="0"):# bura Ece nin kod

#Bu kod RAL 3026 kırmızı rengi için filtreleme uygular.
#Eğer resimde kırmızı varsa bu kırmızı renginin orta merkezini bulur.
#geri döngü olarak frame ve renk yoğunluğunun merkezini döndürür.
# this function returns frame, center
    # copy the original frame
    ret, frame = cap.read()
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)
    shape = frame.shape #shape[0] = y coord , shape[1] = x coord
    #Araç kamerası için Merkez noktası belirlenir range1=[x_center,y_center]
    range_center = shape[1]/2 , shape[0]/2
   
    original_one = frame.copy()

    # convert to hsv for masking
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # mask limits
    yellow_high = np.array([95, 255, 220])
    yellow_low = np.array([55, 160, 0])
    mask = cv2.inRange(hsv, yellow_low, yellow_high)
    cv2.imshow("mask", mask)

    # find contours in the mask and initialize the current
    # (x, y) center
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and centroid

        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        str_radius = "radius: " + str(int(radius))
        cv2.putText(frame, str_radius, (0, 120), 1, 1, (0, 255, 0), 1)

        # area of the kuka contour
        area = "area: " + str(cv2.contourArea(c))
        area_num=cv2.contourArea(c)
        cv2.putText(frame, area, (0, 135), 1, 1, (0, 255, 0), 1)

        # bounding rectangle around the contour
        box_x, box_y, box_w, box_h = cv2.boundingRect(c)
        center = (box_x + int(box_w/2), box_y + int(box_h/2))
        bounding_box = "box width: " + str(int(box_w)) + " box height: " + str(int(box_h))
        cv2.rectangle(frame, (box_x, box_y), (box_x + box_w, box_y + box_h), (0, 255, 0), 2)
        cv2.circle(frame, center, 3, (25, 25, 200), -1)
        cv2.putText(frame, bounding_box, (0, 150), 1, 1, (0, 255, 0), 1)

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            #cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            #cv2.circle(frame, center, 3, (0, 0, 255), -1)
            # Edge detection
            dst = cv2.Canny(mask, 50, 200, None, 3)
            #  Standard Hough Line Transform
            lines = cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)
            # Draw the lines
            # store lines in line_array
            line_array = [0]
            if lines is not None:
                for i in range(0, len(lines)):
                    rho = lines[i][0][0]
                    theta = lines[i][0][1]
                    a = math.cos(theta)
                    b = math.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                    pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                    # if line is below 2/3 of y axis of frame, continue
                    if pt1[1] > (frame.shape[0] - int(frame.shape[0]/3)):
                        continue
                    # if this is first time, insert first detected line
                    if line_array[0] == 0:
                        line_array[0] = pt1[1]
                        cv2.line(frame, pt1, pt2, (0, 255, 255), 1, cv2.LINE_AA)
                    # if there is already a line in the array, compare
                    else:
                        for j in range(0, len(line_array)):
                            # if y axis of new line is at least +-100, insert new line
                            if pt1[1] > line_array[j] + 100 or pt1[1] < line_array[j] - 100:
                                line_array.append(pt1[1])
                                cv2.line(frame, pt1, pt2, (0, 0, 255), 1, cv2.LINE_AA)

            # if there is more than 1 line
            if len(line_array) > 1:
                # calculate middle y axis of two lines
                mid_y = int((line_array[0] + line_array[1])/2)
                # if middle point is smaller than box center, this is our new y axis!
                if mid_y <= center[1]:
                    cv2.circle(frame, (center[0], mid_y), 3, (255, 2, 200), -1)
                    # new center here!
                    center = (center[0], mid_y)
                    support.video_record(frame,out_i,fps_i)
                    return range_center, center,20,area_num

            # DETECTED!
            support.video_record(frame,out_i,fps_i)
            return range_center, (center[0], None),20,area_num
        else:
            # radius is too small
            center = [None, None]
            support.video_record(original_one,out_i,fps_i)
            return range_center, center,20,0
    # there is no contour
    else:
        cv2.putText(frame, "No Contour Detected!", (0, 180), 1, 1, (0, 255, 0), 1)
        center = [None, None]
        support.video_record(original_one,out_i,fps_i)
        return range_center, center,20,0
#############################################------------------------------------Gudumlenme--------------------------------------###################################################

def fatality(pwm_f,cap_alt,cap_on,out_f,press):#ön kamera objesi lazım,
    #ileri gideceği süre
    sec_ang=20
    start = time.time()
    #Anlık Zaman okunur
    stop = time.time()
    #Second_x Başlangıç zamanının anlık zamana göre farkının atanması için oluşturulur
    #Başlangıç olarak 0 atanır.
    Second_x=0
    #While döngüsü fonksiyonun giriş değeri olan sec_ang ile geçen süre Second_x'i değerlendirir
    #Geçen süre, istenilen süreyi geçince fonksiyon durdurulur
    motor_function.motor_fri(pwm_f,351)
    center_g=[None,None]
    center_f=[None,None]

    move=0
    while Second_x <= sec_ang:
        #Buraya PID gelecek######
        #Görüntü işleme yapılıyor Geri döngü olarak hedefin merkezi alınıyor.
        #Geri döngü olarak işlenmiş görüntü ve hedef merkezi alınyor
        #frame_g=image , center_g[x,y]

        
        frame_center_circle,center_g,pressure,Area_on=image_process(cap_on,out_f)
        #Görüntü işleme eğer tespit yapamazsa None döndürür.
        #Eğer tespit yapılırsa güdümlenme döngüsüne girer
        if center_g[1]!=None:
            motor_function.stop(pwm_f)
            while center_g[1]!=None:
                #merkez konumu değerlendirme amacıyla güdümlenme fonksiyonuna gönderilir
                move,press =gudumlenme(center_g,pwm_f,frame_center_circle,out_f,cap_alt,cap_on,Area_on,move,0)
                control.Pressure_Control(press,pwm_f)
                #kayıt alınır
                #Yeni merkez noktasını bulmak amacıyla görüntü işlenir.
                frame_center_circle,center_g,pressure,Area_on=image_process(cap_on,out_f)          
            motor_function.motor_fri(pwm_f,351)
            
        control.Pressure_Control(press,pwm_f)
        #Anlık zaman değeri alınır
        stop =time.time()
        #Başlangıç zamanının anlık zamana göre farkı alınarak Second_x değişkenine atanır
        Second_x=stop-start
    #Burada kırmızı rengi x koordinatında merkezlenebildiyse artık aracı dibe batırır ve motorları kapatır veeeeeee koduda kapatır.
    motor_function.stop(pwm_f)
    time.sleep(1)
    motor_function.motor_u(pwm_f,380)
    while True:
        ret, frame_finish = cap_on.read()
        support.video_record(frame_finish,out_f)
        sensor=control.gyro()
        preassure=float(sensor[1])
        #11 kPascal altıysa baatmıştıt artık be
        if preassure>float(4):
            motor_function.stop(pwm_f)
            time.sleep(1)
            #Motor Gücü kes
            motor_function.start(0)
            #programı kapat
            exit()

def gudumlenme(center_2,pwm_g,range_center_g,out_g,cap_g,cap_gon,Area_g,move=1,activity=1):
    #Merkezi Tolerans
    center_tol=50
    center_tol_y=100
    f = open('pressure.txt', "r")
    # Son kaydedilen basınc değeri ölçülür
    pressure = f.read()
    f.close()
    pressure=float(pressure)
    #Kamera merkezinde y-ekseni için 50 piksel toleranslık alan belirlenir. Araç tespit ettiği nesnenin
    #merkezini bu kümeye sokmaya çalışır.
    range2 = range_center_g[1] -center_tol_y, range_center_g[1] +center_tol_y
    #Kamera merkezinde x-ekseni için 50 piksel toleranslık alan belirlenir. Araç tespit ettiği nesnenin
    #merkezini bu kümeye sokmaya çalışır.
    range3= range_center_g[0]-center_tol,range_center_g[0]+center_tol
        
    #############################################--X_ekseni merkezleme---###########################################
    #Eğer nesneni merkezi kümenin solundaysa sol yengeç hareketi gerçekleştirilir
    if center_2[1]!=None and Area_g>300:
        if center_2[1] < range2[0]:
            motor_function.motor_u(pwm_g,380)
            pressure=0
        elif center_2[1] > range2[1]:
            motor_function.motor_d(pwm_g,300)
            pressure=0 # Basıınç sabitlemeyi kapatır,
        else:
            # pressure.txt belgesi 'w' formatında açılır
            sensor=control.gyro()
            pressure=float(sensor[1])
            f = open('pressure.txt', "w")
            # Yeni alına basınc bilgisi pressure.txt'ye yazdırılır
            f.write(sensor[1])
            # dosya kağatılır
            f.close()
    
        if center_2[0] > range3[1] and center_2[1] > range2[0] and center_2[1] < range2[1]:
            #sola git
            motor_function.r_donus(pwm_g,321)
            move=1
        #Eğer nesneni merkezi kümenin sağındaysa sağ yengeç hareketi gerçekleştirilir
        elif center_2[0] < range3[0] and center_2[1] > range2[0] and center_2[1] > range2[1]:
            #sağa git
            motor_function.l_donus(pwm_g,321)
            move=1
        elif center_2[0] > range3[0] and center_2[0] < range3[1] and center_2[1] > range2[0] and center_2[1] > range2[1]:
            motor_function.motor_fri(pwm_g,351)
            move=0
            if Area_g>32000 and activity==1:
                fatality(pwm_g,cap_g,cap_gon,out_g,pressure)
    else:
        if center_2[0] > range3[1]:
            #sola git
            motor_function.r_donus(pwm_g,321)
            move=1
        #Eğer nesneni merkezi kümenin sağındaysa sağ yengeç hareketi gerçekleştirilir
        elif center_2[0] < range3[0]:
            #sağa git
            motor_function.l_donus(pwm_g,321)
            move=1
    #Eğer nesnenin merkezi ,x-ekseni için belirlenen tolerans kümesi içindeyse araç ileri doğru gider.
    #-----------------------------------------------------------------------------------------------------------------#
        elif center_2[0] > range3[0] and center_2[0] < range3[1]:
            motor_function.motor_fri(pwm_g,351)
            move=0
            '''
            Burata son aşamaya geçiş için şart belirtilecek ve sonra fatality fonksiyonuna girilecek
            fatality(pwm_g,cap_g,out_g)
            '''
            if Area_g>2000 and activity==1:
                fatality(pwm_g,cap_g,cap_gon,out_g,pressure)
            
    return move,pressure
