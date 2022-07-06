#Daire Görevi
#Mission_2 icin kamera acısı 292
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
    #İlk kamera işlemleri.
    ret, frame = cap.read()
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)
    shape = frame.shape #shape[0] = y coord , shape[1] = x coord
    #Araç kamerası için Merkez noktası belirlenir range1=[x_center,y_center]
    range_center = shape[1]/2 , shape[0]/2

    original_frame = frame.copy()
    thresh = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # converting gray scale
    median = cv2.medianBlur(thresh, 5)  # blur operation
    sub = cv2.subtract(median, thresh)  # subtraction operation on images for the clearest image
    cnt, hierarchy = cv2.findContours(sub, 2, 1)  # finding contours in sub image
    big_contour = []  # the largest counter
    max = 0  # its area
    for i in cnt:
        area = cv2.contourArea(i)  # finding the contour having biggest area
        if area <= max:
            continue
        max = area
        big_contour = i
    leng = len(big_contour)  # length of contour array

    l = int(leng / 2)  # 1/2 of the contour length (to calculate y coordinate of the ellipse)
    k = int(leng / 4)  # 1/4 of the contour length (to calculate x coordinate of the ellipse)
    m = leng - k  # 3/4 of the contour length (to calculate x coordinate of the ellipse)
    # x1 y1 x2 y2 of the biggest counter
    xx1 = big_contour[m][0][0]
    yy1 = big_contour[l][0][1] 
    xx2 = big_contour[k][0][0]
    yy2 = big_contour[0][0][1]
    en = abs(yy1 - yy2)
    boy = abs(xx1 - xx2)
    min_y = min(yy1, yy2)

    ###################### calculating center of the found coordinates
    center = int((xx1 - xx2) / 2), int((yy1 - yy2) / 2)

    # height width, channel of the frame
    # hgt, wdt, _ = (original_frame.shape)

    # this is a line or smt else
    if min_y < 0 and min_y < 150:
        center2 = [None, None]
        support.video_record(original_frame,out_i,fps_i)
        return range_center, center2,4,0  # THERE IS NO DETECTION son değerler sırayla sabitlenmek istenen basınç değeri,Area

    if center[0] < 40 or center[1] < 10:
        center2 = [None, None]
        support.video_record(original_frame,out_i,fps_i)
        return range_center, center2,4,0  # THERE IS NO DETECTION son değerler sırayla sabitlenmek istenen basınç değeri,Area

    ###################### this part is a pre-filter
    # alan biraz buyuk olsun
    # eni boydan buyuk elips mi olur, olmaz
    if (en < 10) or ((en * boy) < 1000) or (en > boy): # this is a lanet olasi Line
        center2=[None,None]
        support.video_record(original_frame,out_i,fps_i)
        return range_center,center2,4,0# THERE IS NO DETECTION son değerler sırayla sabitlenmek istenen basınç değeri,Area

    if center[0] < 35 and center[1] < 35: # bu da oyle iste
        center2=[None,None]
        support.video_record(original_frame,out_i,fps_i)
        return range_center,center2,4,0 # THERE IS NO DETECTION son değerler sırayla sabitlenmek istenen basınç değeri,Area
    else:
        # k[0] + middle[0] to calculate x coordinate, 0[0] + middle[1] to calculate y coordinate
        center2 = big_contour[k][0][0] + center[0], big_contour[0][0][1] + center[1]

        # limits top of the frame, 1/4 part of it
        if center2[1] < 120:
            center2 = [None, None]
            support.video_record(original_frame,out_i,fps_i)
            return range_center,center2,4,0  # THERE IS NO DETECTION son değerler sırayla sabitlenmek istenen basınç değeri,Area

        final = cv2.drawContours(frame, big_contour, -1, (0, 0, 255), 2)  # drawing contour
        # bgr value of the coordinate
        final = cv2.circle(final, center2, 3, (0, 255, 255), 3)  # drawing center point of contour we found

        red_filtered = red_filter(final)
        is_line = line(red_filtered)
        is_ellipse = pre_ellipse(red_filtered)

        if is_line == 0 and is_ellipse == 1: # DETECTED!!!
            # limits 48000
            area_circle=int(en*boy)
            cv2.putText(final,str(area_circle),(0,80),1,1,(255,255,255),1)
            support.video_record(final,out_i,fps_i)
            return range_center,center2,4,area_circle #son değerler sırayla sabitlenmek istenen basınç değeri,Area
        else:
            # THERE IS NO DETECTIn
            center2=[None,None]
            support.video_record(original_frame,out_i,fps_i)
            return range_center,center2,4,0 #son değerler sırayla sabitlenmek istenen basınç değeri,Area

def red_filter(frame): 
    red_high = np.array([50, 100, 255])
    red_low = np.array([0, 0, 50])
    mask = cv2.inRange(frame, red_low, red_high)
    result = cv2.bitwise_and(frame, frame, mask=mask)
   
    return result # returns only the red painted part!

def line(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # converting gray scale
    # Edge detection
    dst = cv2.Canny(gray, 50, 200, None, 3)
    # Standard Hough Line Transform
    lines = cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)
    # Draw the lines
    if lines is None: # there is no line
        return 0
    if lines is not None:
        for i in range(0, len(lines)):           
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a))) # x1, y1
            pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a))) # x2, y2
            # here it measures if difference between y points < 600
            # and difference between x points < 2000
            # bunlar line diye tespit ettigi ancak aslinda line olmayan, elips olan
            # abs(y1-y2) > 500 or abs(x1-x2) > 1700
            if abs(pt1[1]-pt2[1]) > 500 or abs(pt1[0]-pt2[0]) > 1700:
                return 1 # There is a funcking LINE!
    return 0 # there is no line

def pre_ellipse(frame):
    if frame is None:
        return 0
    src_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    src_gray = cv2.blur(src_gray, (3, 3))
    thresh = 75
    return ellipse_check(thresh, src_gray)

def ellipse_check(threshold, src_gray):
    canny_output = cv2.Canny(src_gray, threshold, threshold * 2)

    # Originally this part expects 3 arguments but it is not 2,
    # Therefore i have modified as follows
    contours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours is None: ######### There is no ellipse!
        return 0
    return 1

#Bu kod RAL 3026 kırmızı rengi için filtreleme uygular.
#Eğer resimde kırmızı varsa bu kırmızı renginin orta merkezini bulur.
#geri döngü olarak frame ve renk yoğunluğunun merkezini döndürür.
def afk(cap_a,out_a,access=1):
    #Alt kameradan image alınır ve frame değişkenine atanır.
    #Burada testlerin durumuna göre kamera döndürülmesi veya ayna görüntüsünün alınması gerekebilir
    ret, frame = cap_a.read()
    shape = frame.shape #shape[0] = y coord , shape[1] = x coord
    #Araç kamerası için Merkez noktası belirlenir range1=[x_center,y_center]
    frame_center = shape[1]/2 , shape[0]/2
    #center2=[None,None] atanır.Bulunamazsa eğer geri bu değer döndürülür
    
    center2=[None,None] # this is for non-detected case, default
    #RAL3026 için üst treshold seviyesi
    red_high = np.array([70, 255, 255])
    #RAL3026 için alt treshold seviyesi
    red_low = np.array([0, 0, 110])
    #Kırmızı rengi için maskeleme yapılır.
    mask = cv2.inRange(frame, red_low, red_high)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    #maskelenen görüntüde contour çıkarılır.
    # find contours in the mask and initialize the current
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    # (x, y) center
    center2_2 = None # this is for detected case
    #Bu döngüde bulunan contourların alanları ölçülür. Alanı belli sviyenin üzerindeki contourların merkezi bulunur daha sonra 
    #bu merkez center2 değişkenine atanır. En so olarak fonksiyon geriye işlenmiş görüntüyü ve kırmızı renginin merkezini döndürür
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        # to avoid float / zero condition
        if (M["m00"] == 0):
            center2_2 = center2
        else:
            center2_2 = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # print("center: ", center2_2)

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # print("radius: ", radius)
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center2_2, 3, (0, 0, 255), -1)
            if access==1:
                support.video_record(frame,out_a)
            return frame_center, center2_2
    if access==1:
        support.video_record(frame,out_a)
    return frame_center,center2

#############################################------------------------------------Gudumlenme--------------------------------------###################################################
'''
Öyle İşte
'''

def fatality_gudumlenme(center_fg,pwm_fg,frame_center_fg):
    #Merkezi Tolerans
    center_tol=140
    range2 = frame_center_fg[1] -center_tol, frame_center_fg[1] +center_tol
    #Kamera merkezinde x-ekseni için 50 piksel toleranslık alan belirlenir. Araç tespit ettiği nesnenin
    #merkezini bu kümeye sokmaya çalışır.
    range3= frame_center_fg[0]-center_tol,frame_center_fg[0]+center_tol
    control.Pressure_Control(4,pwm_fg)

    #############################################--X_ekseni merkezleme---###########################################
    #Eğer nesneni merkezi kümenin solundaysa sol yengeç hareketi gerçekleştirilir
    if center_fg[0] > range3[1]:
        #sola git
        motor_function.motor_r_yengec(pwm_fg)
    #Eğer nesneni merkezi kümenin sağındaysa sağ yengeç hareketi gerçekleştirilir
    elif center_fg[0] < range3[0]:
        #sağa git
        motor_function.motor_l_yengec(pwm_fg)
    #Eğer nesnenin merkezi ,x-ekseni için belirlenen tolerans kümesi içindeyse araç ileri doğru gider.
    #------------------------------------------------------Artık Bitti-----------------------------------------------------------#
    #Burada kırmızı rengi x koordinatında merkezlenebildiyse artık aracı dibe batırır ve motorları kapatır veeeeeee koduda kapatır.
    elif center_fg[0] > range3[0] and center_fg[0] < range3[1]:
        motor_function.stop(pwm_fg)
        time.sleep(1)
        motor_function.motor_d(pwm_fg,285)
        while True:
            sensor=control.gyro()
            preassure=float(sensor[1])
            #11 kPascal altıysa baatmıştıt artık be
            #Bu arada çok yoruldum bu kod sittin sene çalışmaz
            if preassure>float(31):
                motor_function.stop(pwm_fg)
                time.sleep(1)
                #Motor gücünü kes
                #hheheheheheh Kimse okumuyor zaten
                motor_function.start(0)
                #Kodu kapat
                exit()
   
def fatality(pwm_f,cap_alt,cap_on,out_f):#ön kamera objesi lazım,
    #ileri gideceği süre
    sec_ang=12
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
    pwm_f.set_pwm(0, 0, 225)
    while Second_x <= sec_ang:
        #Buraya PID gelecek######
        #Görüntü işleme yapılıyor Geri döngü olarak hedefin merkezi alınıyor.
        #Geri döngü olarak işlenmiş görüntü ve hedef merkezi alınyor
        #frame_g=image , center_g[x,y]
        frame_center,center_f=afk(cap_alt,out_f,0)
        #Görüntü işleme eğer tespit yapamazsa None döndürür.
        #Eğer tespit yapılırsa güdümlenme döngüsüne girer
        if center_f[1]!=None:
            motor_function.stop(pwm_f)
            while center_f[1]!=None:
                #merkez konumu değerlendirme amacıyla güdümlenme fonksiyonuna gönderilir
                fatality_gudumlenme(center_f,pwm_f,frame_center)
                #kayıt alınır
                #Yeni merkez noktasını bulmak amacıyla görüntü işlenir.
                frame_center,center_f=afk(cap_alt,out_f,1)
            motor_function.stop(pwm_f)
        elif center_f[1]==None:
            frame_center_circle,center_g,pressure,Area_on=image_process(cap_on,out_f)
            if center_g[1]!=None:
                move,pres_f =gudumlenme(center_g,pwm_f,frame_center_circle,out_f,cap_alt,cap_on,Area_on,move,0)
            else:
                motor_function.motor_fri(pwm_f,351)                               
                
        #Anlık zaman değeri alınır
        stop =time.time()
        #Başlangıç zamanının anlık zamana göre farkı alınarak Second_x değişkenine atanır
        Second_x=stop-start
    #Burada kırmızı rengi x koordinatında merkezlenebildiyse artık aracı dibe batırır ve motorları kapatır veeeeeee koduda kapatır.
    motor_function.stop(pwm_f)
    time.sleep(1)
    motor_function.motor_d(pwm_f,285)
    while True:
        ret, frame_finish = cap_alt.read()
        support.video_record(frame_finish,out_f)
        sensor=control.gyro()
        preassure=float(sensor[1])
        #11 kPascal altıysa baatmıştıt artık be
        if preassure>float(31):
            motor_function.stop(pwm_f)
            time.sleep(1)
            #Motor Gücü kes
            motor_function.start(0)
            #programı kapat
            exit()

def gudumlenme(center_2,pwm_g,range_center_g,out_g,cap_g,cap_gon,Area_g,move=1,activity=1):
    #Merkezi Tolerans
    center_tol=120
    pressure=4
    #Kamera merkezinde y-ekseni için 50 piksel toleranslık alan belirlenir. Araç tespit ettiği nesnenin
    #merkezini bu kümeye sokmaya çalışır.
    range2 = range_center_g[1] -center_tol, range_center_g[1] +center_tol
    #Kamera merkezinde x-ekseni için 50 piksel toleranslık alan belirlenir. Araç tespit ettiği nesnenin
    #merkezini bu kümeye sokmaya çalışır.
    range3= range_center_g[0]-center_tol,range_center_g[0]+center_tol
        
    #############################################--X_ekseni merkezleme---###########################################
    #Eğer nesneni merkezi kümenin solundaysa sol yengeç hareketi gerçekleştirilir
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
        if Area_g>35000 and activity==1:
            fatality(pwm_g,cap_g,cap_gon,out_g)
            
    return move,pressure
