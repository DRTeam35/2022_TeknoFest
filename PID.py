class PID_class():
    def __init__(self,last_error,pid_i):
        self.yaw_pid_i=pid_i
        self.yaw_last_error=last_error
        self.yaw_control_signal=0.0
    def resetPidVariables(self):
        self.yaw_pid_i=0.0
        self.yaw_last_error=0.0
    def calculateMotorPowers(self,KP_yaw,KI_yaw,KD_yaw,istenilen_hız,istenilen_acı,imu_yaw,imu_deltaTime):
        yawError=istenilen_acı-imu_yaw
        self.yaw_control_signal= self.getControlSignal(yawError, KP_yaw, KI_yaw, KD_yaw, self.yaw_pid_i, self.yaw_last_error, imu_deltaTime)

        motorpower=[]
        motorpower[0]=istenilen_hız+self.yaw_control_signal
        motorpower[1]=istenilen_hız-self.yaw_control_signal
        motorpower[2]=self.yaw_last_error
        motorpower[3]=self.yaw_pid_i
        if(motorpower[0]>350):
            motorpower[0]=350
        elif (motorpower[1]>350):
            motorpower[1]=350
        elif (motorpower[0]<286):
            motorpower[0]=286
        elif (motorpower[1]<286):
            motorpower[1]=286
        return motorpower
    def getControlSignal(self,error,kp,ki,kd,delta_time):
        pid_p=error
        pid_d=(error-self.yaw_last_error)/delta_time
        self.yaw_pid_i+=error*delta_time
        control_signal=(kp*pid_d)+(ki*self.yaw_pid_i)+(kd*pid_d)
        self.yaw_last_error=error
        return control_signal
