class PID_Kontrol:
    def __init__(self, Kp = 1, Kd = 0, Ki = 0, SetPoints = 0, InMin = 0, InMax = 0, OutMin = 0, OutMax = 0, Ti = 0.001, Td = 0.001, DrawPlot=False):
        """
        PID KONTROL - KRSBI HUMANOID v3

        :param Kp:
        :param Kd:
        :param Ki:
        :param SetPoints:
        :param SetSamplingTime:
        :param Ti:
        :param Td:
        """
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

        self.Ti = Ti
        self.Td = Td

        self.Output = 0

        self.Sp = SetPoints
        # self.SamplingTime = SetSamplingTime

        self.InRangeMin = InMin
        self.InRangeMax = InMax
        self.OutRangeMin = OutMin
        self.OutRangeMax = OutMax

        self.windup_limit = "DISABLE"
        self.windup_crossing = "DISABLE"

        self.isDrawPlotEnable = DrawPlot

    def Init(self):
        self.currTime = time.time()
        self.prevTime = self.currTime

        self.error = 0
        self.prevError = 0
        self.sumError = 0

        self.cP = 0
        self.cI = 0
        self.cD = 0

        self.lastCurrTime_Ti = 0
        self.lastCurrTime_Td = 0

    def setEnableWindUpLimit(self):
        self.windup_limit = "ENABLE"

    def setDisableWindUpLimit(self):
        self.windup_limit = "DISABLE"

    def setEnableWindUpCrossing(self):
        self.windup_crossing = "ENABLE"

    def setDisableWindUpCrossing(self):
        self.windup_crossing = "DISABLE"

    def setSetPoints(self, Sp):
        self.Sp = Sp
    def setError(self, Error):
        self.error = Error

    def setRange(self, InMin, InMax, OutMin, OutMax):
        self.InRangeMin = InMin
        self.InRangeMax = InMax
        self.OutRangeMin = OutMin
        self.OutRangeMax = OutMax

    def setKonstanta(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

    def getCurrentTime(self):
        return int(self.currTime)
    def getPrevTime(self):
        return int(self.prevTime)
    def getOuput(self):
        return self.Output

    def getError(self):
        return  self.error

    def kalkulasi(self, FeedBack):
        """
        KALKULASI PID KONTROL
        DENGAN PARAMETER nilai ERROR dari FEEDBACK
        :param error:
        :param sleep:
        :return:
        """

        self.currTime = int(time.time() * 1000)# S => mS
        deltaTime = self.currTime - self.prevTime       #dt
        deltaError = self.error - self.prevError        #de

        self.error = (self.Sp - FeedBack) / (self.InRangeMax - self.InRangeMin)

        self.cP = self.error
        # self.cI += error * deltaTime
        # self.cD = (deltaError / deltaTime) if deltaTime > 0 else 0
        self.cI = self.sumError
        self.cD = deltaError


        self.Output = sum([ self.Kp * self.cP,
                            self.Ki * self.cI,
                            self.Kd * self.cD])

        self.Output = self.Output * (self.OutRangeMax - self.OutRangeMin)

        if self.Output > self.OutRangeMax:
            self.Output = self.OutRangeMax
        else:
            if self.Output < self.OutRangeMin:
                self.Output = self.Output
            else:
                self.Output = self.Output

        if self.currTime - self.lastCurrTime_Ti > self.Ti:
            self.lastCurrTime_Ti = self.currTime

            if self.windup_limit == "ENABLE":  # ANTI WINDUP LIMIT - GUARD
                self.sumError = self.sumError + self.error
                if self.sumError > 1.0:
                    self.sumError = 1.0
                else:
                    if self.sumError < -1.0:
                        self.sumError = -1.0
                    else:
                        self.sumError = self.sumError

        if self.currTime - self.lastCurrTime_Td > self.Td:
            self.lastCurrTime_Td = self.currTime
            self.prevError = self.error

        if self.windup_crossing == "ENABLE":  # ANTI WINDUP CROSSING - GUARD
            if self.prevError * self.error < 0:
                self.sumError = 0

        self.prevTime = self.currTime
        self.prevError = self.error
        self.Output = self.Output

        return  self.Output

    def reset(self):
        self.prevError = 0
        self.sumError = 0
        self.Output = 0
