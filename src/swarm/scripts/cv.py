import cv2
import numpy 

class Vision():
    def __init__(self) -> None:
        with open("confi.txt","r") as file:
            line = file.readline()
            print(line.rstrip().split("%")[1].split(",")[0].split(";")[0].split(":"))
            self.startpointX1 = int(line.rstrip().split("%")[1].split(",")[0].split(";")[0].split(":")[1])
            self.startpointY1 = int(line.rstrip().split("%")[1].split(",")[0].split(";")[1].split(":")[1])
            self.startpointX2 = int(line.rstrip().split("%")[1].split(",")[1].split(";")[0].split(":")[1])
            self.startpointY2 = int(line.rstrip().split("%")[1].split(",")[1].split(";")[1].split(":")[1])
            self.areaelimination = int(file.readline().rstrip().split("%")[1])  
    def main(self,img:numpy.ndarray):
        imaProcessed = numpy.copy(img[self.startpointY1:self.startpointY2,self.startpointX1:self.startpointX2])
        imgGray = cv2.cvtColor(imaProcessed,cv2.COLOR_BGR2GRAY)
        imgGray = cv2.dilate(imgGray,(3,3),iterations=2)
        imgGray = cv2.erode(imgGray,(3,3),iterations=2)
        ret, thresh = cv2.threshold(imgGray, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print(contours)
        for con in contours:
            area = cv2.contourArea(con)
            if area < self.areaelimination:
                continue
            if con[0][0][0] == 0 or con[0][0][0] == self.startpointY2  or con[0][0][1] == 0 or con[0][0][1] == self.startpointX2:
                continue
            leftCont = con
        corner = cv2.minAreaRect(leftCont)
        box = cv2.boxPoints(corner)
        box = numpy.int0(box)

        return box