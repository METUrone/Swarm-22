import cv2 as cv 
import numpy as np
import threading as thrd

class fireDetecter():
    def __init__(self)->None:
        with open("config.txt","r") as file:
            a = file.readline()
            b = file.readline()
            c = file.readline()
            d = file.readline()
            self.colorElimination = int(d.split(":")[1])
            self.console = 1
            alist = a.split(":")[1]
            blist = b.split(":")[1]
            self.verticalDivision = int(c.split(":")[1]) 
            areaElimination = int(alist[1])
            print(blist)
            self.x1 ,self.y1 ,self.x2, self.y2 =blist.split(",")
            self.cap = cv.VideoCapture(0)
    def multiThread(self):
        a = thrd.Thread(target=self.readIMG)
        a.start()
    def readIMG(self):
        while self.cap.isOpened():
            ret ,self.frame = self.cap.read() 
            self.frame = self.frame[int(self.y1):int(self.y2),int(self.x1):int(self.x2)]
            if self.console == 0:
                break
            cv.waitKey(5)
    def main(self,image:np.ndarray):
        kernel = (3,3)
        imgray = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
        imgray = cv.dilate(imgray, kernel)
        imgray = cv.erode(imgray, kernel)
        ret,threshimg = cv.threshold(imgray,127,255,cv.THRESH_BINARY)
        blank = np.zeros((image.shape[0],image.shape[1]),dtype=np.int8)
        b = 0
        constant = int(image.shape[0]/self.verticalDivision)
        for i in np.array_split(threshimg,self.verticalDivision):
            a = 0 
            for j in range(int(image.shape[0]/self.verticalDivision)-1,image.shape[1]-1,int(image.shape[0]/self.verticalDivision)):
                average = np.average(i[:,j-(int(image.shape[0]/self.verticalDivision)-1):j])
                if average < self.colorElimination :
                    cv.rectangle(blank,(a*constant,b*constant+constant*2),(a*constant+constant,b*constant+3*constant),255,-1)
                a +=1
            b +=1
        cv.imshow("window",blank)
        cv.waitKey(10)
        return blank
if __name__ == "__main__":
    fire = fireDetecter()
    fire.multiThread()
    cv.waitKey(1000)
    for i in range(200):
        fire.main(fire.frame)    
    fire.console = 0