import pclpy
from pclpy import pcl
import numpy as np
import matplotlib.pyplot as plt
import cv2

np.set_printoptions(suppress=True)
obj = pclpy.pcl.PointCloud.PointXYZRGBA()
pcl.io.loadPCDFile('rotfish.pcd',obj)

_xyz = obj.xyz
_rgb = obj.rgb

big=2

maxX=max(_xyz[:,0])*big
minX=min(_xyz[:,0])*big
maxY=max(_xyz[:,1])*big
minY=min(_xyz[:,1])*big
maxZ=max(_xyz[:,2])*big
minZ=min(_xyz[:,2])*big

width=abs(int(maxX-minX))+100
height=abs(int(maxY-minY))+100
z_height=abs(int(maxZ-minZ))+100

print(width,height)
print(width,z_height)

midX=width/2
midY=height/2
midZ=z_height/2
#z轴投影
img=np.zeros([height,width,3],np.uint8)
binImg=np.zeros([height,width,3],np.uint8)
for i in range(len(_xyz)):
    x=int(_xyz[i][0]*big+midX)
    y=int(height-(_xyz[i][1]*big+midY))
    if(x<width-1 and y<height-1):
        img[y,x,:]=_rgb[i,:]
        binImg[y,x,:]=255
#y轴投影
z_binImg=np.zeros([z_height,width,3],np.uint8)
for i in range(len(_xyz)):
    x=int(_xyz[i][0]*big+midX)
    y=int(z_height-(_xyz[i][2]*big+midZ))
    if(x<width-1 and y<z_height-1):
        z_binImg[y,x,:]=255



kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
close=cv2.morphologyEx(img,cv2.MORPH_CLOSE,kernel)
binclose=cv2.morphologyEx(binImg,cv2.MORPH_CLOSE,kernel)
z_binclose=cv2.morphologyEx(z_binImg,cv2.MORPH_CLOSE,kernel)

#中值滤波
img_median = cv2.medianBlur(binclose, 9)
z_img_median = cv2.medianBlur(z_binclose, 9)

cannyImg=cv2.Canny(img_median,200,300)
z_cannyImg=cv2.Canny(z_img_median,200,300)


gray = cv2.cvtColor(z_binImg, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)
#thresh = np.array(thresh,np.uint8)
contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,  cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(z_binImg, contours, -1, (0, 0, 255), 1)
cv2.namedWindow("weibing",0)
cv2.imshow("weibing",z_binImg)
cv2.waitKey(0)
cv2.destroyAllWindows()

z_differ=[]
for i in range(int(width/2)):
    bordary=[]
    for j in range(z_height):
        if(z_cannyImg[j,i]==255):
            bordary.append(j)
    bordary.sort()
    if(len(bordary)!=0):
        z_differ.append(abs(bordary[len(bordary)-1]-bordary[0]))
    else:
        z_differ.append(0)

d_z_differ=[]
for i in range(len(z_differ)-1):
    d_z_differ.append(z_differ[i+1]-z_differ[i])
print(d_z_differ)
#z_cannyImg[:,100]=255

plt.plot(z_differ)
plt.show()

cv2.namedWindow("z",0)
cv2.imshow("z",z_cannyImg)
cv2.namedWindow("weibing",0)
cv2.imshow("weibing",cannyImg)
cv2.waitKey(0)
cv2.destroyAllWindows()

#找尾柄
minweibing=1000
weibingIndex=0
for i in range(int(width/2)):
    bordary=[]
    for j in range(height):
        if(cannyImg[j,i]==255):
            bordary.append(j)
    if(len(bordary)==2):
        weibingtall=abs(bordary[0]-bordary[1])
        if(weibingtall<minweibing):
            minweibing=weibingtall
            weibingIndex=i
#cannyImg[:,weibingIndex]=255
cv2.namedWindow("weibing",0)
cv2.imshow("weibing",cannyImg)
cv2.waitKey(0)
cv2.destroyAllWindows()

#find convexhull
gray = cv2.cvtColor(img_median, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)
contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,  cv2.CHAIN_APPROX_SIMPLE)
cnt = contours[0]
hull = cv2.convexHull(cnt,returnPoints=False)
#print(hull)
length = len(hull)
"""
for i in range(len(contours)):
    cn=contours[i]
    epsilon = img_median.shape[0]/64
    approx = cv2.approxPolyDP(cn, epsilon, True)
    cv2.polylines(img_median, [approx], True, (0, 0, 255), 1)
"""
#for i in range(len(hull)):
    #cv2.line(img_median, tuple(hull[i][0]), tuple(hull[(i+1)%length][0]), (0,255,0), 2)
#cv2.drawContours(img,contours,-1,(0,0,255),1)
#找凹点
defects=cv2.convexityDefects(cnt,hull)
print(defects)
for i in range(defects.shape[0]):
    s,e,f,d = defects[i,0]
    start = tuple(cnt[s][0])
    end = tuple(cnt[e][0])
    far = tuple(cnt[f][0])
    cv2.line(img_median,start,end,[0,255,0],1)
    if(d>400):
        cv2.circle(img_median,far,5,[0,0,255],-1)


cv2.namedWindow("img",0)
cv2.imshow("img",img_median)
cv2.namedWindow("close",0)
cv2.imshow("close",binclose)
cv2.waitKey(0)
cv2.destroyAllWindows()
