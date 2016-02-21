#code for Convolution Testing in python
#code for the distance transform
import numpy as np
import math 
import matplotlib.pyplot as plt
#from cv2.cv import *
from PIL import Image
from tempfile import TemporaryFile
from numpy import linalg as LA
import numpy as np
import cv2
from cv2.cv import *
im = cv2.imread('4_1_map.png');#Read the Input Image 
imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)# Converting it into gray scale 
ret,thresh = cv2.threshold(imgray,127,255,0)#thresholding of image 
contours, im2 = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)# contour detection 
dist = cv2.pointPolygonTest(contours[0],(50,50),True)
print dist
print im.shape
print thresh.shape
print im2.shape
DistanceTransform = np.zeros((1175,1100));
for i in range(0,1175):
	for j in range(0,1100):
		print i,j
		if (thresh[i,j] == 0 ):
			min =abs(cv2.pointPolygonTest(contours[0],(j,i),True)) ;
			#minimum over every contour in the plane using Point Polygon Test i
			for k in range(1,61):
				dist = abs(cv2.pointPolygonTest(contours[k],(j,i),True)) ;
				if (dist<min):
					min = dist;
			DistanceTransform[i,j]=min;


		
#print thresh
cv2.imwrite('4_1_map.jpg',DistanceTransform);

#print np.shape(contours)
#print np.shape(contours[0])
#print contours[0][0][0]
#print im2[0]
#im2 =  np.asarray(im2)
#img = cv2.fromarray(im2)
##img.save('/home/pulkit/Desktop/im2.jpg')

#print imgray[0,0]
#thresh = im;
#contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE);
#print Image1im2
#print Image1[10,:]
#[ R,C] = np.shape(Image1);
#print Image1[1167,538]
#print Image1[1174,0];
#n = 0;
#a = [];
#b = [];
#for i in range(0,R):
#	for j in range(0,C):
#		if (Image1[i,j] != 0):
#			a.append(i)
#			b.append(j)
#			n=n+1;#break
#						
#print np.argmax(Image1)
#print a,b
#print a
#print b 
#print n 
#print Image1[a[0],b[0]];
#print Image1[a[1],b[1]];
#print Image1[a[2],b[2]];
#print Image1[a[n-1000],b[n-1000]];