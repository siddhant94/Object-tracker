# USAGE
# python track.py --video video/sample.mov

# import the necessary packages
import numpy as np
import argparse
import cv2
from kalman2d import Kalman2D

# initialize the current frame of the video, along with the list of
# ROI points along with whether or not this is input mode
frame = None
roiPts = []
inputMode = False
Width = 640
Height = 480
wl = Width*4.5/10
wr = Width*5.5/10
ht = Height*4.5/10
hb = Height*5.5/10
targetBox = np.array([[wl,ht], [wr,ht], [wr,hb], [wl,hb]])

class CenterInfo(object):

		def __init__(self): #Init center information
				self.x, self.y = -1, -1
		def __str__(self): #for print center information
				return '%4d %4d' % (self.x, self.y)

def selectROI(event, x, y, flags, param):
	# grab the reference to the current frame, list of ROI
	# points and whether or not it is ROI selection mode
	global frame, roiPts, inputMode

	# if we are in ROI selection mode, the mouse was clicked,
	# and we do not already have four points, then update the
	# list of ROI points with the (x, y) location of the click
	# and draw the circle
	if inputMode and event == cv2.EVENT_LBUTTONDOWN and len(roiPts) < 4:
		roiPts.append((x, y))
		cv2.circle(frame, (x, y), 4, (0, 255, 0), 2)
		cv2.imshow("frame", frame)

def main():
	# construct the argument parse and parse the arguments
	ap = argparse.ArgumentParser()
	ap.add_argument("-v", "--video",
		help = "path to the (optional) video file")
	args = vars(ap.parse_args())

	# grab the reference to the current frame, list of ROI
	# points and whether or not it is ROI selection mode
	global frame, roiPts, inputMode

	cnt = 0	#count for new roiBox from kalmanfilter 
	centerX = 0
	centerY = 0
	toggle = True #toggle for imshow
	flag = False #flag for moving

	kalman2d = Kalman2D() #Create new Kalman filter and initailize

	# if the video path was not supplied, grab the reference to the
	# camera
	if not args.get("video", False):
		camera = cv2.VideoCapture(0)

	# otherwise, load the video
	else:
		camera = cv2.VideoCapture(args["video"])

	# setup the mouse callback
	cv2.namedWindow("frame")
	cv2.setMouseCallback("frame", selectROI)

	# initialize the termination criteria for cam shift, indicating
	# a maximum of ten iterations or movement by a least one pixel
	# along with the bounding box of the ROI
	termination = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
	roiBox = None

	# keep looping over the frames
	while True:
		# grab the current frame
		(grabbed, frame) = camera.read()

		# check to see if we have reached the end of the
		# video
		if not grabbed:
			break

		# if the see if the ROI has been computed
		if roiBox is not None:
			# convert the current frame to the HSV color space
			# and perform mean shift
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			backProj = cv2.calcBackProject([hsv], [0], roiHist, [0, 180], 1)

			# apply cam shift to the back projection, convert the
			# points to a bounding box, and then draw them
			(r, roiBox) = cv2.CamShift(backProj, roiBox, termination)
			pts = np.int0(cv2.cv.BoxPoints(r))
			cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

			#Calculate center x,y
			centerX = (pts[0][0] + pts[2][0])/2
			centerY = (pts[0][1] + pts[2][1])/2
						
			#Update x,y to kalman filter and get estimated x,y
			CenterInfo.x = centerX
			CenterInfo.y = centerY

	#Send center x,y to Arduino
			if CenterInfo.x / 10 == 0:
				tempCenterX = '00' + str(CenterInfo.x)
			elif CenterInfo.x / 100 == 0:
				tempCenterX = '0' + str(CenterInfo.x)
			else:
				tempCenterX = str(CenterInfo.x)
						
			if CenterInfo.y / 10 == 0:
				tempCenterY = '00' + str(CenterInfo.y)
			elif CenterInfo.y / 100 == 0:
				tempCenterY = '0' + str(CenterInfo.y)
			else:
				tempCenterY = str(CenterInfo.y)

			centerData = str(int(flag)) + tempCenterX + tempCenterY
			#print centerData

						
						#Update Kalman
			kalman2d.update(CenterInfo.x, CenterInfo.y)
			estimated = [int (c) for c in kalman2d.getEstimate()]

			estimatedX = estimated[0]
			estimatedY = estimated[1]
						
						#Calculate delta x,y
			deltaX = estimatedX - centerX
			deltaY = estimatedY - centerY

				#Apply new roiBox from kalmanfilter
			if cnt > 1:
				roiBox = (roiBox[0]+deltaX, roiBox[1]+deltaY, br[0], br[1])
				cnt = 0
						
						#Draw estimated center x,y from kalman filter for test
						#cv2.circle(frame,(estimatedX,estimatedY), 4, (0, 255, 255),2)
						
						#Change a color when target is in target box
			if wl < centerX and wr > centerX and centerY < hb and centerY > ht :
				cv2.circle(frame,(centerX,centerY), 4, (255,0,0),2)
				flag = False
			else :
				cv2.circle(frame,(centerX,centerY), 4, (0,255,0),2)
				flag = True

			cnt = cnt+1	#count for apply new box from kalman filter				
						
						#Draw kalman top left point for test
						#cv2.circle(frame,(roiBox[0],roiBox[1]), 4, (0,0,255),2)

					#Draw target box
			cv2.circle(frame,(Width/2,Height/2) , 4, (255,255,255),2)
			cv2.polylines(frame, np.int32([targetBox]), 1, (255,255,255))

		# show the frame and record if the user presses a key
		cv2.imshow("frame", frame)
		key = cv2.waitKey(1) & 0xFF


		# handle if the 'i' key is pressed, then go into ROI
		# selection mode
		if key == ord("i") and len(roiPts) < 4:
			# indicate that we are in input mode and clone the
			# frame
			inputMode = True
			orig = frame.copy()

			# keep looping until 4 reference ROI points have
			# been selected; press any key to exit ROI selction
			# mode once 4 points have been selected
			while len(roiPts) < 4:
				cv2.imshow("frame", frame)
				cv2.waitKey(0)

			# determine the top-left and bottom-right points
			roiPts = np.array(roiPts)
			s = roiPts.sum(axis = 1)
			tl = roiPts[np.argmin(s)]
			br = roiPts[np.argmax(s)]

			# grab the ROI for the bounding box and convert it
			# to the HSV color space
			roi = orig[tl[1]:br[1], tl[0]:br[0]]
			roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
			#roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

			# compute a HSV histogram for the ROI and store the
			# bounding box
			roiHist = cv2.calcHist([roi], [0], None, [16], [0, 180])
			roiHist = cv2.normalize(roiHist, roiHist, 0, 255, cv2.NORM_MINMAX)
			roiBox = (tl[0], tl[1], br[0], br[1])

		# if the 'q' key is pressed, stop the loop
		elif key == ord("q"):
			break

	# cleanup the camera and close any open windows
	camera.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()
