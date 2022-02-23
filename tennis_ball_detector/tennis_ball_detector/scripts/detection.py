import cv2
import imutils
import time
import math

def frame_to_center(x, y, L, H):

	x_c = round((x/(L/2) - 1), 3)

	y_c = round((-y/(H/2) + 1), 3)

	return x_c, y_c

def frame_back(x_c, y_c, L, H):
	
	x = int((x_c+1)*(L/2))

	y = int((-y_c+1)*(H/2))

	return x, y

def draw_boxes_from_center_coord(image, boxes, color = (0,255,0), width = 5):
	for box in boxes:
		x_c, y_c, w_n, h_n = box[:4]
		x, y = frame_back(x_c, y_c, image.shape[1], image.shape[0])
		w, h = int(w_n*(image.shape[1]/2)), int(h_n*(image.shape[0]/2))
		cv2.rectangle(image,(x,y),(x+w,y+h),color,width)

def draw_boxes(image, boxes, color = (0,255,0), width = 5):
	for box in boxes:
		x, y, w, h = box
		cv2.rectangle(image,(x,y),(x+w,y+h),color,width)

def detect(image, rgb_filter):
	
	"""
	return image filtered and list of objets coordinates and sizes (x,y,w,h)

	"""

	nb_min_points_contours = 3
	
	mask = cv2.GaussianBlur(image, (3,3), 0)
	
	mask = cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)

	mask = cv2.inRange(mask, rgb_filter[:3], rgb_filter[3:])

	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

	objects = []

	if len(cnts) > 0 :

		for cnt in cnts :
			
			if len(cnt) > nb_min_points_contours:
				
				x, y, w, h = cv2.boundingRect(cnt)

				x_center, y_center = frame_to_center(x, y, mask.shape[1], mask.shape[0])

				objects.append((x_center, y_center, round(w/(mask.shape[1]/2),4), round(h/(mask.shape[0]/2),4)))

	return mask, objects


def detect_balls(image, rgb_filter = (81, 102, 0, 148, 147, 60)): # (0, 90, 0, 255, 255, 50)
	return detect(image, rgb_filter)


def detect_zones(image, rgb_filter = (0, 0, 0, 255, 100, 50)):
	return detect(image, rgb_filter)


def detect_circles(image, rgb_filter, radius_max, radius_min):

	nb_min_points_contours = 3
	
	mask = cv2.GaussianBlur(image, (3,3), 0)
	
	mask = cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)

	mask = cv2.inRange(mask, rgb_filter[:3], rgb_filter[3:])

	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

	objects = []

	if len(cnts) > 0 :

		for cnt in cnts :
			
			if len(cnt) > nb_min_points_contours:
				
				((x, y), radius) = cv2.minEnclosingCircle(cnt)

				if radius_max >= radius >= radius_min:

					objects.append((x, y, radius))

	return mask, objects


def detect_robot(image, lenght_max=10000, lenght_min =0, radius_max=1000, radius_min=0):

	mask_red, circles_red = detect_circles(image, (130, 0, 0, 255, 2, 2), radius_max, radius_min)
	mask_green, circles_green = detect_circles(image, (0, 130, 0, 2, 255, 2), radius_max, radius_min)

	mask = cv2.bitwise_or(mask_green, mask_red)
	# show_img("mask", mask)
	# show_img("image", image)
	robot_pos = None

	if circles_green != [] and circles_red != []:

		for circle_red in circles_red:

				for circle_green in circles_green:

					lenght_between_circles = math.sqrt((circle_red[0]-circle_green[0])**2+(circle_red[1]-circle_green[1])**2)

					if lenght_max >= lenght_between_circles >= lenght_min:
						x, y, theta = compute_pos(circle_green[:2], circle_red[:2], image)
						# print("Lenght between circle = ", lenght_between_circles)
						

						x_center, y_center = frame_to_center(x, y, image.shape[1], image.shape[0])
						robot_pos = (x_center, y_center, theta)

	return mask, robot_pos


def draw_pos(image, pos, color, thickness, lenght):

	x, y = pos[0], pos[1]
	start_point = int(x), int(y)
	end_point = int(x + lenght*math.cos(pos[2])), int(y + lenght*math.sin(pos[2]))

	cv2.arrowedLine(image, start_point, end_point, color, thickness)


def compute_pos(xyA, xyB, image):
	xA, yA = xyA
	xB, yB = xyB
	x, y = (xA+xB)/2, (yA+yB)/2
	vec_A_B = xB - xA, yB - yA
	vec_dir = - vec_A_B[1], vec_A_B[0]
	theta = math.atan2(vec_dir[1], vec_dir[0])

	return x, y, theta


def show_img(name, img, device = "Ubuntu"):

	cv2.namedWindow(name, cv2.WINDOW_NORMAL)
	if device != "Windows":
		cv2.resizeWindow(name, 1000, 800)
	cv2.imshow(name, img)


if __name__ == "__main__":

	image_name = 'balls.png'
	img = cv2.imread(image_name)

	t = 0

	r1 = 100
	r2 = 120
	
	while t < 20:

		frame = img.copy()

		x1 = int(r1*math.cos(t) + 500)
		y1 = int(r1*math.sin(t) + 500)
		x2 = int(r2*math.cos(t) + 500)
		y2 = int(r2*math.sin(t) + 500)

		cv2.circle(frame, (x1, y1), 10, (0, 0, 255), -1)
		cv2.circle(frame, (x2, y2), 10, (0, 255, 0), -1)

		ball_img, balls = detect_balls(frame)

		zone_img, zones = detect_zones(frame)

		robot_img, robot = detect_robot(frame)

		show_img("Balls", ball_img)

		show_img("Zones", zone_img)

		show_img('Robot', robot_img)

		show_img("Origin", frame)

		print("Robot position :", robot)

		t += 0.1

		if cv2.waitKey(100) & 0xFF is ord('q'):
			cv2.destroyAllWindows()
			break
