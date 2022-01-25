import cv2
import imutils

def frame_to_center(x, y, L, H):

	x_c = round((x/(L/2) - 1), 3)

	y_c = round((-y/(H/2) + 1), 3)

	return x_c, y_c


def detect(image, rgb_filter):
	
	"""
	return image filtered and list of objets coordinates and sizes (x,y,w,h)

	"""

	nb_min_points_contours = 4
	
	mask = cv2.GaussianBlur(image, (3,3), 0)
	
	mask = cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)

	mask = cv2.inRange(mask, rgb_filter[:3], rgb_filter[3:])

	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

	objects = []
	print(mask.shape)
	
	if len(cnts) > 0 :

		for cnt in cnts :
			
			if len(cnt) > nb_min_points_contours:
				
				x, y, w, h = cv2.boundingRect(cnt)
				
				cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),5)

				x_center, y_center = frame_to_center(x, y, mask.shape[1], mask.shape[0])

				objects.append((x_center, y_center, w, h))

				print(x, y, " to ", x_center, y_center)

	return mask, objects


def detect_balls(image, rgb_filter = (0, 90, 0, 255, 255, 50)): 
	return detect(image, rgb_filter)

def detect_zones(image, rgb_filter = (0, 0, 0, 255, 100, 50)):
	return detect(image, rgb_filter)

def show_img(name, img, device = "Ubuntu"):

	cv2.namedWindow(name, cv2.WINDOW_NORMAL)
	if device != "Windows":
		cv2.resizeWindow(name, 1000, 800)
	cv2.imshow(name, img)


if __name__ == "__main__":

	image_name = 'balls.png'
	img = cv2.imread(image_name)

	ball_img, balls = detect_balls(img)

	zone_img, zones = detect_zones(img)
	
	while True:

		show_img("Balls", ball_img)

		show_img("Zones", zone_img)

		show_img("Origin", img)

		if cv2.waitKey(1) & 0xFF is ord('q'):
			cv2.destroyAllWindows()
			break