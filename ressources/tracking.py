
"""
Numeroter les balles

Pour chaque nouvelles balles :
	- Si nouvelle balle déjà dans anciennces balles :
		- si balle déja dans valid
			- met à jour sa position
		- sinon :
			- ajouter dans valid avec le temps d'apparition
		break





test if in orange square

"""

import cv2

from detection import detect_balls, draw_boxes_from_center_coord, show_img


def callback(value):
	pass

def setup_trackbars(range_filter,image_shape):
	cv2.namedWindow("Trackbars", 0)
	for j in range_filter:
		for i in ["MIN", "MAX"]:
			v = 0 if i == "MIN" else image_shape[0 if j == "X" else 1]
			cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, image_shape[0 if j == "X" else 1], callback)

def get_trackbar_values(range_filter):
	values = []

	for i in ["MIN", "MAX"]:
		for j in range_filter:
			v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
			values.append(v)

	return values

def get_image_coord(image):
	
	setup_trackbars("XY",image.shape)
	print("Press K when finished")
	while True:
		x_min, y_min, x_max, y_max = get_trackbar_values("XY")
		
		if x_min < x_max and y_min < y_max :
			cv2.imshow("Find keyboard", image[x_min:x_max,y_min:y_max])
		else :
			print("Invalide value please respect x_max > x_min and y_max > y_min")
		
		if cv2.waitKey(1) & 0xFF is ord('k'):
			cv2.destroyAllWindows()
			break
	return x_min, x_max, y_min, y_max

def ball_in_balls(ball_to_test, balls, proportion_size_box_validation):
	test = False
	k = 0 
	for ball in balls :
		if is_the_same_ball(ball_to_test, ball, proportion_size_box_validation):
			test = True
			break
		k += 1
	return k, test


def is_the_same_ball(new_ball, last_ball, psbv):
	valid_x = (last_ball[0] - last_ball[2]/2*psbv <= new_ball[0] <= last_ball[0] + last_ball[2]/2*psbv)
	valid_y = (last_ball[1] - last_ball[3]/2*psbv <= new_ball[1] <= last_ball[1] + last_ball[3]/2*psbv)

	return valid_x and valid_y

def track_balls(t, image, last_balls, valid_balls):

	ball_img, new_balls = detect_balls(image)
	print(new_balls)

	for new_b in new_balls:
		_, new_ball_in_last_balls = ball_in_balls(new_b, last_balls, 0.5)
		
		if new_ball_in_last_balls:

			indice_valid_ball, new_ball_in_valid_balls = ball_in_balls(new_b, valid_balls, 1.5)

			if new_ball_in_valid_balls:
				valid_balls[indice_valid_ball] = (new_b[0], new_b[1], new_b[2], new_b[3], valid_balls[indice_valid_ball][4])

			else:

				valid_balls.append((new_b[0], new_b[1], new_b[2], new_b[3], t))

	return valid_balls, new_balls[:]


def num_to_str(num):
	if num//100 > 0:
		return str(num)
	elif num//10 > 0:
		return "0" + str(num)
	else:
		return "00" + str(num)

if __name__ == "__main__":

	folder_path = 'Frames_balls/'

	last_detected_balls = []

	valid_detected_balls = []

	i = 0

	dt = 0.5

	for filename in range(1,102):

		frame = cv2.imread(folder_path+num_to_str(filename)+".png")

		print("Open image :", filename, " at time ", i*dt)

		x_min, x_max, y_min, y_max = 287, 903, 572, 1660

		frame = frame[x_min:x_max,y_min:y_max]

		valid_detected_balls, last_detected_balls = track_balls(i*dt, frame, last_detected_balls, valid_detected_balls)
		print(valid_detected_balls)

		i += 1


		draw_boxes_from_center_coord(frame, last_detected_balls, (0,0,255), 1)

		draw_boxes_from_center_coord(frame, valid_detected_balls, (0,255,0), 1)

		show_img("frame", frame)

		while True:
			if cv2.waitKey(1) & 0xFF is ord('q'):
				cv2.destroyAllWindows()
				break

		
