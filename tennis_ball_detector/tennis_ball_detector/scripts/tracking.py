#usr/bin/python3
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

from .detection import detect_balls, detect_zones, draw_boxes_from_center_coord, show_img


def ball_in_balls(ball_to_test, balls, proportion_size_box_validation):
	test = False
	k = 0 
	for ball in balls :
		if ball_in_the_box(ball_to_test, ball, proportion_size_box_validation):
			test = True
			break
		k += 1
	return k, test


def ball_in_zones(ball, zones, psbv):
	ball_in_zone = False
	for zone in zones:
		ball_in_zone |= ball_in_the_box(ball, zone, psbv)
	return ball_in_zone


def ball_in_the_box(ball, box, psbv):
	valid_x = (box[0] - box[2]/2*psbv <= ball[0] <= box[0] + box[2]/2*psbv)
	valid_y = (box[1] - box[3]/2*psbv <= ball[1] <= box[1] + box[3]/2*psbv)

	return valid_x and valid_y


def track_balls(t, image, last_balls, valid_balls):

	ball_img, new_balls = detect_balls(image)

	#show_img("mask", ball_img)
	#print(new_balls)

	zone_img, zones = detect_zones(image)

	updated_valid_balls = []

	for new_b in new_balls:

		new_ball_in_zones = ball_in_zones(new_b, zones, 1)

		if new_ball_in_zones :
			continue

		_, new_ball_in_last_balls = ball_in_balls(new_b, last_balls, 1)
		
		if new_ball_in_last_balls:

			indice_valid_ball, new_ball_in_valid_balls = ball_in_balls(new_b, valid_balls, 1)

			if new_ball_in_valid_balls:
				updated_valid_balls.append((new_b[0], new_b[1], new_b[2], new_b[3], valid_balls[indice_valid_ball][4]))

			else:

				updated_valid_balls.append((new_b[0], new_b[1], new_b[2], new_b[3], t))

	return updated_valid_balls, new_balls[:]


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

	end = False

	for filename in range(1,102):

		frame = cv2.imread(folder_path+num_to_str(filename)+".png")

		# print("Open image :", filename, " at time ", i*dt)


		x_min, x_max, y_min, y_max = 287, 903, 572, 1660

		frame = frame[x_min:x_max,y_min:y_max]

		valid_detected_balls, last_detected_balls = track_balls(i*dt, frame, last_detected_balls, valid_detected_balls)

		# print(valid_detected_balls)


		i += 1

		draw_boxes_from_center_coord(frame, last_detected_balls, (0,0,255), 1)

		draw_boxes_from_center_coord(frame, valid_detected_balls, (0,255,0), 1)

		#show_img("frame", frame)

		while True:
			key = cv2.waitKey(1) & 0xFF 
			if key is ord('q'):
				end = True
				break
			elif key is ord('d'):
				break

		if end:
			cv2.destroyAllWindows()
			break

		
