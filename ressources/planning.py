import math
import numpy as np

import cv2
from detection import detect_balls, detect_zones, detect_robot, draw_boxes_from_center_coord, show_img, draw_pos, frame_back
from tracking import track_balls, num_to_str



def ball_score(t_simulation, t_apparition):
	return 10000/t_simulation + 20000/t_apparition


def net_is_crossed(x1, x2):
	return x1*x2 <= 0


def distance(x1, y1, x2, y2):
	return math.sqrt((x1-x2)**2+(y1-y2)**2)


def nearest_zone(x,y,zones):
	distances = []
	for zone in zones:
		distances.append(distance(x,y,zone[0],zone[1]))
	return min(distances)


def best_score(x, y, balls, t_sim, v_robot, zones = [[-0.9, 0.9], [0.9, -0.9]], passages = [[0, 0.9], [0, -0.9]]):

	"""
	Compute every ball score
	Return indice of the ball the with the best score

	"""

	scores = []

	for ball in balls:

		xb, yb, wb, hb, t_app = ball

		if net_is_crossed(x,xb):
			detour0 = distance(x,y,passages[0][0],passages[0][1])+distance(xb,yb,passages[0][0],passages[0][1])
			detour1 = distance(x,y,passages[1][0],passages[1][1])+distance(xb,yb,passages[1][0],passages[1][1])
			d_ball = min(detour0, detour1)
		else:
			d_ball = distance(x,y,xb,yb)

		d_zone = nearest_zone(xb, yb, zones)

		t_mission = (d_ball + d_zone)/v_robot

		score = ball_score(t_sim+t_mission, t_app+t_mission)

		scores.append(score)

	return np.argmin(scores)


if __name__ == "__main__":

	folder_path = 'Frames_balls/'

	last_detected_balls = []

	valid_detected_balls = []

	i = 0

	dt = 0.5

	end = False

	r1 = 100
	r2 = 120

	vitesse_robot = 0.5

	for filename in range(1,102):

		x1 = int(r1*math.cos(i*dt) + 250)
		y1 = int(r1*math.sin(i*dt) + 250)
		x2 = int(r2*math.cos(i*dt) + 250)
		y2 = int(r2*math.sin(i*dt) + 250)

		img = cv2.imread(folder_path+num_to_str(filename)+".png")

		frame = img.copy()

		print("Open image :", filename, " at time ", i*dt)

		x_min, x_max, y_min, y_max = 287, 903, 572, 1660

		frame = frame[x_min:x_max,y_min:y_max]

		valid_detected_balls, last_detected_balls = track_balls(i*dt, frame, last_detected_balls, valid_detected_balls)

		cv2.circle(frame, (x1, y1), 10, (0, 0, 255), -1)
		cv2.circle(frame, (x2, y2), 10, (0, 255, 0), -1)

		robot_img, robot = detect_robot(frame)
		x_rob, y_rob = frame_back(robot[0], robot[1], frame.shape[1], frame.shape[0])
		draw_pos(frame, (x_rob,y_rob,robot[2]), (255, 0, 0), 10, 50)

		draw_boxes_from_center_coord(frame, last_detected_balls, (0,0,255), 1)

		draw_boxes_from_center_coord(frame, valid_detected_balls, (0,255,0), 1)

		if valid_detected_balls != []:

			i_best_ball = best_score(robot[0], robot[1], valid_detected_balls, i*dt, vitesse_robot)
			draw_boxes_from_center_coord(frame, [valid_detected_balls[i_best_ball]], (0,0,0), 1)

		show_img("Camera", frame)

		i += 1

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

		


