#collect the data for the CARLA 0.7.1
from PIL import Image
import random
import time
import keyboard
import sys
import argparse
import logging
import os
import pandas as pd
#import keyboard
#in the new version CARLA,there is no client_axample.py any more
#the carla import packages has changed
from carla.client import make_carla_client
from carla.sensor import Camera
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line
#-----------------------------------------------------------------
# Constant that set how often the episodes are reseted
RESET_FREQUENCY = 100
from enum import Enum
class WeatherCondition(Enum):
	Default =0
	ClearNoon = 1
	CloudyNoon= 2
	WetNoon=3
	WetCloudyNoon = 4
	MidRainyNoon = 5
	HardRainNoon = 6
	SoftRainNoon = 7
	ClearSunset = 8
	CloudySunset = 9
	WetSunset = 10
	WetCloudySunset = 11
	MidRainSunset = 12
	HardRainSunset = 13
	SoftRainSunset = 14
#------------------------------------------------------------------
"""
Print function, prints all the measurements saving
the images into a folder. WARNING just prints the first BGRA image
Args:
    param1: The measurements dictionary to be printed
    param2: The iterations

Returns:
    None
Raises:
    None
"""
def print_measurements(measurements):
	number_of_agents = len(measurements.non_player_agents)
	player_measurements = measurements.player_measurements
	message = 'Vehicle at ({pos_x:.1f}, {pos_y:.1f}), '
	message += '{speed:.2f} km/h, '
	message += 'Collision: {{vehicles={col_cars:.0f}, pedestrians={col_ped:.0f}, other={col_other:.0f}}}, '
	message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road, '
	message += '({agents_num:d} non-player agents in the scene)'
	message = message.format(
        pos_x=player_measurements.transform.location.x / 100, # cm -> m
        pos_y=player_measurements.transform.location.y / 100,
        speed=player_measurements.forward_speed,
        col_cars=player_measurements.collision_vehicles,
        col_ped=player_measurements.collision_pedestrians,
        col_other=player_measurements.collision_other,
        other_lane=100 * player_measurements.intersection_otherlane,
        offroad=100 * player_measurements.intersection_offroad,
        agents_num=number_of_agents)
	print_over_same_line(message)
#----------------------------------------------------------------------
def run_carla_client(host, port, save_images_to_disk, image_filename_format):
	number_of_episodes=2
	frames_per_episod=300
	with make_carla_client(host, port) as client:
		print("carla colient connected")
		for episode in range(0, number_of_episodes):
			data = []
			settings = CarlaSettings()
			settings.set(
				SynchronousMode=True,
				SendNonPlayerAgentsInfo=True,
				NumberOfVehicles=50,
				NumberOfPedestrians=50,
				WeatherId=random.choice([1, 3, 7, 8, 14]))
			settings.randomize_seeds()

			# Now we want to add a couple of cameras to the player vehicle.
			# We will collect the images produced by these cameras every
			# frame.
			# The default camera captures RGB images of the scene.
			camera0 = Camera('CameraRGB')
			# Set image resolution in pixels.
			camera0.set_image_size(800, 600)
			# Set its position relative to the car in centimeters.
			camera0.set_position(200, 0, 140)
			camera0.set_rotation(-15, 0.0, 0.0)
			settings.add_sensor(camera0)

			# Let's add another camera producing ground-truth depth.
			camera1 = Camera('CameraDepth', PostProcessing='Depth')
			camera1.set_image_size(800, 600)
			camera1.set_position(200, 0, 140)
			camera1.set_rotation(-15, 0.0, 0.0)
			settings.add_sensor(camera1)
			# Let's add another camera producing ground-truth depth.
			camera2 = Camera('CameraSemSeg', PostProcessing='SemanticSegmentation')
			camera2.set_image_size(800, 600)
			camera2.set_position(200, 0, 140)
			camera2.set_rotation(-15, 0.0, 0.0)
			settings.add_sensor(camera2)
			#load the configuration settings
			scene = client.load_settings(settings)
			# Choose one player start at random.
			number_of_player_starts = len(scene.player_start_spots)
			player_start = random.randint(0, max(0, number_of_player_starts - 1))

			# Notify the server that we want to start the episode at the
			# player_start index. This function blocks until the server is ready
			# to start the episode.
			print('Starting new episode...')
			client.start_episode(player_start)
			# Iterate every frame in the episode.
			for frame in range(0, frames_per_episod):

				# Read the data produced by the server this frame.
				measurements, sensor_data = client.read_data()
				player_measurements = measurements.player_measurements
				#----------------------save the data------------------
				pos_x = player_measurements.transform.location.x / 100,  # cm -> m
				pos_y = player_measurements.transform.location.y / 100,
				speed = player_measurements.forward_speed,
				col_cars = player_measurements.collision_vehicles,
				col_ped = player_measurements.collision_pedestrians,
				col_other = player_measurements.collision_other,
				other_lane = 100 * player_measurements.intersection_otherlane,
				offroad = 100 * player_measurements.intersection_offroad,
				# Print some of the measurements.
				print_measurements(measurements)

				# We can access the encoded data of a given image as numpy
				# array using its "data" property. For instance, to get the
				# depth value (normalized) at pixel X, Y
				#
				#     depth_array = sensor_data['CameraDepth'].data
				#     value_at_pixel = depth_array[Y, X]

				# Now we have to send the instructions to control the vehicle.
				# If we are in synchronous mode the server will pause the
				# simulation until we send this control.
				control = measurements.player_measurements.autopilot_control
				control.steer += random.uniform(-0.2, 0.2)

				if measurements.player_measurements.forward_speed < 35:
				#control the speed is lower than 35,in the carla the speed limit is often 30
					control.brake = control.brake
				else:
					control.brake = 1.0
				client.send_control(control)
				#save the data
				# Save the images to disk if requested.
				if save_images_to_disk:
					# here set that running speed is 20 FPSm,save the images every 20 frame
					if (frame % 20) == 0:
						print('Collecting data', frame)
						for name, image in sensor_data.items():
							image.save_to_disk(image_filename_format.format(episode, name, frame))
						if control.brake is None:
							control.brake = 0.0
						if control.steer is None:
							control.steer = 0.0
						if control.throttle is None:
							control.throttle = 0.0
						if control.hand_brake is None:
							control.hand_brake = 0
						else:
							control.hand_brake = 1
						if control.reverse is None:
							control.reverse = 0
						else:
							control.reverse = 1
						data.append(
							[frame,pos_x,pos_y,speed,col_cars,col_other,col_ped,other_lane,offroad,control.brake, control.steer, control.throttle, control.hand_brake, control.reverse])
			df = pd.DataFrame.from_records(data,
										   columns=['frame','X','Y','forward_spped','col_cars','col_ped','col_other',
													'other_lane','off_road','brake','steer', 'throttle', 'hand_brake', 'reverse'])
			path = 'datas/episode_{:0>3d}/data.csv'
			path = path.format(episode)
			folder = os.path.dirname(path)
			if not os.path.isdir(folder):
				os.makedirs(folder)
			df.to_csv(path, sep=',', encoding='utf-8')

#---------------------------------------------------------------------------------
def main():
	argparser = argparse.ArgumentParser(description=__doc__)
	argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
	argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
	argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
	# argparser.add_argument(
     #    '-i', '--images-to-disk',
     #    action='store_true',
     #    help='save images to disk')
	# argparser.add_argument(
     #    '-c', '--carla-settings',
     #    metavar='PATH',
     #    default=None,
     #    help='Path to a "CarlaSettings.ini" file')

	args = argparser.parse_args()
	log_level = logging.DEBUG if args.debug else logging.INFO
	logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
	logging.info('listening to server %s:%s', args.host, args.port)


	try:
		run_carla_client(
            host=args.host,
            port=args.port,
            save_images_to_disk=True,
            image_filename_format='_images/episode_{:0>3d}/{:s}/image_{:0>5d}.png',)
		print('Done.')

	except TCPConnectionError as error:
		logging.error(error)
		time.sleep(1)


if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
		print('\nCancelled by user. Bye!')
