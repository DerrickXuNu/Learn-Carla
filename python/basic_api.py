"""
In this script, we are going to learn how to spawn a vehicle on the road and make it autopilot.
At the same time, we will collect camera and lidar data from it.
"""

import carla
import os
import random


def main():
    actor_list = []
    sensor_list = []

    try:
        # First of all, we need to create the client that will send the requests, assume port is 2000
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        world = client.get_world()
        # world = client.load_world('Town02') # you can also retrive another world by specifically defining
        blueprint_library = world.get_blueprint_library()
        # Set weather for your world
        weather = carla.WeatherParameters(cloudiness=10.0,
                                          precipitation=10.0,
                                          fog_density=10.0)
        world.set_weather(weather)

        # create the ego vehicle
        ego_vehicle_bp = blueprint_library.find('vehicle.mercedes-benz.coupe')
        # black color
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # get a random valid occupation in the world
        transform = random.choice(world.get_map().get_spawn_points())
        # spawn the vehilce
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)
        # set the vehicle autopilot mode
        ego_vehicle.set_autopilot(True)

        # collect all actors to destroy when we quit the script
        actor_list.append(ego_vehicle)

        # add a camera
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        # camera relative position related to the vehicle
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)

        output_path = '../outputs/output_basic_api'
        if not os.path.exists(output_path):
            os.makedirs(output_path)

        # set the callback function
        camera.listen(lambda image: image.save_to_disk(os.path.join(output_path, '%06d.png' % image.frame)))
        sensor_list.append(camera)

        # we also add a lidar on it
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(32))
        lidar_bp.set_attribute('points_per_second', str(90000))
        lidar_bp.set_attribute('rotation_frequency', str(40))
        lidar_bp.set_attribute('range', str(20))

        # set the relative location
        lidar_location = carla.Location(0, 0, 2)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)

        # spawn the lidar
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        lidar.listen(
            lambda point_cloud: point_cloud.save_to_disk(os.path.join(output_path, '%06d.ply' % point_cloud.frame)))
        sensor_list.append(lidar)

        while True:
            # set the sectator to follow the ego vehicle
            spectator = world.get_spectator()
            transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
                                                    carla.Rotation(pitch=-90)))

    finally:
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        for sensor in sensor_list:
            sensor.destroy()
        print('done.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')

