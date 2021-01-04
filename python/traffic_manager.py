import argparse
import carla
import cv2
import logging
import time
import numpy as np
from numpy import random
from queue import Queue
from queue import Empty


def parser():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=20,
        type=int,
        help='number of vehicles (default: 30)')
    argparser.add_argument(
        '-d', '--number-of-dangerous-vehicles',
        metavar='N',
        default=1,
        type=int,
        help='number of dangerous vehicles (default: 3)')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        default=True,
        help='Synchronous mode execution')

    return argparser.parse_args()


def sensor_callback(sensor_data, sensor_queue):
    array = np.frombuffer(sensor_data.raw_data, dtype=np.dtype("uint8"))
    # image is rgba format
    array = np.reshape(array, (sensor_data.height, sensor_data.width, 4))
    array = array[:, :, :3]
    sensor_queue.put((sensor_data.frame, array))


def main():
    args = parser()
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_id_list = []

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False

    try:    
        world = client.get_world()
        origin_settings = world.get_settings()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        # every vehicle keeps a distance of 3.0 meter
        traffic_manager.set_global_distance_to_leading_vehicle(3.0)
        # Set physical mode only for cars around ego vehicle to save computation
        traffic_manager.set_hybrid_physics_mode(True)
        # default speed is 30
        traffic_manager.global_percentage_speed_difference(80)

        # Suggest using syncmode
        if args.sync:
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                # 20fps
                settings.fixed_delta_seconds = 0.05
                world.apply_settings(settings)

        blueprints_vehicle = world.get_blueprint_library().filter("vehicle.*")
        # sort the vehicle list by id
        blueprints_vehicle = sorted(blueprints_vehicle, key=lambda bp: bp.id)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles >= number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points - 1

        # Use command to apply actions on batch of data
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        # this is equal to int 0
        FutureActor = carla.command.FutureActor

        batch = []

        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break

            blueprint = random.choice(blueprints_vehicle)

            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)

            # set autopilot
            blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot all together
            batch.append(SpawnActor(blueprint, transform)
                         .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

        # excute the command
        for (i, response) in enumerate(client.apply_batch_sync(batch, synchronous_master)):
            if response.error:
                logging.error(response.error)
            else:
                print("Fucture Actor", response.actor_id)
                vehicles_id_list.append(response.actor_id)

        vehicles_list = world.get_actors().filter('vehicle.*')
        # wait for a tick to ensure client receives the last transform of the vehicles we have just created
        if not args.sync or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

        # set several of the cars as dangerous car
        for i in range(args.number_of_dangerous_vehicles):
            danger_car = vehicles_list[i]
            # crazy car ignore traffic light, do not keep safe distance, and very fast
            traffic_manager.ignore_lights_percentage(danger_car, 100)
            traffic_manager.distance_to_leading_vehicle(danger_car, 0)
            traffic_manager.vehicle_percentage_speed_difference(danger_car, -50)

        print('spawned %d vehicles , press Ctrl+C to exit.' % (len(vehicles_list)))

        # create ego vehicle
        ego_vehicle_bp = world.get_blueprint_library().find('vehicle.mercedes-benz.coupe')
        # green color
        ego_vehicle_bp.set_attribute('color', '0, 255, 0')
        # set this one as ego
        ego_vehicle_bp.set_attribute('role_name', 'hero')
        # get a valid transform that has not been assigned yet
        transform = spawn_points[len(vehicles_id_list)]

        ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)
        ego_vehicle.set_autopilot(True, args.tm_port)
        vehicles_id_list.append(ego_vehicle.id)

        # create sensor queue
        sensor_queue = Queue(maxsize=10)

        #  create camera
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        # camera relative position related to the vehicle
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
        # set the callback function
        camera.listen(lambda image: sensor_callback(image, sensor_queue))

        while True:
            if args.sync and synchronous_master:
                world.tick()
                try:
                    s_frame = sensor_queue.get(True, 1.0)
                    print("Camera Frame: %d" % (s_frame[0]))

                    # show image in a poping window
                    cv2.imshow('camera', s_frame[1])
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                except Empty:
                    print("Some of the sensor information is missed")
            else:
                world.wait_for_tick()

    finally:
        world.apply_settings(origin_settings)
        print('\ndestroying %d vehicles' % len(vehicles_id_list))

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_id_list])
        camera.destroy()
        cv2.destroyAllWindows()

        time.sleep(0.5)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
