import carla
import os
from queue import Queue
from queue import Empty


def sensor_callback(sensor_data, sensor_queue, sensor_name):
    save_dir = os.path.join('output/left_turn', sensor_name)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    if 'lidar' in sensor_name:
        sensor_data.save_to_disk(os.path.join(save_dir, '%06d.ply' % sensor_data.frame))
    if 'camera' in sensor_name:
        sensor_data.save_to_disk(os.path.join(save_dir, '%06d.png' % sensor_data.frame))
    sensor_queue.put((sensor_data.frame, sensor_name))


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
        bp_lib = world.get_blueprint_library()
        # Set weather for your world
        # weather = carla.WeatherParameters(cloudiness=10.0,
        #                                   precipitation=10.0,
        #                                   fog_density=10.0)
        # world.set_weather(weather)

        # set synchorinized mode
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.fixed_delta_seconds = 0.05
        settings.synchronous_mode = True
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        # set spectator
        init_trans = carla.Transform(carla.Location(z=45, x=-80, y=2), carla.Rotation(pitch=-90, yaw=-90, roll=0))
        spectator = world.get_spectator() 
        spectator.set_transform(init_trans)

        # create sensor queue
        sensor_queue = Queue()

        # create 3 vehicles
        vehicle_bp = bp_lib.find('vehicle.mercedes-benz.coupe') 
        vehicle_init_trans = carla.Transform(carla.Location(z=1, x=-78, y=10), carla.Rotation(pitch=0, yaw=-110, roll=0))
        vehicle1 = world.spawn_actor(vehicle_bp, vehicle_init_trans)
        vehicle_bp = bp_lib.find('vehicle.carlamotors.carlacola') 
        vehicle_init_trans = carla.Transform(carla.Location(z=1, x=-84, y=-3), carla.Rotation(pitch=0, yaw=60, roll=0))
        vehicle2 = world.spawn_actor(vehicle_bp, vehicle_init_trans)
        vehicle_bp = bp_lib.find('vehicle.audi.a2') 
        vehicle_init_trans = carla.Transform(carla.Location(z=1, x=-88, y=-10), carla.Rotation(pitch=0, yaw=90, roll=0))
        vehicle3 = world.spawn_actor(vehicle_bp, vehicle_init_trans)
        
        # collect all actors to destroy when we quit the script
        actor_list.extend([vehicle1, vehicle2, vehicle3])

        # create directory for outputs
        output_path = 'output/left_turn'
        if not os.path.exists(output_path):
            os.makedirs(output_path)

        # add a camera
        camera_bp = bp_lib.find('sensor.camera.rgb')
        # camera relative position related to the vehicle
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera1 = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle1)
        camera3 = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle3)
        camera_top = world.spawn_actor(camera_bp, init_trans)

        # we also add a lidar on it
        lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '50')
        lidar_bp.set_attribute('noise_stddev', '0.1')
        lidar_bp.set_attribute('upper_fov', '4')
        lidar_bp.set_attribute('lower_fov', '-20')
        lidar_bp.set_attribute('channels', '64')
        lidar_bp.set_attribute('rotation_frequency', '20')
        lidar_bp.set_attribute('points_per_second', '2304000')

        # set the relative location
        lidar_location = carla.Location(0, 0, 2)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        # spawn the lidar
        lidar1 = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle1)
        lidar3 = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle3)
        # set the callback function
        camera1.listen(lambda image: sensor_callback(image, sensor_queue, "camera1"))
        camera3.listen(lambda image: sensor_callback(image, sensor_queue, "camera3"))
        camera_top.listen(lambda image: sensor_callback(image, sensor_queue, "camera_top"))
        lidar1.listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, "lidar1"))
        lidar3.listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, "lidar3"))
        sensor_list.extend([camera1, camera3, camera_top, lidar1, lidar3])

        while True:
            world.tick()
            # As the queue is blocking, we will wait in the queue.get() methods
            # until all the information is processed and we continue with the next frame.
            try:
                for i in range(0, len(sensor_list)):
                    s_frame = sensor_queue.get(True, 1.0)
                    print("    Frame: %d   Sensor: %s" % (s_frame[0], s_frame[1]))

            except Empty:
                print("   Some of the sensor information is missed")

    finally:
        world.apply_settings(original_settings)
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
