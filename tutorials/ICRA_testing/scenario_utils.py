"""
scenario_utils.py — Shared Setup Helpers for the Scenario Suite
===================================================================
Pulled out of scenario_1_integrated.py so scenarios 2–4 don't each
re-implement debug weather, the overhead camera, and CARLA recorder
start/stop (same start_recorder()/stop_recorder() calls used in CARLA's
own start_recording.py / start_replaying.py examples).

Author: Ahmad Ahmad | For: Nidhi's Autonomous Driving Course
"""

import os
import carla


def debug_weather() -> carla.WeatherParameters:
    """Flat, glare-free lighting tuned for debugging (sun straight up,
    dry road, no atmospheric bloom). Same rationale as scenario 1."""
    return carla.WeatherParameters(
        cloudiness=60.0,
        precipitation=0.0,
        precipitation_deposits=0.0,
        wind_intensity=0.0,
        sun_azimuth_angle=0.0,
        sun_altitude_angle=90.0,
        fog_density=0.0,
        fog_distance=0.0,
        fog_falloff=0.0,
        wetness=0.0,
        scattering_intensity=0.0,
        mie_scattering_scale=0.0,
        rayleigh_scattering_scale=0.0331,
    )


def attach_overhead_camera(world, ego, output_dir, width=1280, height=720):
    """Spawn a chase camera behind/above the ego and save frames to disk,
    mirroring the camera setup in scenario_1_integrated.py."""
    os.makedirs(output_dir, exist_ok=True)
    bp_lib = world.get_blueprint_library()
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', str(width))
    cam_bp.set_attribute('image_size_y', str(height))
    cam = world.spawn_actor(
        cam_bp,
        carla.Transform(carla.Location(x=-10.0, y=0.0, z=6.0),
                         carla.Rotation(pitch=-25.0, yaw=0.0)),
        attach_to=ego,
    )
    cam.listen(lambda img: img.save_to_disk(f'{output_dir}/frame_{img.frame:06d}.png'))
    return cam


def top_down_follow(world, ego, height=30.0, pitch=-70.0):
    """Point the spectator at a top-down chase view of the ego. Call once
    per tick from the main loop, same pattern as scenario 1."""
    spectator = world.get_spectator()
    tf = ego.get_transform()
    spectator.set_transform(carla.Transform(
        tf.location + carla.Location(z=height),
        carla.Rotation(pitch=pitch, yaw=tf.rotation.yaw),
    ))


def start_recording(client, filename):
    """Start the CARLA recorder — same call as start_recording.py.
    Pass --record on the CLI to enable; filenames land next to
    output/<scenario>/ so a replay always has its matching frames."""
    print("Recording on file: %s" % client.start_recorder(filename))


def stop_recording(client):
    """Stop the CARLA recorder — same call as start_recording.py's finally
    block. Replay any .log later with CARLA's start_replaying.py:
        python start_replaying.py -f <filename>
    """
    client.stop_recorder()
    print("Recording stopped.")


def spawn_ego(world, carla_map, spawn_index=12, color='255,255,255',
              model='vehicle.tesla.model3'):
    """Spawn the ego at a known-good spawn point index, same rationale as
    scenario 1: indices are pre-validated road positions, safer than
    searching by raw coordinate."""
    bp_lib = world.get_blueprint_library()
    ego_bp = bp_lib.find(model)
    if ego_bp.has_attribute('color'):
        ego_bp.set_attribute('color', color)

    spawn_pts = carla_map.get_spawn_points()
    spawn_tf = spawn_pts[spawn_index]
    ego = world.try_spawn_actor(ego_bp, spawn_tf)
    if ego is None:
        raise RuntimeError(f"Could not spawn ego at spawn point {spawn_index}")
    return ego


def nearest_light(world, ego, max_dist=80.0):
    """Return the traffic light nearest the ego, or None if none within
    max_dist metres."""
    lights = world.get_actors().filter('traffic.traffic_light')
    light = min(lights, key=lambda l: l.get_location().distance(ego.get_location()),
                default=None)
    if light is None:
        return None
    if light.get_location().distance(ego.get_location()) > max_dist:
        return None
    return light
