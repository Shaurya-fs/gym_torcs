from gymnasium import spaces
import numpy as np
# from os import path
import snakeoil3_gym as snakeoil3
import copy
import csv
import collections as col
import os
import time
from controller.diagnostics import (
    compact_frame_report,
    now_timestamp,
    print_warnings,
    validate_command,
    validate_sensor_frame,
)


class TorcsEnv:
    terminal_judge_start = 500  # Speed limit is applied after this step
    termination_limit_progress = 5  # [km/h], episode terminates if car is running slower than this limit
    default_speed = 50

    initial_reset = True


    def __init__(
        self,
        vision=False,
        throttle=False,
        gear_change=False,
        host="localhost",
        port=3001,
        connection_attempts=10,
    ):
       #print("Init")
        self.vision = vision
        self.throttle = throttle
        self.gear_change = gear_change
        self.host = host
        self.port = port
        self.connection_attempts = connection_attempts
        self.telemetry_path = os.path.join(os.getcwd(), "controller_telemetry.csv")
        self.telemetry_header_written = os.path.exists(self.telemetry_path)

        self.initial_run = True

        ##print("launch torcs")
        """self.vision = vision
        self.throttle = throttle

        self.gear_change = gear_change

        self.initial_run = True"""

        print(
            "Manual TORCS mode: start TORCS with scr_server 1 before running "
            "this program. The default UDP port is 3001."
        )
        """os.system('pkill torcs')
        time.sleep(0.5)
        if self.vision is True:
            os.system('torcs -nofuel -nodamage -nolaptime  -vision &')
        else:
            os.system('torcs  -nofuel -nodamage -nolaptime &')
        time.sleep(0.5)
        os.system('sh autostart.sh')
        time.sleep(0.5)
        """

        """
        # Modify here if you use multiple tracks in the environment
            self.client = snakeoil3.Client(
                H=self.host,
                p=self.port,
                vision=self.vision,
                max_connection_attempts=self.connection_attempts,
            )  # Open new UDP in vtorcs
        self.client.MAX_STEPS = np.inf

        client = self.client
        client.get_servers_input()  # Get the initial input from torcs

        obs = client.S.d  # Get the current full-observation from torcs
        """
        if throttle is False:
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,))
        elif gear_change is True:
            self.action_space = spaces.Box(
                low=np.array([-1.0, 0.0, 0.0, -1.0], dtype=np.float32),
                high=np.array([1.0, 1.0, 1.0, 6.0], dtype=np.float32),
                dtype=np.float32,
            )
        else:
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,))

        if vision is False:
            high = np.array([1., np.inf, np.inf, np.inf, 1., np.inf, 1., np.inf], dtype=np.float32)
            low = np.array([0., -np.inf, -np.inf, -np.inf, 0., -np.inf, 0., -np.inf], dtype=np.float32)
            self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        else:
            high = np.array([1., np.inf, np.inf, np.inf, 1., np.inf, 1., np.inf, 255], dtype=np.float32)
            low = np.array([0., -np.inf, -np.inf, -np.inf, 0., -np.inf, 0., -np.inf, 0], dtype=np.float32)
            self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

    def step(self, u):
       #print("Step")
        # convert thisAction to the actual torcs actionstr
        client = self.client

        this_action = self.agent_to_torcs(u)

        # Apply Action
        action_torcs = client.R.d

        # Steering
        action_torcs['steer'] = this_action['steer']  # in [-1, 1]

        #  Simple Autnmatic Throttle Control by Snakeoil
        if self.throttle is False:
            target_speed = self.default_speed
            if client.S.d['speedX'] < target_speed - (client.R.d['steer']*50):
                client.R.d['accel'] += .01
            else:
                client.R.d['accel'] -= .01

            if client.R.d['accel'] > 0.2:
                client.R.d['accel'] = 0.2

            if client.S.d['speedX'] < 10:
                client.R.d['accel'] += 1/(client.S.d['speedX']+.1)

            # Traction Control System
            if ((client.S.d['wheelSpinVel'][2]+client.S.d['wheelSpinVel'][3]) -
               (client.S.d['wheelSpinVel'][0]+client.S.d['wheelSpinVel'][1]) > 5):
                action_torcs['accel'] -= .2
        else:
           action_torcs['accel'] = max(0.0, min(1.0, float(this_action['accel'])))
           action_torcs['brake'] = max(0.0, min(1.0, float(this_action.get('brake', 0.0))))
        #  Automatic Gear Change by Snakeoil
        if self.gear_change :
            action_torcs['gear'] = int(this_action.get('gear', client.S.d['gear']))
        else:
            #  Automatic Gear Change by Snakeoil is possible
            action_torcs['gear'] = 1
            """
            if client.S.d['speedX'] > 50:
                action_torcs['gear'] = 2
            if client.S.d['speedX'] > 80:
                action_torcs['gear'] = 3
            if client.S.d['speedX'] > 110:
                action_torcs['gear'] = 4
            if client.S.d['speedX'] > 140:
                action_torcs['gear'] = 5
            if client.S.d['speedX'] > 170:
                action_torcs['gear'] = 6
            """

        self._trace_command_flow(u, this_action, action_torcs)

        # Save the privious full-obs from torcs for the reward calculation
        obs_pre = copy.deepcopy(client.S.d)
        print_warnings(validate_command(action_torcs, "packet.pre_send"))

        # One-Step Dynamics Update #################################
        # Apply the Agent's action into torcs
        client.respond_to_server()
        packet_values = copy.deepcopy(client.R.d)
        print_warnings(validate_command(packet_values, "packet.final"))
        # Get the response of TORCS
        client.get_servers_input()

        # Get the current full-observation from torcs
        obs = client.S.d

        # Make an obsevation from a raw observation vector from TORCS
        self.observation = self.make_observaton(obs)

        # Reward setting Here #######################################
        # direction-dependent positive reward
        track = np.array(obs['track'])
        sp = np.array(obs['speedX'])
        progress = sp*np.cos(obs['angle'])
        reward = progress

        # collision detection
        if obs['damage'] - obs_pre['damage'] > 0:
            reward = -1

        # Termination judgement #########################
        episode_terminate = False
        if track.min() < 0:  # Episode is terminated if the car is out of track
            reward = - 1
            episode_terminate = True
            client.R.d['meta'] = True

        if self.terminal_judge_start < self.time_step: # Episode terminates if the progress of agent is small
            if progress < self.termination_limit_progress:
                episode_terminate = True
                client.R.d['meta'] = True

        if np.cos(obs['angle']) < 0: # Episode is terminated if the agent runs backward
            episode_terminate = True
            client.R.d['meta'] = True


        if client.R.d['meta'] is True: # Send a reset signal
            self.initial_run = False
            client.respond_to_server()

        self.time_step += 1
        telemetry_context = getattr(u, "telemetry_context", {})
        self._write_telemetry(telemetry_context, packet_values)
        compact_frame_report(telemetry_context, packet_values)

        return self.get_obs(), reward, client.R.d['meta'], {}

    def reset(self, relaunch=False):
        #print("Reset")

        self.time_step = 0

        if self.initial_reset is not True:
            self.client.R.d['meta'] = True
            self.client.respond_to_server()

            ## TENTATIVE. Restarting TORCS every episode suffers the memory leak bug!
            if relaunch is True:
                self.reset_torcs()
                print("### TORCS is RELAUNCHED ###")

        # Modify here if you use multiple tracks in the environment
        self.client = snakeoil3.Client(
            H=self.host,
            p=self.port,
            vision=self.vision,
            max_connection_attempts=self.connection_attempts,
        )  # Open new UDP in vtorcs
        self.client.MAX_STEPS = np.inf

        client = self.client
        client.get_servers_input()  # Get the initial input from torcs

        obs = client.S.d  # Get the current full-observation from torcs
        self.observation = self.make_observaton(obs)

        self.last_u = None

        self.initial_reset = False
        return self.get_obs()

    def end(self):
        client = getattr(self, "client", None)
        if client is not None:
            client.shutdown()

    def get_obs(self):
        return self.observation

    def reset_torcs(self):
       """#print("relaunch torcs")
        os.system('pkill torcs')
        time.sleep(0.5)
        if self.vision is True:
            os.system('torcs -nofuel -nodamage -nolaptime -vision &')
        else:
            os.system('torcs -nofuel -nodamage -nolaptime &')
        time.sleep(0.5)
        os.system('sh autostart.sh')
        time.sleep(0.5)"""
       pass

    def agent_to_torcs(self, u):
        torcs_action = {'steer': u[0]}

        if self.throttle is True:  # throttle action is enabled
            torcs_action.update({'accel': u[1]})

        if self.gear_change is True: # gear change action is enabled
            if len(u) >= 4:
                torcs_action.update({'brake': u[2]})
                torcs_action.update({'gear': u[3]})
            else:
                torcs_action.update({'gear': u[2]})

        return torcs_action

    def _trace_command_flow(self, u, this_action, action_torcs):
        telemetry_context = getattr(u, "telemetry_context", {})
        frame = telemetry_context.get("frame", self.time_step)
        if frame % 20 != 0:
            return

        ai_action = telemetry_context.get("ai_action", {})
        command = telemetry_context.get("command", {})
        driving_context = telemetry_context.get("driving_context", {})
        print(
            "[COMMAND TRACE] "
            f"frame={frame} state={driving_context.get('fsm_state', '')} "
            f"AI={ai_action.get('longitudinal', '')}/{ai_action.get('gear', '')} "
            f"FINAL=steer:{command.get('steering', 0.0):.3f} "
            f"throttle:{command.get('acceleration', 0.0):.3f} "
            f"brake:{command.get('brake', 0.0):.3f} gear:{command.get('gear', 0)} "
            f"AGENT=[{u[0]:.3f}, {u[1]:.3f}, {u[2]:.3f}, {int(u[3]) if len(u) > 3 else 'NA'}] "
            f"TORCS=steer:{action_torcs.get('steer', 0.0):.3f} "
            f"accel:{action_torcs.get('accel', 0.0):.3f} "
            f"brake:{action_torcs.get('brake', 0.0):.3f} "
            f"gear:{action_torcs.get('gear', 0)}"
        )


    def obs_vision_to_image_rgb(self, obs_image_vec):
        image_vec =  obs_image_vec
        rgb = []
        temp = []
        # convert size 64x64x3 = 12288 to 64x64=4096 2-D list 
        # with rgb values grouped together.
        # Format similar to the observation in openai gym
        for i in range(0,12286,3):
            temp.append(image_vec[i])
            temp.append(image_vec[i+1])
            temp.append(image_vec[i+2])
            rgb.append(temp)
            temp = []
        return np.array(rgb, dtype=np.uint8)

    def make_observaton(self, raw_obs):
        print_warnings(validate_sensor_frame(raw_obs, "raw_udp"))

        if self.vision is False:

            names = [
                'focus',
                'speedX',
                'speedY',
                'speedZ',
                'angle',
                'opponents',
                'rpm',
                'track',
                'trackPos',
                'wheelSpinVel',
                'gear',
                'fuel',
                'damage',
                'curLapTime',
                'distFromStart',
                'distRaced'
            ]

            Observation = col.namedtuple('Observation', names)

            return Observation(
                focus=np.array(raw_obs['focus'], dtype=np.float32) / 200.,
                speedX=np.array(raw_obs['speedX'], dtype=np.float32),
                speedY=np.array(raw_obs['speedY'], dtype=np.float32),
                speedZ=np.array(raw_obs['speedZ'], dtype=np.float32),
                angle=np.array(raw_obs['angle'], dtype=np.float32),
                opponents=np.array(raw_obs['opponents'], dtype=np.float32) / 200.,
                rpm=np.array(raw_obs['rpm'], dtype=np.float32),
                track=np.array(raw_obs['track'], dtype=np.float32),
                trackPos=np.array(raw_obs['trackPos'], dtype=np.float32),
                wheelSpinVel=np.array(raw_obs['wheelSpinVel'], dtype=np.float32),
                gear=int(raw_obs['gear']),
                fuel=np.array(raw_obs['fuel'], dtype=np.float32),
                damage=np.array(raw_obs['damage'], dtype=np.float32),
                curLapTime=np.array(raw_obs.get('curLapTime', 0.0), dtype=np.float32),
                distFromStart=np.array(raw_obs.get('distFromStart', 0.0), dtype=np.float32),
                distRaced=np.array(raw_obs.get('distRaced', 0.0), dtype=np.float32)
            )

        else:

            names = [
                'focus',
                'speedX',
                'speedY',
                'speedZ',
                'angle',
                'opponents',
                'rpm',
                'track',
                'trackPos',
                'wheelSpinVel',
                'gear',
                'fuel',
                'damage',
                'curLapTime',
                'distFromStart',
                'distRaced',
                'img'
            ]

            Observation = col.namedtuple('Observation', names)

            image_rgb = self.obs_vision_to_image_rgb(raw_obs['img'])

            return Observation(
                focus=np.array(raw_obs['focus'], dtype=np.float32) / 200.,
                speedX=np.array(raw_obs['speedX'], dtype=np.float32),
                speedY=np.array(raw_obs['speedY'], dtype=np.float32),
                speedZ=np.array(raw_obs['speedZ'], dtype=np.float32),
                angle=np.array(raw_obs['angle'], dtype=np.float32),
                opponents=np.array(raw_obs['opponents'], dtype=np.float32) / 200.,
                rpm=np.array(raw_obs['rpm'], dtype=np.float32),
                track=np.array(raw_obs['track'], dtype=np.float32),
                trackPos=np.array(raw_obs['trackPos'], dtype=np.float32),
                wheelSpinVel=np.array(raw_obs['wheelSpinVel'], dtype=np.float32),
                gear=int(raw_obs['gear']),
                fuel=np.array(raw_obs['fuel'], dtype=np.float32),
                damage=np.array(raw_obs['damage'], dtype=np.float32),
                curLapTime=np.array(raw_obs.get('curLapTime', 0.0), dtype=np.float32),
                distFromStart=np.array(raw_obs.get('distFromStart', 0.0), dtype=np.float32),
                distRaced=np.array(raw_obs.get('distRaced', 0.0), dtype=np.float32),
                img=image_rgb
            )

    def _write_telemetry(self, context, packet_values):
        if not context:
            return

        sensor = context.get("sensor_data", {})
        vehicle = context.get("vehicle", {})
        road = context.get("road", {})
        corner = context.get("corner", {})
        plan = context.get("plan", {})
        command = context.get("command", {})
        driving_context = context.get("driving_context", {})
        ai_action = context.get("ai_action", {})
        track = list(sensor.get("track", []))
        while len(track) < 19:
            track.append("")

        headers = [
            "timestamp", "frame", "lap", "lap_time_s", "dist_from_start_m", "dist_raced_m",
            "speed_x_kmh", "speed_y_kmh", "speed_z_kmh", "rpm", "gear", "fuel_l",
            "damage", "track_pos_norm", "angle_rad", "wheel_slip",
            "corner_type", "road_visibility_m", "road_curvature_signed",
            "planner_target_speed_kmh", "planner_target_gear", "planner_steering_gain",
            "planner_target_track_pos", "planner_brake_point_m",
            "fsm_state", "fsm_time_in_state", "ai_longitudinal_action", "ai_gear_action",
            "ai_reason", "ai_speed_error", "ai_vehicle_stability",
            "controller_steering", "controller_throttle", "controller_brake", "controller_gear",
            "recovery_active",
            "packet_steer", "packet_accel", "packet_brake", "packet_gear", "packet_meta",
        ] + [f"track_{idx:02d}_m" for idx in range(19)]

        row = {
            "timestamp": now_timestamp(),
            "frame": context.get("frame", 0),
            "lap": sensor.get("lap", 0),
            "lap_time_s": sensor.get("curLapTime", 0.0),
            "dist_from_start_m": sensor.get("distFromStart", 0.0),
            "dist_raced_m": sensor.get("distRaced", 0.0),
            "speed_x_kmh": vehicle.get("speed_x", sensor.get("speedX", 0.0)),
            "speed_y_kmh": vehicle.get("speed_y", sensor.get("speedY", 0.0)),
            "speed_z_kmh": vehicle.get("speed_z", sensor.get("speedZ", 0.0)),
            "rpm": vehicle.get("rpm", sensor.get("rpm", 0.0)),
            "gear": vehicle.get("gear", sensor.get("gear", 0)),
            "fuel_l": sensor.get("fuel", 0.0),
            "damage": sensor.get("damage", 0.0),
            "track_pos_norm": vehicle.get("track_pos", sensor.get("trackPos", 0.0)),
            "angle_rad": vehicle.get("steering_angle", sensor.get("angle", 0.0)),
            "wheel_slip": vehicle.get("wheel_slip", 0.0),
            "corner_type": corner.get("corner_type", ""),
            "road_visibility_m": road.get("forward_visibility", 0.0),
            "road_curvature_signed": road.get("signed_curvature", 0.0),
            "planner_target_speed_kmh": plan.get("target_speed", 0.0),
            "planner_target_gear": plan.get("target_gear", 0),
            "planner_steering_gain": plan.get("steering_gain", 0.0),
            "planner_target_track_pos": plan.get("target_track_pos", 0.0),
            "planner_brake_point_m": plan.get("brake_point", 0.0),
            "fsm_state": driving_context.get("fsm_state", ""),
            "fsm_time_in_state": driving_context.get("time_in_state", 0),
            "ai_longitudinal_action": ai_action.get("longitudinal", ""),
            "ai_gear_action": ai_action.get("gear", ""),
            "ai_reason": ai_action.get("reason", ""),
            "ai_speed_error": driving_context.get("speed_error", 0.0),
            "ai_vehicle_stability": driving_context.get("vehicle_stability", 0.0),
            "controller_steering": command.get("steering", 0.0),
            "controller_throttle": command.get("acceleration", 0.0),
            "controller_brake": command.get("brake", 0.0),
            "controller_gear": command.get("gear", 0),
            "recovery_active": context.get("recovery_active", False),
            "packet_steer": packet_values.get("steer", 0.0),
            "packet_accel": packet_values.get("accel", 0.0),
            "packet_brake": packet_values.get("brake", 0.0),
            "packet_gear": packet_values.get("gear", 0),
            "packet_meta": packet_values.get("meta", 0),
        }
        for idx in range(19):
            row[f"track_{idx:02d}_m"] = track[idx]

        write_header = not self.telemetry_header_written
        with open(self.telemetry_path, "a", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=headers)
            if write_header:
                writer.writeheader()
            writer.writerow(row)
        self.telemetry_header_written = True
