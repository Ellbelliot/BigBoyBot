import math

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.orientation import Orientation
from util.vec import Vec3


class BigBoyBot(BaseAgent):

    def initialize_agent(self):
        # This runs once before the bot starts up
        self.controller_state = SimpleControllerState()
        self.gotta_go_fast = False
        self.flipping = False
        self.possession = False
        self.mode = "attacking"
        self.boosting = False
        self.jumping = False
        self.drifting = False
        self.status = "driving"
        self.pitching = 0

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        # Constant values can be found the the FieldInfo:
        info = self.get_field_info()

        ball_location = Vec3(packet.game_ball.physics.location)
        ball_velocity = Vec3(packet.game_ball.physics.velocity)
        total_ball_speed = abs(ball_velocity[0]) + abs(ball_velocity[1]) + abs(ball_velocity[2])

        my_car = packet.game_cars[self.index]
        other_cars = packet.game_cars

        car_location = Vec3(my_car.physics.location)
        other_car_location = Vec3(other_cars[1 - self.index].physics.location)

        car_to_ball = ball_location - car_location
        car_ball_distance = abs(car_to_ball[0]) + abs(car_to_ball[1])
        other_car_to_ball = ball_location - other_car_location
        other_car_ball_distance = abs(other_car_to_ball[0]) + abs(other_car_to_ball[1])

        goals = info.goals
        my_goal = goals[my_car.team]
        other_goal = goals[1 - my_car.team]

        other_goal_to_ball = Vec3(other_goal.location) - Vec3(ball_location)
        other_goal_to_ball_distance = abs(other_goal_to_ball[0]) + abs(other_goal_to_ball[1])

        other_goal_to_car = Vec3(other_goal.location) - Vec3(car_location)

        other_car_velocity = Vec3(other_cars[1 - self.index].physics.velocity)

        goal_to_opponent = Vec3(my_goal.location) - Vec3(other_car_location)
        goal_opponent_distance = abs(goal_to_opponent[0]) + abs(goal_to_opponent[1])
        goal_to_car = Vec3(my_goal.location) - car_location
        goal_car_distance = abs(goal_to_car[0]) + abs(goal_to_car[1])
        goal_to_ball = Vec3(my_goal.location) - Vec3(ball_location)
        goal_ball_distance = abs(goal_to_ball[0]) + abs(goal_to_ball[1])

        between_goal_ball = Vec3((Vec3(my_goal.location)[0] + car_location[0]) / 2, (Vec3(my_goal.location)[1]
                                                                                     + car_location[1]) / 1,
                                 car_location[2])

        # Find the direction of our car using the Orientation class
        car_orientation = Orientation(my_car.physics.rotation)
        car_direction = car_orientation.forward
        car_velocity = Vec3(my_car.physics.velocity)
        car_speed = abs(car_velocity[0]) + abs(car_velocity[1])

        vertical_difference = abs(car_location[2] - ball_location[2])

        ball_predictions = self.get_ball_prediction_struct()
        ball_prediction = Vec3(ball_predictions.slices[0].physics.location)

        self.boosting = False
        self.jumping = False
        self.flipping = False
        self.drifting = False
        self.pitching = 0

        distance_to_boost = 100000
        closest_boost = None

        for i in range(info.num_boosts):
            pad = info.boost_pads[i]
            if pad.is_full_boost and packet.game_boosts[i].is_active:
                car_to_boost = Vec3(pad.location) - Vec3(car_location)
                distance_to_pad = abs(car_to_boost[0]) + abs(car_to_boost[1])
                if distance_to_pad < distance_to_boost:
                    distance_to_boost = distance_to_pad
                    closest_boost = pad

        car_to_boost = Vec3(closest_boost.location) - Vec3(car_location)
        steer_correction_radians_to_boost = find_correction(car_direction, car_to_boost)

        steer_correction_radians_to_goal = find_correction(car_direction, goal_to_car)
        distance_to_goal = abs(goal_to_car[0]) + abs(goal_to_car[0])

        steer_correction_radians_to_ball = find_correction(car_direction, car_to_ball)

        pitch_correction_radians_to_ball = find_pitch_correction(car_direction, car_to_ball)

        is_going_in = False
        closest_location = ball_location
        car_to_closest_location = closest_location - car_location

        if ball_predictions is not None:
            for i in range(0, ball_predictions.num_slices):
                prediction_slice = ball_predictions.slices[i]
                location = prediction_slice.physics.location
                if 0 < (prediction_slice.game_seconds - packet.game_info.seconds_elapsed) - \
                        (abs((Vec3(location)[0] - car_location[0]) / (1 + car_velocity[0])) + \
                        abs((Vec3(location)[1] - car_location[1]) / (1 + car_velocity[1]))) / 50 < 0.1:
                    ball_prediction = Vec3(location)
                    correct_slice = prediction_slice
                goal_to_location = Vec3(my_goal.location) - Vec3(location)
                car_to_location = Vec3(location) - car_location
                if abs(goal_to_location[0]) + abs(goal_to_location[1]) + abs(goal_to_location[2]) < 1500:
                    is_going_in = True
                if abs(car_to_location[0]) + abs(car_to_location[1]) < abs(car_to_closest_location[0]) + \
                                                                            abs(car_to_closest_location[1]):
                   closest_location = Vec3(location)
                   car_to_closest_location = car_to_location


        if goal_car_distance > other_goal_to_ball_distance:
            offset = Vec3(-other_goal_to_ball[0] / abs(other_goal_to_car[0] + 0.1), -other_goal_to_ball[1] /
                          abs(other_goal_to_car[1] + 0.1), -other_goal_to_ball[2] / abs(other_goal_to_car[2] + 0.1)) * \
            (50 + (car_speed / 10000) * 50)
        else:
            offset = Vec3(goal_to_ball[0] / abs(goal_to_car[0] + 0.1), goal_to_ball[1] /
                          abs(goal_to_car[1] + 0.1), goal_to_ball[2] / abs(goal_to_car[2] + 0.1)) * \
            (50 + (car_speed / 10000) * 50)


        time_to_hit = (abs((ball_prediction[0] - car_location[0]) / (1 + car_velocity[0])) + \
                      abs((ball_prediction[1] - car_location[1]) / (1 + car_velocity[1]))) / 50

        other_time_to_hit = (abs((ball_prediction[0] - other_car_location[0]) / (1 + other_car_velocity[0])) + \
                      abs((ball_prediction[1] - other_car_location[1]) / (1 + other_car_velocity[1]))) / 50

        car_to_hit_location = ball_prediction + offset - car_location

        car_to_shadow_location = ((ball_prediction + offset) + my_goal.location) / 2 - car_location

        steer_correction_radians_to_ball_prediction = find_correction(car_direction, car_to_hit_location)
        distance_to_hit = abs(car_to_hit_location[0]) + abs(car_to_hit_location[0])

        steer_correction_radians_to_shadow = find_correction(car_direction, car_to_shadow_location)
        distance_to_shadow = abs(car_to_shadow_location[0]) + abs(car_to_shadow_location[0])

        steer_correction_radians_to_save = find_correction(car_direction, car_to_closest_location)
        distance_to_save = abs(car_to_closest_location[0]) + abs(car_to_closest_location[0])

        steer_correction_radians_to_focus = 0
        distance_to_focus = 0

        close_distance = 600
        far_distance = 2000

        ball_height_to_jump = 180
        ball_height_to_flip = 200
        ball_height_to_aerial = 400

        height_of_large_jump = 80
        height_to_flip = 100

        if goal_car_distance < goal_ball_distance < other_car_ball_distance * 2 and is_going_in:
            self.mode = "defending"
        elif time_to_hit > other_time_to_hit + 0.2:
            self.mode = "shadowing"
        elif abs(steer_correction_radians_to_boost) < math.pi / (2 * (1 + (my_car.boost / 5))) and \
                distance_to_boost < other_car_ball_distance and distance_to_boost < car_ball_distance:
            self.mode = "getting boost"
        else:
            self.mode = "attacking"

        if self.mode == "attacking":
            if car_speed < total_ball_speed * 0.8 or time_to_hit + 0.2 > other_time_to_hit and not \
                    my_car.is_super_sonic and car_ball_distance > (close_distance + car_speed - total_ball_speed):
                if my_car.boost > 0:
                    self.status = "boosting"
                elif 1000 < car_speed < car_ball_distance and abs(steer_correction_radians_to_ball) < math.pi / 8:
                    self.status = "flipping for speed"
            elif car_ball_distance < close_distance + car_speed / 2 - total_ball_speed / 2:
                if ball_height_to_jump < ball_prediction[2] < ball_height_to_flip and \
                        abs(steer_correction_radians_to_ball) < math.pi / 2:
                    self.status = "jumping"
                if ball_height_to_aerial > ball_prediction[2] > ball_height_to_flip and \
                    abs(steer_correction_radians_to_ball) < math.pi / 8 or abs(time_to_hit - other_time_to_hit) < 0.1 \
                        and abs(steer_correction_radians_to_ball) < math.pi / 8:
                    self.status = "flipping for ball"
            else:
                self.status = "driving"

            steer_correction_radians_to_focus = steer_correction_radians_to_ball_prediction
            distance_to_focus = car_ball_distance

        if self.mode == "defending":
            if closest_location[2] > car_location[2] + height_of_large_jump:
                self.status = "jumping"
            if my_car.boost > 0:
                self.status = "boosting"
            elif 1000 > car_speed > 500 and abs(steer_correction_radians_to_save) < math.pi / 8:
                self.status = "flipping for speed"
            steer_correction_radians_to_focus = steer_correction_radians_to_save
            distance_to_focus = distance_to_save

        if self.mode == "shadowing":
            if my_car.boost > 0 and car_speed < total_ball_speed:
                self.status = "boosting"
            elif total_ball_speed > car_speed > 1000 and abs(steer_correction_radians_to_shadow) < math.pi / 8:
                self.status = "flipping for speed"
            else:
                self.status = "driving"
            steer_correction_radians_to_focus = steer_correction_radians_to_shadow
            distance_to_focus = distance_to_shadow

        if self.mode == "getting boost":
            steer_correction_radians_to_focus = steer_correction_radians_to_boost
            distance_to_focus = distance_to_boost

        # flipping to hit the ball
        if self.status == "flipping for ball":
            if my_car.has_wheel_contact and not my_car.jumped or car_location[2] + 50 < ball_prediction[2]:
                self.jumping = True
            else:
                if not self.controller_state.jump and not my_car.double_jumped and not my_car.has_wheel_contact:
                    self.pitching = -1
                    self.jumping = True
                    self.flipping = False

        # flipping to go fast
        if self.status == "flipping for speed":
            if my_car.has_wheel_contact and not my_car.jumped and car_speed > 1500 \
                    or car_location[2] < height_of_large_jump and \
                    my_car.jumped:
                self.jumping = True
            else:
                if not self.controller_state.jump and not my_car.double_jumped and not my_car.has_wheel_contact:
                    self.pitching = -1
                    self.jumping = True
                    self.flipping = False

        # just jumpin'
        if self.status == "doing an aerial":
            if my_car.has_wheel_contact and not my_car.jumped:
                self.jumping = True
            elif not my_car.double_jumped and my_car.jumped and not self.controller_state.jump:
                self.jumping = True
            else:
                self.boosting = True
                self.controller_state.pitch = -pitch_correction_radians_to_ball * 5
                if self.controller_state.pitch > 1:
                    self.controller_state.pitch = 1
                if self.controller_state.pitch < -1:
                    self.controller_state.pitch = -1
        else:
            if abs(my_car.physics.rotation.pitch) > 0.1 and not my_car.has_wheel_contact and \
                    not self.jumping:
                self.controller_state.pitch = -my_car.physics.rotation.pitch
                if self.controller_state.pitch > 1:
                    self.controller_state.pitch = 1
                if self.controller_state.pitch < -1:
                    self.controller_state.pitch = -1

        # just jumpin'
        if self.status == "jumping":
            if my_car.has_wheel_contact and not my_car.jumped or car_location[2] < ball_prediction[2]:
                self.jumping = True

        # just boostin'
        if self.status == "boosting":
            if not my_car.is_super_sonic:
                self.boosting = True

        if abs(my_car.physics.rotation.roll) > 0.2 and not my_car.has_wheel_contact and \
                self.status != "flipping for speed":
            self.controller_state.roll = -my_car.physics.rotation.roll / 5
            if self.controller_state.roll > 1:
                self.controller_state.roll = 1
            if self.controller_state.roll < -1:
                self.controller_state.roll = -1

        if car_ball_distance > 500 and car_speed < 200 and abs(my_car.physics.rotation.pitch) > 0.1:
            self.jumping = True

        speed = 1
        speed = (0.1 + distance_to_focus * 2) / 1000
        if speed > 1:
            speed = 1
        if speed < 0:
            speed = 0
        turn = -steer_correction_radians_to_focus * 3
        if 2 > abs(steer_correction_radians_to_focus) > 1:
            self.drifting = True
            self.boosting = False
            self.jumping = False
            self.flipping = False
            turn = -steer_correction_radians_to_focus * 3
        elif abs(steer_correction_radians_to_focus) > 2:
            speed = speed * -1
            self.boosting = False
            self.jumping = False
            self.flipping = False
            turn = steer_correction_radians_to_focus * 4
        self.controller_state.throttle = speed

        if turn > 1:
            turn = 1
        if turn < -1:
            turn = -1

        if my_car.has_wheel_contact:
            self.controller_state.steer = turn
        else:
            turn = -steer_correction_radians_to_focus / 2
            if turn > 1:
                turn = 1
            if turn < -1:
                turn = -1
            self.controller_state.yaw = turn

        # boosting
        if self.boosting:
            self.controller_state.boost = True
        else:
            self.controller_state.boost = False

        # jumping
        if self.jumping:
            self.controller_state.jump = True
        else:
            self.controller_state.jump = False

        # drifting
        if self.drifting:
            self.controller_state.handbrake = True
        else:
            self.controller_state.handbrake = False

        # pitching
        self.controller_state.pitch = self.pitching

        action_display = str(self.mode + self.status)
        draw_debug(self.renderer, my_car.physics.location, closest_location, ball_prediction, action_display)

        return self.controller_state

def find_correction(current: Vec3, ideal: Vec3) -> float:
    # Finds the angle from current to ideal vector in the xy-plane. Angle will be between -pi and +pi.

    # The in-game axes are left handed, so use -x
    current_in_radians = math.atan2(current.y, -current.x)
    ideal_in_radians = math.atan2(ideal.y, -ideal.x)

    diff = ideal_in_radians - current_in_radians

    # Make sure that diff is between -pi and +pi.
    if abs(diff) > math.pi:
        if diff < 0:
            diff += 2 * math.pi
        else:
            diff -= 2 * math.pi

    return diff


def find_pitch_correction(current: Vec3, ideal: Vec3) -> float:
    # Finds the angle from current to ideal vector in the xy-plane. Angle will be between -pi and +pi.

    # The in-game axes are left handed, so use -x
    current_in_radians = math.atan2(current.y, -current.x)
    ideal_in_radians = math.atan2(ideal.y, -ideal.x)

    diff = ideal_in_radians - current_in_radians

    # Make sure that diff is between -pi and +pi.
    if abs(diff) > math.pi:
        if diff < 0:
            diff += 2 * math.pi
        else:
            diff -= 2 * math.pi

    return diff


def draw_debug(renderer, pos1, pos2, pos3, action_display):
    renderer.begin_rendering()
    # draw a line from the car to the ball
    renderer.draw_line_3d(pos1, pos2, renderer.white())
    renderer.draw_line_3d(pos2, pos3, renderer.red())
    # print the action that the bot is taking
    renderer.draw_string_3d(pos1, 2, 2, action_display, renderer.white())
    renderer.end_rendering()
