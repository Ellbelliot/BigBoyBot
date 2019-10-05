import math

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.orientation import Orientation
from util.vec import Vec3


class BigBoyBot(BaseAgent):

    def initialize_agent(self):
        # This runs once before the bot starts up
        self.controller_state = SimpleControllerState()
        self.optimal_ball_location = 0
        self.prediction_time = 0
        self.speed = 1
        self.status = 0
        self.state = 0

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        # Constant values can be found the the FieldInfo:
        info = self.get_field_info()
        ball_predictions = self.get_ball_prediction_struct()

        ball = packet.game_ball
        ball_location = Vec3(ball.physics.location)
        ball_velocity = Vec3(ball.physics.velocity)
        ball_speed = abs(ball_velocity[0]) + abs(ball_velocity[1]) + abs(ball_velocity[2])

        bot = packet.game_cars[self.index]
        bot_location = Vec3(bot.physics.location)
        bot_velocity = Vec3(bot.physics.velocity)
        bot_speed = abs(bot_velocity[0]) + abs(bot_velocity[1]) + abs(bot_velocity[2])

        bot_to_ball = ball_location - bot_location
        bot_ball_distance = abs(bot_to_ball[0]) + abs(bot_to_ball[1])

        other_bot = packet.game_cars[1 - self.index]
        other_bot_location = Vec3(other_bot.physics.location)
        other_bot_velocity = Vec3(other_bot.physics.velocity)
        other_bot_speed = abs(other_bot_velocity[0]) + abs(other_bot_velocity[1]) + abs(other_bot_velocity[2])

        other_bot_to_ball = ball_location - other_bot_location
        other_bot_ball_distance = abs(other_bot_to_ball[0]) + abs(other_bot_to_ball[1])

        goal = info.goals[bot.team]
        goal_location = Vec3(goal.location)

        goal_to_opponent = goal_location - Vec3(other_bot_location)
        goal_opponent_distance = abs(goal_to_opponent[0]) + abs(goal_to_opponent[1])
        goal_to_bot = goal_location - bot_location
        goal_bot_distance = abs(goal_to_bot[0]) + abs(goal_to_bot[1])
        goal_to_ball = goal_location - Vec3(ball_location)
        goal_ball_distance = abs(goal_to_ball[0]) + abs(goal_to_ball[1])

        other_goal = info.goals[1 - bot.team]
        other_goal_location = Vec3(other_goal.location)

        other_goal_to_ball = other_goal_location - Vec3(ball_location)
        other_goal_to_ball_distance = abs(other_goal_to_ball[0]) + abs(other_goal_to_ball[1])
        other_goal_to_bot = other_goal_location - Vec3(bot_location)

        full_big_pads = []
        closest_boost_location = 0
        closest_boost_distance = 1000000
        for i in range(info.num_boosts):
            pad = info.boost_pads[i]
            if pad.is_full_boost and packet.game_boosts[i].is_active:
                full_big_pads.append(pad)
                pad_to_bot = Vec3(pad.location) - Vec3(bot_location)
                if abs(pad_to_bot[0]) + abs(pad_to_bot[1]) < closest_boost_distance:
                    closest_boost_to_bot = pad_to_bot
                    closest_boost_distance = abs(pad_to_bot[0]) + abs(pad_to_bot[1])
                    closest_boost_location = Vec3(pad.location)

        best_score = 0
        for i in range(ball_predictions.num_slices):
            pos = Vec3(ball_predictions.slices[i].physics.location)
            time = ball_predictions.slices[i].game_seconds
            if best_score < get_score(time - packet.game_info.seconds_elapsed,
                    find_eta(bot_location, pos, bot_velocity), find_eta(other_bot_location, pos, other_bot_velocity),
                    pos):
                best_score = get_score(time - packet.game_info.seconds_elapsed,
                    find_eta(bot_location, pos, bot_velocity), find_eta(other_bot_location, pos, other_bot_velocity),
                    pos)
                self.optimal_ball_location = pos
                self.prediction_time = time

        if self.optimal_ball_location == 0:
            self.optimal_ball_location = ball_location

        bot_orientation = Orientation(bot.physics.rotation)
        bot_direction = bot_orientation.forward

        self.controller_state.jump = False
        self.controller_state.pitch = 0
        self.controller_state.boost = False

        radians_to_focus = 0
        radians_to_boost = find_correction(bot_direction, closest_boost_to_bot)
        radians_to_goal = find_correction(bot_direction, goal_to_bot)

        if goal_ball_distance < goal_bot_distance:
            self.status = "defending"
            aim_offset = Vec3(0, 0, 0)
        elif closest_boost_distance / (1 + bot.boost) > 5000 and abs(radians_to_boost) < math.pi / (2 + bot.boost):
            self.status = "grabbing boost"
            aim_offset = Vec3(0, 0, 0)
        elif goal_ball_distance < other_goal_to_ball_distance and bot_ball_distance < other_bot_ball_distance:
            self.status = "clearing"
            aim_offset = Vec3(-other_goal_to_ball[0] / abs(other_goal_to_bot[0] + 0.1), -other_goal_to_ball[1] /
                              abs(other_goal_to_bot[1] + 0.1),
                              -other_goal_to_ball[2] / abs(other_goal_to_bot[2] + 0.1)) * \
                         (50 + (bot_speed / 10000) * 50)
        else:
            self.status = "attacking"
            aim_offset = Vec3(goal_to_ball[0] / abs(goal_to_bot[0] + 0.1), goal_to_ball[1] /
                              abs(goal_to_bot[1] + 0.1), goal_to_ball[2] / abs(goal_to_bot[2] + 0.1)) * \
                         (50 + (bot_speed / 10000) * 50)

        self.optimal_ball_location = self.optimal_ball_location + aim_offset

        bot_to_ball = self.optimal_ball_location - bot_location

        radians_to_ball = find_correction(bot_direction, bot_to_ball)

        if radians_to_focus == radians_to_boost or radians_to_focus == radians_to_goal:
            self.speed = 1
        else:
            self.speed = 1 + find_eta(bot_location, self.optimal_ball_location, bot_velocity) - (self.prediction_time -
                packet.game_info.seconds_elapsed)
            if self.speed > 1.5 or find_eta(bot_location, self.optimal_ball_location, bot_velocity) < \
                    find_eta(other_bot_location, self.optimal_ball_location, other_bot_velocity):
                if bot.boost != 0:
                    self.controller_state.boost = True
                else:
                    self.state = "flipping for speed"

        if self.speed > 1:
            self.speed = 1
        elif self.speed < 0:
            self.speed = 0

        if self.status == "defending":
            radians_to_focus = radians_to_goal
            if ball_speed > bot_speed and abs(radians_to_goal) < math.pi / 4:
                if bot.boost != 0:
                    self.controller_state.boost = True
                else:
                    self.state = "flipping for speed"
            else:
                self.state = "driving"

        elif self.status == "grabbing boost":
            radians_to_focus = radians_to_boost
            if goal_ball_distance * 1.5 > goal_bot_distance and abs(radians_to_boost) < math.pi / 4:
                if bot.boost != 0:
                    self.controller_state.boost = True
                else:
                    self.state = "flipping for speed"
            else:
                self.state = "driving"

        elif self.status == "clearing":
            radians_to_focus = radians_to_ball
            if find_eta(bot_location, self.optimal_ball_location, bot_velocity) < 0.2 and \
                    abs(radians_to_ball) < math.pi / 4:
                self.state = "flipping for ball"
            else:
                self.state = "driving"
        elif self.status == "attacking":
            radians_to_focus = radians_to_ball
            if find_eta(bot_location, self.optimal_ball_location, bot_velocity) < 0.2 and \
                    abs(radians_to_ball) < math.pi / 6 and ball_speed < 700 and find_eta(other_bot_location,
                    self.optimal_ball_location, other_bot_velocity) > 1:
                self.state = "flipping for ball"
            else:
                self.state = "driving"

        # flipping to go faster
        if self.state == "flipping for speed":
            if bot.has_wheel_contact and not bot.jumped:
                self.controller_state.jump = True
            else:
                if not self.controller_state.jump and not bot.double_jumped and not bot.has_wheel_contact and \
                        bot_location[2] > 100:
                    self.controller_state.pitch = -1
                    self.controller_state.jump = True

        # flipping to hit the ball
        if self.state == "flipping for ball":
            if bot.has_wheel_contact and not bot.jumped:
                self.controller_state.jump = True
            else:
                if not self.controller_state.jump and not bot.double_jumped and not bot.has_wheel_contact and \
                        abs(bot_location[2] - ball_location[2]) < 100:
                    self.controller_state.pitch = -1
                    self.controller_state.jump = True

        #movin' and turnin'
        turn = -radians_to_focus * 2

        if abs(radians_to_focus) > math.pi * 0.75:
            self.speed =  self.speed * -1

        if turn > 1:
            turn = 1
        elif turn < -1:
            turn = -1

        self.controller_state.throttle = self.speed
        self.controller_state.steer = turn

        draw_debug(self.renderer, bot_location, self.optimal_ball_location, ball_location, self.status)

        return self.controller_state


def find_eta(location1: Vec3, location2: Vec3, speed: Vec3) -> float:
    pos_to_pos = location1 - location2
    top_speed = 1000
    if (pos_to_pos[0] + 1) / abs(pos_to_pos[0] + 1) == (speed[0] + 1) / abs(speed[0] + 1):
        xtime = abs(pos_to_pos[0] / (top_speed + abs(speed[0] / 2)))
    else:
        xtime = abs(pos_to_pos[0] / (top_speed - abs(speed[0] / 2)))
    if (pos_to_pos[1] + 1) / abs(pos_to_pos[1] + 1) == (speed[1] + 1) / (abs(speed[1] + 1)):
        ytime = abs(pos_to_pos[1] / (top_speed + abs(speed[1] / 2)))
    else:
        ytime = abs(pos_to_pos[1] / (top_speed - abs(speed[1] / 2)))
    return xtime + ytime


def get_score(ball_prediction_time: float, bot_eta: float, other_bot_eta: float, ball_prediction: Vec3) -> float:
    bot_arrival_score = 10 - bot_eta
    other_bot_arrival_score = other_bot_eta
    soon_score = 10 - ball_prediction_time * 2
    height_score = 10 - ball_prediction[2] / 250

    return bot_arrival_score + other_bot_arrival_score + soon_score + height_score



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


def draw_debug(renderer, pos1, pos2, pos3, action_display):
    renderer.begin_rendering()
    # draw a line from the car to the ball
    renderer.draw_line_3d(pos1, pos2, renderer.white())
    renderer.draw_line_3d(pos2, pos3, renderer.red())
    # print the action that the bot is taking
    renderer.draw_string_3d(pos1, 2, 2, action_display, renderer.white())
    renderer.end_rendering()
