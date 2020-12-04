#!/usr/bin/python3
import tkinter as tk
from tkinter import ttk
import time as tm
import RPi.GPIO as GPIO  # using Rpi.GPIO module
from time import sleep  # import function sleep for delay
from pvlib import solarposition
import pandas as pd
import enum
from datetime import datetime, timedelta, timezone
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import configparser
import logging
import sys
import copy

config = configparser.ConfigParser()
config.read("solar.properties")
CONFIG_SECTION = "solar_config"

AZI_DECREASE_WIN_DEG = float(
    config.get(CONFIG_SECTION, 'AZI_DECREASE_WIN_DEG'))  # factor * % power to calc actuator deceleration mode
MIN_AZIMUTH_DEGREES = int(config.get(CONFIG_SECTION, 'MIN_AZIMUTH_DEGREES'))
MIN_AZIMUTH_VOLTS = float(config.get(CONFIG_SECTION, 'MIN_AZIMUTH_VOLTS'))
MAX_AZIMUTH_DEGREES = int(config.get(CONFIG_SECTION, 'MAX_AZIMUTH_DEGREES'))
MAX_AZIMUTH_VOLTS = float(config.get(CONFIG_SECTION, 'MAX_AZIMUTH_VOLTS'))
AZI_MIN_DEGREES_ERR = float(config.get(CONFIG_SECTION, 'AZI_MIN_DEGREES_ERR'))

ELV_DECREASE_WIN_DEG = float(
    config.get(CONFIG_SECTION, 'ELV_DECREASE_WIN_DEG'))  # factor * % power to calc actuator deceleration mode
MIN_ELEVATION_DEGREES = int(config.get(CONFIG_SECTION, 'MIN_ELEVATION_DEGREES'))
MIN_ELEVATION_VOLTS = float(config.get(CONFIG_SECTION, 'MIN_ELEVATION_VOLTS'))
MAX_ELEVATION_DEGREES = int(config.get(CONFIG_SECTION, 'MAX_ELEVATION_DEGREES'))
MAX_ELEVATION_VOLTS = float(config.get(CONFIG_SECTION, 'MAX_ELEVATION_VOLTS'))
ELV_MIN_DEGREES_ERR = float(config.get(CONFIG_SECTION, 'ELV_MIN_DEGREES_ERR'))

POWER_ADJ_BY = float(config.get(CONFIG_SECTION, 'POWER_ADJ_BY'))  # + or - power adjust percentage for each loop_interval
LOOP_INTERVAL_MS = int(config.get(CONFIG_SECTION, 'LOOP_INTERVAL_MS'))  # loop interval in milliseconds to recheck the state of things

MIN_STARTING_POWER = int(config.get(CONFIG_SECTION, 'MIN_STARTING_POWER'))  # starting % actuator power to use to start increasing from
MAX_POWER = int(config.get(CONFIG_SECTION, 'MAX_POWER'))  # max % actuator power limit
PWM_HZ = int(config.get(CONFIG_SECTION, 'PWM_HZ'))

MAX_WIND_MPH_TO_AUTO_WINDY_MODE = int(config.get(CONFIG_SECTION, 'MAX_WIND_MPH_TO_AUTO_WINDY_MODE'))  # triggers elevation switch to stormy mode
AUTO_WINDY_MODE_LOCK_OUT_TIME_MINUTES = int(config.get(CONFIG_SECTION, 'AUTO_WINDY_MODE_LOCK_OUT_TIME_MINUTES'))

LAT = float(config.get(CONFIG_SECTION, 'LAT'))
LON = float(config.get(CONFIG_SECTION, 'LON'))

# initialize GPIO pins, I/O directions and PWM usage
AZI_PWM_PIN = 12  # set pin# used to for azimuth pwm power control
AZI_DIRECTION_PIN = 26  # set pin# used to control azimuth direction
AZI_INCREASE = GPIO.LOW  # value needed to move westward
ELV_PWM_PIN = 13  # set pin# used to for elevation pwm power control
ELV_DIRECTION_PIN = 24  # set pin# used to control elevation direction
ELV_INCREASE = GPIO.HIGH  # value needed to increase elevation
GPIO.setmode(GPIO.BCM)  # GPIO Broadcom pin-numbering scheme
GPIO.setwarnings(False)  # disable warning from GPIO
GPIO.setup(ELV_PWM_PIN, GPIO.OUT)  # set pin as output
GPIO.setup(AZI_PWM_PIN, GPIO.OUT)  # set pin as output
GPIO.setup(AZI_DIRECTION_PIN, GPIO.OUT)  # set pin as output
GPIO.setup(ELV_DIRECTION_PIN, GPIO.OUT)  # set pin as output

sleep(1)  # delay for 1 seconds

azimuth_power = GPIO.PWM(AZI_PWM_PIN, PWM_HZ)  # for azimuth power pin used and pwm frequency hz
azimuth_power.start(0)

elevation_power = GPIO.PWM(ELV_PWM_PIN, PWM_HZ)  # for elevation power pin used and pwm frequency hz
elevation_power.start(0)

# Create the I2C bus for use with ADC board
i2c = busio.I2C(board.SCL, board.SDA)
# Create the ADC object using the I2C bus
ads = ADS.ADS1115(i2c)
# Create single-ended inputs
chan0 = AnalogIn(ads, ADS.P0)  # elevation pot, connected so that larger voltage values == greater elevation
chan1 = AnalogIn(ads, ADS.P1)  # azimuth pot, connected so that larger voltage values == more westward
chan2 = AnalogIn(ads, ADS.P2)  # wind speed range from 0.4V (0 mph wind) up to 2.0V (for 72.5 mph wind speed)

solar_data = None

# enable logger object
logging.basicConfig(filename='solar.log', filemode='a', format='%(asctime)s | %(message)s', level=logging.INFO)


class Modes(enum.Enum):
    RUN = 0
    MAINTENANCE = 1
    CALIBRATION = 2
    AUTO_WINDY = 3


class ActuatorNames(enum.Enum):
    ELEVATION = 0
    AZIMUTH = 1


class PoweringMode(enum.Enum):
    IDLE = 0
    INCREASE = 1
    DECREASE = 2


class Maintenance(enum.Enum):
    IDLE = 0
    WASH_POSITION = 1
    STORM_POSITION = 2


class Actuator:
    def __init__(self, name, dir_pin, value_used_to_increase_dir, pwm_power_control, powering_mode, analog_channel):
        self.name = name
        self.dir_pin = dir_pin
        # value of GPIO.HIGH or LOW so that controls to more elevation or more azimuth (westward)
        self.value_used_to_increase_dir = value_used_to_increase_dir
        self.pwm_power_control = pwm_power_control
        self.powering_mode = powering_mode
        self.power = 0
        self.to_degrees = 0
        self.analog_channel = analog_channel

    def move_to(self, to_degrees):
        # print("actu move_to " + str(self.name))
        # self.pwm_power_control.start(0)
        self.to_degrees = to_degrees
        curr_pos = self.get_current_position()
        curr_pos_degrees = curr_pos["degrees"]
        err_degs = to_degrees - curr_pos_degrees

        logging.info("move_to start {} curr={:0.1f} to={:0.1f} err={:0.1f}".format(self.name, curr_pos_degrees, self.to_degrees, err_degs))
        print(str(self.name) + " move_to() curr=" + str(curr_pos_degrees) + " to_deg=" + str(self.to_degrees))
        if self.value_used_to_increase_dir == GPIO.HIGH:
            decrease_value_dir = GPIO.LOW
        else:
            decrease_value_dir = GPIO.HIGH
        # check which direction to move to
        if self.to_degrees - curr_pos_degrees > 0:
            # print("actu move_to set increase direction - " + str(self.value_used_to_increase_dir))
            GPIO.output(self.dir_pin, self.value_used_to_increase_dir)
        else:
            GPIO.output(self.dir_pin, decrease_value_dir)

        self.powering_mode = PoweringMode.INCREASE
        # self.pwm_power_control.ChangeDutyCycle(MIN_STARTING_POWER)

    def increment_power(self):
        self.power += POWER_ADJ_BY
        # clamp at 100
        if self.power > MAX_POWER:
            self.power = MAX_POWER
        self.pwm_power_control.start(self.power)
        print(str(self.name) + " pwr inc to=" + str(round(self.power, 1)))

    def decrement_power(self):
        self.power -= POWER_ADJ_BY
        # clamp at 0
        if self.power <= 0:
            self.power = 0
            self.powering_mode = PoweringMode.IDLE
            # self.pwm_power_control.stop()
            curr_pos = self.get_current_position()
            curr_pos_degrees = curr_pos["degrees"]
            err_degs = self.to_degrees - curr_pos_degrees
            logging.info("move_to end {} curr={:0.1f} to={:0.1f} err={:0.1f}".format(self.name, curr_pos_degrees, self.to_degrees, err_degs))

        self.pwm_power_control.start(self.power)
        print(str(self.name) + " pwr dec to=" + str(round(self.power, 1)))

    def update(self):
        # print("actu update " + str(self.name))
        curr_pos = self.get_current_position()
        curr_degs = curr_pos["degrees"]
        curr_degs_err = self.to_degrees - curr_degs
        if self.powering_mode == PoweringMode.INCREASE:
            # check if need to switch to decreasing power
            # power = self.power - MIN_STARTING_POWER  # reduces sensitivity to starting power
            if self.name == ActuatorNames.ELEVATION:
                decr_win_size_degs = ELV_DECREASE_WIN_DEG
            else:
                decr_win_size_degs = AZI_DECREASE_WIN_DEG

            # reduce window size by power percentage
            decr_win_size_degs = decr_win_size_degs * self.power / 100

            print(str(self.name) + " update() curr=" + str(round(curr_degs, 1)) + " to=" + str(
                round(self.to_degrees, 1)) + " err=" + str(round(curr_degs_err, 1)) + " win=" + str(
                round(decr_win_size_degs, 1)))
            if abs(curr_degs_err) <= decr_win_size_degs:
                self.powering_mode = PoweringMode.DECREASE
                print(str(self.name) + " update() set powering mode to decrease")
                # self.decrement_power()
            else:
                self.increment_power()

        if self.powering_mode == PoweringMode.DECREASE:
            self.decrement_power()
            print(str(self.name) + " update() curr=" + str(round(curr_degs, 1)) + " to=" + str(
                round(self.to_degrees, 1)) + " err=" + str(round(curr_degs_err, 1)))

    def stop(self):
        self.power = 0
        self.powering_mode = PoweringMode.IDLE
        self.pwm_power_control.start(0)

    def get_current_position(self):
        pos_data = {
            "voltage": round(self.analog_channel.voltage, 4),
            "raw": self.analog_channel.value,
            "degrees": convert_to_degrees(self.name, self.analog_channel.voltage)
        }
        return pos_data


class PositionController:
    def __init__(self):
        self.elevation_actuator = Actuator(ActuatorNames.ELEVATION, ELV_DIRECTION_PIN, ELV_INCREASE, elevation_power,
                                           PoweringMode.IDLE, chan0)
        self.azimuth_actuator = Actuator(ActuatorNames.AZIMUTH, AZI_DIRECTION_PIN, AZI_INCREASE, azimuth_power,
                                         PoweringMode.IDLE, chan1)
        self.mode = Modes.RUN
        self.sub_mode = PoweringMode.IDLE

        # initialize previous position with the current positions of the actuators
        curr_pos = self.elevation_actuator.get_current_position()
        curr_pos_degrees = curr_pos["degrees"]
        self.prev_elv_move_to_degrees = copy.deepcopy(curr_pos_degrees)
        curr_pos = self.azimuth_actuator.get_current_position()
        curr_pos_degrees = curr_pos["degrees"]
        self.prev_azi_move_to_degrees = copy.deepcopy(curr_pos_degrees)

        self.prev_mode = Modes.RUN
        self.last_windy_timestamp = get_system_time()

    def stop(self):
        self.elevation_actuator.stop()
        self.azimuth_actuator.stop()

    def check_windy_condition(self):
        wind_mph = get_wind_speed()
        if wind_mph >= MAX_WIND_MPH_TO_AUTO_WINDY_MODE:
            # update timestamp of windy condition
            self.last_windy_timestamp = get_system_time()
            if self.mode != Modes.AUTO_WINDY:
                # record last mode
                self.prev_mode = self.mode
                # switch mode to auto_windy mode
                self.set_mode(Modes.AUTO_WINDY)
        elif self.mode == Modes.AUTO_WINDY:
            # not above windy trigger, check if windy timeout expired
            curr_date_time = get_system_time()
            delta_time = curr_date_time - self.last_windy_timestamp
            minutes = (delta_time.seconds % 3600) // 60
            if minutes > AUTO_WINDY_MODE_LOCK_OUT_TIME_MINUTES:
                self.set_mode(self.prev_mode)

    def move(self, actuator, degrees):
        if actuator.name == ActuatorNames.ELEVATION:
            # print("posctlr ele move =" + str(degrees) + " prev=" + str(self.prev_elv_move_to_degrees))
            # ignore issuing move_to if previous position commanded is the same
            if self.prev_elv_move_to_degrees != degrees:
                # check if the move delta qualifies to try to actually attempt a move
                curr_pos = self.elevation_actuator.get_current_position()
                curr_pos_degrees = curr_pos["degrees"]
                if abs(degrees - curr_pos_degrees) > ELV_MIN_DEGREES_ERR:
                    # determine direction of move
                    if degrees > self.prev_elv_move_to_degrees:
                        # add the err window to requested degrees to overshoot some
                        new_pos = degrees + ELV_MIN_DEGREES_ERR
                    else:
                        # subtract err window size
                        new_pos = degrees - ELV_MIN_DEGREES_ERR

                    # clamp the degs to not exceed min / max
                    if new_pos < MIN_ELEVATION_DEGREES:
                        new_pos = MIN_ELEVATION_DEGREES

                    if new_pos > MAX_ELEVATION_DEGREES:
                        new_pos = MAX_ELEVATION_DEGREES

                    self.elevation_actuator.move_to(new_pos)
                    print("posCntlr move elv " + str(actuator.name) + " to_deg=" + str(new_pos))

                # update prev state
                self.prev_elv_move_to_degrees = copy.deepcopy(degrees)
        else:
            # print("posctlr azi move =" + str(degrees) + " prev=" + str(self.prev_azi_move_to_degrees))
            # ignore issuing move_to if previous position commanded is the same
            if self.prev_azi_move_to_degrees != degrees:
                # check if the move delta qualifies to try to actually attempt a move
                curr_pos = self.azimuth_actuator.get_current_position()
                curr_pos_degrees = curr_pos["degrees"]
                if abs(degrees - curr_pos_degrees) > AZI_MIN_DEGREES_ERR:
                    # determine direction of move
                    if degrees > self.prev_azi_move_to_degrees:
                        # add the err window to requested degrees to overshoot some
                        new_pos = degrees + AZI_MIN_DEGREES_ERR
                    else:
                        # subtract err window size
                        new_pos = degrees - AZI_MIN_DEGREES_ERR

                    # clamp the degs to not exceed min / max
                    if new_pos < MIN_AZIMUTH_DEGREES:
                        new_pos = MIN_AZIMUTH_DEGREES

                    if new_pos > MAX_AZIMUTH_DEGREES:
                        new_pos = MAX_AZIMUTH_DEGREES

                    self.prev_azi_move_to_degrees = degrees
                    self.azimuth_actuator.move_to(new_pos)
                    print("posCntlr move azi " + str(actuator.name) + " to_deg=" + str(new_pos))

                # update prev state
                self.prev_azi_move_to_degrees = copy.deepcopy(degrees)

    def set_mode(self, mode, sub_mode=None):
        if mode == Modes.RUN:
            self.mode = Modes.RUN
            # check current state of system and adjust
            print("set PositionController to run mode")
        elif mode == Modes.MAINTENANCE:
            self.mode = Modes.MAINTENANCE
            print("set PositionController to Maintenance mode")
            # check if sub_mode is being passed in to request 'wash' or 'stormy' postition setting
            if sub_mode is not None:
                self.sub_mode = sub_mode
                # start elevation actuator to requested position
                if sub_mode == Maintenance.WASH_POSITION:
                    self.move(self.elevation_actuator, MIN_ELEVATION_DEGREES)
                    self.move(self.azimuth_actuator, MIN_AZIMUTH_DEGREES)
                    print("set PositionController to Maintenance mode - Wash position")
                elif sub_mode == Maintenance.STORM_POSITION:
                    self.move(self.elevation_actuator, MAX_ELEVATION_DEGREES)
                    print("set PositionController to Maintenance mode - Stormy position")
                else:
                    self.elevation_actuator.stop()
                    print("set PositionController to Maintenance mode - Idle")
        elif mode == Modes.AUTO_WINDY:
            self.mode = Modes.AUTO_WINDY
            self.move(self.elevation_actuator, MAX_ELEVATION_DEGREES)
            print("set PositionController to AUTO_WINDY position")
        else:
            self.mode = Modes.CALIBRATION
            # default to mode Calibrate if not the other modes
            print("set PositionController to Calibrate mode")

    def update(self, sys_date_time):
        # first check on windy condition status
        self.check_windy_condition()

        if self.mode == Modes.RUN:
            # check current state of system and adjust
            # print("PositionController update in run mode")
            # 2020-04-10 13:05:00-00:00 format to get a row from solar_data
            curr_date_time = sys_date_time.strftime("%Y-%m-%d %H:%M:00-00:00")
            # print(curr_date_time)
            # based on date / time get the desired angles to be at
            solar_position_now = solar_data.loc[curr_date_time]
            # get actuator positions
            elv = self.elevation_actuator.get_current_position()
            azi = self.azimuth_actuator.get_current_position()
            update_ui_with_solar_data(solar_position_now, elv, azi)
            update_ui_for_wind(self.mode)
            # print("solar elv=" + str(round(solar_position_now.apparent_elevation, 1)) + " pos=" + str(elv['degrees']))
            # print("solar azi=" + str(round(solar_position_now.azimuth, 1)) + " pos=" + str(azi['degrees']))

            # check limits
            solar_elv_adjusted = round(solar_position_now.apparent_elevation, 1)
            if solar_elv_adjusted < MIN_ELEVATION_DEGREES:
                solar_elv_adjusted = MIN_ELEVATION_DEGREES
            elif solar_elv_adjusted > MAX_ELEVATION_DEGREES:
                # clamp ele at max if needed
                solar_elv_adjusted = MAX_ELEVATION_DEGREES

            solar_azi_adjusted = round(solar_position_now.azimuth, 1)
            if solar_azi_adjusted < MIN_AZIMUTH_DEGREES:
                # clamp azi at min if needed
                solar_azi_adjusted = MIN_AZIMUTH_DEGREES
            elif solar_azi_adjusted > MAX_AZIMUTH_DEGREES:
                # clamp azi at max if needed
                solar_azi_adjusted = MAX_AZIMUTH_DEGREES

            # extra check if sun is below the horizon
            if solar_position_now.apparent_elevation < 0:
                # ready azimuth facing east while sun below hozizon in east or west
                # so no movements are occurring in the middle of the night
                solar_azi_adjusted = MIN_AZIMUTH_DEGREES

            self.move(self.elevation_actuator, solar_elv_adjusted)
            self.elevation_actuator.update()

            self.move(self.azimuth_actuator, solar_azi_adjusted)
            self.azimuth_actuator.update()
        elif self.mode == Modes.MAINTENANCE:
            degrees = get_solar_degrees(sys_date_time)
            print("sun=", degrees)
            # check current state of system and adjust
            # print("PositionController update in Maintenance mode")
            # get actuator positions
            azi = self.azimuth_actuator.get_current_position()
            print("azi=", azi)
            ele = self.elevation_actuator.get_current_position()
            print("ele=", ele)
            self.elevation_actuator.update()
            self.azimuth_actuator.update()
        elif self.mode == Modes.AUTO_WINDY:
            self.elevation_actuator.update()
            # update ui to indicate it's in AUTO_WINDY mode
            update_ui_for_wind(self.mode)
        # else:
        # default to mode Calibrate if not the other modes
        # print("PositionController update in Calibrate mode")
        # self.elevation_actuator.move_to(MAX_ELEVATION_DEGREES)


class DigitalClock:
    def __init__(self, the_window):
        self.the_window = the_window
        self.clock_label = tk.Label(self.the_window, font='ariel 40')
        self.clock_label.grid(row=0, column=1, columnspan=3)
        sys_time = get_system_time()
        sys_local_time = sys_time.replace(tzinfo=timezone.utc).astimezone(tz=None)
        self.current_time = sys_local_time.strftime('%H:%M:%S')
        self.display_time()

    def display_time(self):
        sys_time = get_system_time()
        sys_local_time = sys_time.replace(tzinfo=timezone.utc).astimezone(tz=None)
        self.current_time = sys_local_time.strftime('%H:%M:%S')
        self.clock_label['text'] = self.current_time
        self.the_window.after(1000, self.display_time)


def on_closing():
    print("closing - stopping actuators")
    positionController.stop()
    form.destroy()


def on_tab_selected(event):
    positionController.stop()
    selected_tab = event.widget.select()
    tab_text = event.widget.tab(selected_tab, "text")
    print(tab_text + " selected")
    logging.info(tab_text + " Tab selected")
    uc_tab = tab_text.upper()
    if "RUN" in uc_tab:
        positionController.set_mode(Modes.RUN)
    elif "MAINTENANCE" in uc_tab:
        positionController.set_mode(Modes.MAINTENANCE, Maintenance.IDLE)
    else:
        positionController.set_mode(Modes.CALIBRATION)


def set_wash_position():
    print("Rotate to wash position")
    positionController.set_mode(Modes.MAINTENANCE, Maintenance.WASH_POSITION)


def set_stormy_position():
    print("Rotate to stormy position")
    positionController.set_mode(Modes.MAINTENANCE, Maintenance.STORM_POSITION)


def get_wind_speed():
    # The voltage will range from 0.4V (0 mph wind) up to 2.0V (for 72.5 mph wind speed)
    volts = round(chan2.voltage, 4)
    # y = mx + b,  72.5 mph = 45.3125 * 2.0 - 18.125
    wind_speed = round(45.3125 * volts - 18.125)
    if wind_speed < 0:
        wind_speed = 0
    return wind_speed


def update_ui_for_wind(mode):
    windSpeedMphLabelTabOne['text'] = get_wind_speed()
    if mode != Modes.AUTO_WINDY:
        # show normal condition wind speed
        windSpeedMphLabelTabOne['background'] = form['background']
    else:
        # indicate system in AUTO_WINDY mode
        windSpeedMphLabelTabOne['background'] = 'red'


def get_solar_degrees(sys_date_time):
    # 2020-04-10 13:05:00-00:00 format to get a row from solar_data
    curr_date_time = sys_date_time.strftime("%Y-%m-%d %H:%M:00-00:00")
    # print(curr_date_time)
    # based on date / time get the desired angles to be at
    solar_position_now = solar_data.loc[curr_date_time]
    deg_data = {
        "ele": round(solar_position_now.apparent_elevation, 1),
        "azi": round(solar_position_now.azimuth, 1)
    }
    return deg_data


def update_ui_calibration_tab():
    sun_pos = get_solar_degrees(get_system_time())
    sunAZITab3['text'] = sun_pos["azi"]
    sunELETab3['text'] = sun_pos["ele"]
    aziVoltageTab3['text'] = round(chan1.voltage, 4)
    aziDegreesTab3['text'] = convert_to_degrees(ActuatorNames.AZIMUTH, chan1.voltage)
    elevationVoltageTab3['text'] = round(chan0.voltage, 4)
    elevationDegreesTab3['text'] = convert_to_degrees(ActuatorNames.ELEVATION, chan0.voltage)


def get_todays_solar_data():
    today = get_system_time().date()
    tomorrow = today + timedelta(days=1)
    print("get_todays_solar_data() from:", today, "to:", tomorrow)

    # get date/times array in increments of 1 min for just today
    times = pd.date_range(today, tomorrow, closed='left', freq='1min', tz=timezone.utc)
    # print("times", times)
    # print times[0] information
    # get the solar data for these times
    solpos = solarposition.get_solarposition(times, LAT, LON)
    # keep only solar data where sun is above the horizon
    # solpos = solpos.loc[solpos['apparent_elevation'] > 0, :]
    # print("solpos", solpos)
    return solpos


def convert_to_degrees(name, ad_voltage):
    # print("ActuatorName: " + name + " ad_voltage: " + str(ad_voltage))
    if name == ActuatorNames.ELEVATION:
        # slope m = (y-y1)/(x-x1)
        m = (MAX_ELEVATION_DEGREES - MIN_ELEVATION_DEGREES) / (MAX_ELEVATION_VOLTS - MIN_ELEVATION_VOLTS)
        b = MIN_ELEVATION_DEGREES - (m * MIN_ELEVATION_VOLTS)
        degs = round(ad_voltage * m + b, 1)
        # print("elv v=", str(round(ad_voltage,2)), " degs=", str(round(degs,1)))
        return degs
    else:
        m = (MAX_AZIMUTH_DEGREES - MIN_AZIMUTH_DEGREES) / (MAX_AZIMUTH_VOLTS - MIN_AZIMUTH_VOLTS)
        b = MIN_AZIMUTH_DEGREES - (m * MIN_AZIMUTH_VOLTS)
        degs = round(ad_voltage * m + b, 1)
        # print("azi v=", str(round(ad_voltage,2)), " degs=", str(round(degs,1)))
        return degs


def update_ui_with_solar_data(sol_data_now, elv_sys_pos, azi_sys_pos):
    elv = sol_data_now.apparent_elevation
    azi = sol_data_now.azimuth
    # print(sol_data_now)
    azimuthAngleTabOne['text'] = str(round(azi, 1)) + "\N{DEGREE SIGN}"
    elevationAngleTabOne['text'] = str(round(elv, 1)) + "\N{DEGREE SIGN}"
    elv_err = elv_sys_pos['degrees'] - elv
    elevationErrorLabelTabOne['text'] = str(round(elv_err, 1)) + "\N{DEGREE SIGN}"
    azi_err = azi_sys_pos['degrees'] - azi
    azimuthErrorLabelTabOne['text'] = str(round(azi_err, 1)) + "\N{DEGREE SIGN}"


def get_system_time():
    sys_date_time = datetime.utcnow()
    if time_delta_adj is not None:
        sys_date_time = sys_date_time - time_delta_adj
        if seconds_multiplier > 1:
            running_seconds = (sys_date_time - app_start_time).total_seconds()
            sys_date_time = sys_date_time + timedelta(seconds=running_seconds * seconds_multiplier)

    # print(sys_date_time)
    return sys_date_time


def heartbeat():
    global last_check_of_today
    global solar_data
    try:
        sys_date_time = get_system_time()
        sys_date = sys_date_time.date()
        if sys_date != last_check_of_today:
            last_check_of_today = sys_date
            solar_data = get_todays_solar_data()

        positionController.update(sys_date_time)
        tab_parent.after(LOOP_INTERVAL_MS, heartbeat)
    except Exception as e:
        exception_type, exception_object, exception_traceback = sys.exc_info()
        filename = exception_traceback.tb_frame.f_code.co_filename
        line_number = exception_traceback.tb_lineno
        print("Exception type: ", exception_type)
        print("File name: ", filename)
        print("Line number: ", line_number)

        # stop the actuators and close the UI
        on_closing()


global time_delta_adj
global app_start_time
# set to old time
last_check_of_today = datetime.now() - timedelta(days=366)
print("in main")
time_delta_adj = None

# hard code while running in the IDE
# str_adj_dt = "12/21/20 05:00:00"
# print("adjusted datetime arg = " + str_adj_dt)
# adj_dt = datetime.strptime(str_adj_dt, '%m/%d/%y %H:%M:%S')
# time_delta_adj = datetime.now() - adj_dt
app_start_time = datetime.utcnow()
global seconds_multiplier
seconds_multiplier = 1
# seconds_multiplier = 720

if len(sys.argv) > 1:  # if adjusted datetime passed in use it
    str_adj_dt = sys.argv[1]
    print("adjusted datetime arg = " + str_adj_dt)
    adj_dt = datetime.strptime(str_adj_dt, '%m/%d/%y %H:%M:%S')
    time_delta_adj = datetime.now() - adj_dt
    app_start_time = datetime.utcnow() - time_delta_adj
    seconds_multiplier = 1
    if len(sys.argv) > 2:
        seconds_multiplier = int(sys.argv[2])

print(get_system_time())
positionController = PositionController()

form = tk.Tk()
form.title("Gilman Solar")
# window size x by y, position +x +y from top left corner
form.geometry("550x330+150+50")

tab_parent = ttk.Notebook(form)
tab_parent.after(1000, heartbeat)

tab1 = ttk.Frame(tab_parent)
tab2 = ttk.Frame(tab_parent)
tab3 = ttk.Frame(tab_parent)

tab_parent.bind("<<NotebookTabChanged>>", on_tab_selected)

tab_parent.add(tab1, text="Run mode")
tab_parent.add(tab2, text="Maintenance")
tab_parent.add(tab3, text="Calibration")

# === WIDGETS FOR 'Run mode' TAB
clockTabOne = DigitalClock(tab1)

windSpeedLabelTabOne = tk.Label(tab1, text="Wind mph:", font='ariel 14')
windSpeedMphLabelTabOne = tk.Label(tab1, text="0.0", font='ariel 18')

angleLabelTabOne = tk.Label(tab1, text="Solar Angle:", font='ariel 14')
errorLabelTabOne = tk.Label(tab1, text="Error:", font='ariel 14')

azimuthLabelTabOne = tk.Label(tab1, text="Azimuth:", font='ariel 14')
azimuthAngleTabOne = tk.Label(tab1, text="0.0", font='ariel 18')
azimuthErrorLabelTabOne = tk.Label(tab1, text="0.0", font='ariel 18')

elevationLabelTabOne = tk.Label(tab1, text="Elevation:", font='ariel 14')
elevationAngleTabOne = tk.Label(tab1, text="0.0", font='ariel 18')
elevationErrorLabelTabOne = tk.Label(tab1, text="0.0", font='ariel 18')

# === ADD WIDGETS TO GRID ON 'Run mode' TAB
windSpeedLabelTabOne.grid(row=1, column=3, padx=15, pady=15)
windSpeedMphLabelTabOne.grid(row=2, column=3, padx=15, pady=15)

angleLabelTabOne.grid(row=1, column=1, padx=15, pady=15)
errorLabelTabOne.grid(row=1, column=2, padx=15, pady=15)

azimuthLabelTabOne.grid(row=2, column=0, padx=15, pady=15)
azimuthAngleTabOne.grid(row=2, column=1, padx=15, pady=15)
azimuthErrorLabelTabOne.grid(row=2, column=2, padx=15, pady=15)

elevationLabelTabOne.grid(row=3, column=0, padx=15, pady=15)
elevationAngleTabOne.grid(row=3, column=1, padx=15, pady=15)
elevationErrorLabelTabOne.grid(row=3, column=2, padx=15, pady=15)

# === WIDGETS FOR 'Maintenance' TAB
buttonWash = tk.Button(tab2, text="Wash Position", command=set_wash_position)
buttonStormy = tk.Button(tab2, text="Stormy Position", command=set_stormy_position)
buttonWash.grid(row=0, column=0, padx=15, pady=15)
buttonStormy.grid(row=0, column=1, padx=15, pady=15)
# === WIDGETS FOR 'Calibration' TAB
sunAZILabelTab3 = tk.Label(tab3, text="Sun AZI:", font='ariel 10')
sunAZITab3 = tk.Label(tab3, text="0.0", font='ariel 10')
sunELELabelTab3 = tk.Label(tab3, text="Sun ELE:", font='ariel 10')
sunELETab3 = tk.Label(tab3, text="0.0", font='ariel 10')
aziVoltageLabelTab3 = tk.Label(tab3, text="AZI Voltage:", font='ariel 10')
aziDegreesLabelTab3 = tk.Label(tab3, text="AZI Degrees:", font='ariel 10')
elevationVoltageLabelTab3 = tk.Label(tab3, text="Elevation Voltage:", font='ariel 10')
elevationDegreesLabelTab3 = tk.Label(tab3, text="Elevation Degrees:", font='ariel 10')
aziVoltageTab3 = tk.Label(tab3, text="0.0", font='ariel 10')
aziDegreesTab3 = tk.Label(tab3, text="0.0", font='ariel 10')
elevationVoltageTab3 = tk.Label(tab3, text="0.0", font='ariel 10')
elevationDegreesTab3 = tk.Label(tab3, text="0.0", font='ariel 10')
buttonGetVoltageReadings = tk.Button(tab3, text="Actuator Readings", command=update_ui_calibration_tab)

# === ADD WIDGETS TO GRID ON 'Calibration' TAB
buttonGetVoltageReadings.grid(row=0, column=1, padx=5, pady=5)
sunAZILabelTab3.grid(row=1, column=0, padx=5, pady=5)
sunAZITab3.grid(row=1, column=1, padx=5, pady=5)
aziVoltageLabelTab3.grid(row=2, column=0, padx=5, pady=5)
aziVoltageTab3.grid(row=2, column=1, padx=5, pady=5)
aziDegreesLabelTab3.grid(row=3, column=0, padx=5, pady=5)
aziDegreesTab3.grid(row=3, column=1, padx=5, pady=5)
sunAZILabelTab3.grid(row=3, column=2, padx=5, pady=5)
sunAZITab3.grid(row=3, column=3, padx=5, pady=5)
elevationVoltageLabelTab3.grid(row=4, column=0, padx=5, pady=5)
elevationVoltageTab3.grid(row=4, column=1, padx=5, pady=5)
elevationDegreesLabelTab3.grid(row=5, column=0, padx=5, pady=5)
elevationDegreesTab3.grid(row=5, column=1, padx=5, pady=5)
sunELELabelTab3.grid(row=5, column=2, padx=5, pady=5)
sunELETab3.grid(row=5, column=3, padx=5, pady=5)

tab_parent.pack(expand=1, fill='both')

form.protocol("WM_DELETE_WINDOW", on_closing)

form.mainloop()
