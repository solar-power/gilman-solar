#!/usr/bin/python3
import tkinter as tk
from tkinter import ttk
import time as tm
import RPi.GPIO as GPIO  # using Rpi.GPIO module
from time import sleep  # import function sleep for delay
from pvlib import solarposition
import pandas as pd
import enum
from datetime import datetime
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

GPIO.setmode(GPIO.BCM)  # GPIO Broadcom pin-numbering scheme
GPIO.setwarnings(False)  # disable warning from GPIO

AZI_PWM_PIN = 12  # set pin# used to for azimuth pwm power control
AZI_DIRECTION_PIN = 26  # set pin# used to control azimuth direction
AZI_INCREASE = GPIO.LOW  # value needed to move westward
AZI_DECREASE_FACTOR = 0.02  # factor * % power to calc actuator deceleration mode
AZI_SLOPE = 59.801
AZI_OFFSET = 74.236
MIN_AZIMUTH_DEGREES = 102
MAX_AZIMUTH_DEGREES = 250
AZI_MIN_DEGREES_ERR = 5.0

ELV_PWM_PIN = 13  # set pin# used to for elevation pwm power control
ELV_DIRECTION_PIN = 24  # set pin# used to control elevation direction
ELV_INCREASE = GPIO.HIGH  # value needed to increase elevation
ELV_DECREASE_FACTOR = 0.03  # factor * % power to calc actuator deceleration mode
ELV_SLOPE = 28.398
ELV_OFFSET = 26.602
MIN_ELEVATION_DEGREES = 32
MAX_ELEVATION_DEGREES = 90
ELV_MIN_DEGREES_ERR = 2.5

POWER_ADJ_BY = 11.0  # + or - power adjust percentage for each loop_interval
LOOP_INTERVAL_MS = 100  # loop interval in milliseconds to recheck the state of things

MIN_STARTING_POWER = 10  # starting % actuator power to use to start increasing from
MAX_POWER = 80  # max % actuator power limit
PWM_HZ = 100

MAX_WIND_MPH_TO_AUTO_WINDY_MODE = 15  # triggers elevation switch to stormy mode
AUTO_WINDY_MODE_LOCK_OUT_TIME_MINUTES = 30

GPIO.setup(ELV_PWM_PIN, GPIO.OUT)  # set pin as output
GPIO.setup(AZI_PWM_PIN, GPIO.OUT)  # set pin as output
GPIO.setup(AZI_DIRECTION_PIN, GPIO.OUT)  # set pin as output
GPIO.setup(ELV_DIRECTION_PIN, GPIO.OUT)  # set pin as output

sleep(1)  # delay for 1 seconds

azimuth_power = GPIO.PWM(AZI_PWM_PIN, PWM_HZ)  # for azimuth power pin used and pwm frequency hz
azimuth_power.start(0)

elevation_power = GPIO.PWM(ELV_PWM_PIN, PWM_HZ)  # for elevation power pin used and pwm frequency hz
elevation_power.start(0)

tz = 'America/Los_Angeles'
lat, lon = 37.9810, -120.6419
last_check_of_today = '2020-01-01 00:00:00-07:00'
solar_data = None
# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)
# Create the ADC object using the I2C bus
ads = ADS.ADS1115(i2c)
# Create single-ended inputs
chan0 = AnalogIn(ads, ADS.P0)  # elevation pot, connected so that larger voltage values == greater elevation
chan1 = AnalogIn(ads, ADS.P1)  # azimuth pot, connected so that larger voltage values == more westward
chan2 = AnalogIn(ads, ADS.P2)  # wind speed range from 0.4V (0 mph wind) up to 2.0V (for 72.5 mph wind speed)


#try:
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
        self.to_degrees = to_degrees
        curr_pos = self.get_current_position()
        curr_pos_degrees = curr_pos["degrees"]
        # if self.name == ActuatorNames.ELEVATION:
        #     config_min_degs_err = ELV_MIN_DEGREES_ERR
        # else:
        #     config_min_degs_err = AZI_MIN_DEGREES_ERR

        print("actu move_to curr=" + str(curr_pos_degrees) + " to_deg=" + str(self.to_degrees))
        if self.value_used_to_increase_dir == GPIO.HIGH:
            decrease_value_dir = GPIO.LOW
        else:
            decrease_value_dir = GPIO.HIGH
        # check which direction to move to
        if self.to_degrees - curr_pos["degrees"] > 0:
            # print("actu move_to set increase direction - " + str(self.value_used_to_increase_dir))
            GPIO.output(self.dir_pin, self.value_used_to_increase_dir)
        else:
            GPIO.output(self.dir_pin, decrease_value_dir)

        self.powering_mode = PoweringMode.INCREASE
        self.pwm_power_control.start(MIN_STARTING_POWER)

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
        self.pwm_power_control.start(self.power)
        print(str(self.name) + " pwr dec to=" + str(round(self.power, 1)))

    def update(self):
        print("actu update " + str(self.name))
        curr_pos = self.get_current_position()
        curr_degs = curr_pos["degrees"]
        curr_degs_err = self.to_degrees - curr_degs
        if self.powering_mode == PoweringMode.INCREASE:
            # check if need to switch to decreasing power
            power = self.power - MIN_STARTING_POWER  # reduces sensitivity to starting power
            if self.name == ActuatorNames.ELEVATION:
                decr_win_size_degs = ELV_DECREASE_FACTOR * power
            else:
                decr_win_size_degs = AZI_DECREASE_FACTOR * power

            print("actu update err=" + str(round(curr_degs_err, 1)) + " win=" + str(round(decr_win_size_degs, 1)))
            if abs(curr_degs_err) <= decr_win_size_degs:
                self.powering_mode = PoweringMode.DECREASE
                print("actu set powering mode to decrease")
                self.decrement_power()
            else:
                self.increment_power()

        if self.powering_mode == PoweringMode.DECREASE:
            self.decrement_power()
            print(str(self.name) + "actu update err=" + str(round(curr_degs_err, 1)))

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
        self.prev_elv_move_to_degrees = curr_pos_degrees
        curr_pos = self.azimuth_actuator.get_current_position()
        curr_pos_degrees = curr_pos["degrees"]
        self.prev_azi_move_to_degrees = curr_pos_degrees

        self.prev_mode = Modes.RUN
        self.last_windy_timestamp = datetime.now()

    def stop(self):
        self.elevation_actuator.stop()
        self.azimuth_actuator.stop()

    def check_windy_condition(self):
        wind_mph = get_wind_speed()
        if wind_mph >= MAX_WIND_MPH_TO_AUTO_WINDY_MODE:
            # update timestamp of windy condition
            self.last_windy_timestamp = datetime.now()
            if self.mode != Modes.AUTO_WINDY:
                # record last mode
                self.prev_mode = self.mode
                # switch mode to auto_windy mode
                self.set_mode(Modes.AUTO_WINDY)
        elif self.mode == Modes.AUTO_WINDY:
            # not above windy trigger, check if windy timeout expired
            curr_date_time = datetime.now()
            delta_time = curr_date_time - self.last_windy_timestamp
            minutes = (delta_time.seconds % 3600) // 60
            if minutes > AUTO_WINDY_MODE_LOCK_OUT_TIME_MINUTES:
                self.set_mode(self.prev_mode)

    def move(self, actuator, degrees):
        if actuator.name == ActuatorNames.ELEVATION:
            print("move =" + str(degrees) + " prev=" + str(self.prev_elv_move_to_degrees))
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

                    self.elevation_actuator.move_to(new_pos)
                    print("posCntlr move elv " + str(actuator.name) + " to_deg=" + str(new_pos))

                #update prev state
                self.prev_elv_move_to_degrees = degrees
        else:
            print("move =" + str(degrees) + " prev=" + str(self.prev_azi_move_to_degrees))
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

                    self.prev_azi_move_to_degrees = degrees
                    self.azimuth_actuator.move_to(new_pos)
                    print("posCntlr move azi " + str(actuator.name) + " to_deg=" + str(new_pos))

                #update prev state
                self.prev_azi_move_to_degrees = degrees

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

    def update(self):
        # print("positioncontroller update called")
        # first check on windy condition status
        self.check_windy_condition()

        if self.mode == Modes.RUN:
            # check current state of system and adjust
            print("PositionController update in run mode")
            now = datetime.now()  # current date and time
            # 2020-04-10 13:05:00-07:00 format to get a row from solar_data
            curr_date_time = now.strftime("%Y-%m-%d %H:%M:00-08:00")
            # print(curr_date_time)
            # based on date / time get the desired angles to be at
            # for i, j in solar_data.iterrows():
            #    print(i, j)
            #    print()
            solar_position_now = solar_data.loc[curr_date_time]
            # get actuator positions
            elv = self.elevation_actuator.get_current_position()
            azi = self.azimuth_actuator.get_current_position()
            update_ui_with_solar_data(solar_position_now, elv, azi)
            update_ui_for_wind(self.mode)
            print("solar elv=" + str(round(solar_position_now.apparent_elevation, 1)) + " pos=" + str(elv['degrees']))
            print("solar azi=" + str(round(solar_position_now.azimuth, 1)) + " pos=" + str(azi['degrees']))

            # check limits
            solar_elv_adjusted = round(solar_position_now.apparent_elevation, 1)
            if solar_elv_adjusted < MIN_ELEVATION_DEGREES:
                solar_elv_adjusted = MIN_ELEVATION_DEGREES

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
            # check current state of system and adjust
            print("PositionController update in Maintenance mode")
            # get elevation actuator position
            elv = self.elevation_actuator.get_current_position()
            print(elv)
            self.elevation_actuator.update()
            self.azimuth_actuator.update()
        elif self.mode == Modes.AUTO_WINDY:
            self.elevation_actuator.update()
            # update ui to indicate it's in AUTO_WINDY mode
            update_ui_for_wind(self.mode)
        else:
            # default to mode Calibrate if not the other modes
            print("PositionController update in Calibrate mode")
            # self.elevation_actuator.move_to(MAX_ELEVATION_DEGREES)


class DigitalClock:
    def __init__(self, the_window):
        self.the_window = the_window
        self.clock_label = tk.Label(self.the_window, font='ariel 40')
        self.clock_label.grid(row=0, column=1, columnspan=3)
        self.current_time = tm.strftime('%H:%M:%S')
        self.display_time()

    def display_time(self):
        self.current_time = tm.strftime('%H:%M:%S')
        self.clock_label['text'] = self.current_time
        self.the_window.after(1000, self.display_time)


def on_closing():
    print("closing - stopping actuators")
    positionController.stop()
    form.destroy()


def on_tab_selected(event):
    selected_tab = event.widget.select()
    tab_text = event.widget.tab(selected_tab, "text")
    print(tab_text + " selected")
    uc_tab = tab_text.upper()
    if "RUN" in uc_tab:
        positionController.set_mode(Modes.RUN)
    elif "MAINTENANCE" in uc_tab:
        positionController.set_mode(Modes.MAINTENANCE, Maintenance.IDLE)
    else:
        positionController.set_mode(Modes.CALIBRATION)


def set_wash_position():
    print ("Rotate to wash position")
    positionController.set_mode(Modes.MAINTENANCE, Maintenance.WASH_POSITION)


def set_stormy_position():
    print ("Rotate to stormy position")
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


def get_todays_solar_data():
    print("get_todays_solar_data")
    today = pd.to_datetime('today').date()
    print("today = " , today)
    tomorrow = pd.to_datetime('today').date() + pd.to_timedelta(1, unit='D')
    print("tomorrow = " , tomorrow)
    # get date/times array in increments of 1 min for just today
    times = pd.date_range(today, tomorrow, closed='left', freq='1min', tz=tz)
    print("times", times)
    # print times[0] information
    # get the solar data for these times
    solpos = solarposition.get_solarposition(times, lat, lon)
    # keep only solar data where sun is above the horizon
    # solpos = solpos.loc[solpos['apparent_elevation'] > 0, :]
    #print("solpos", solpos)
    return solpos


def convert_to_degrees(name, ad_voltage):
    print("ActuatorName: " + name + " ad_voltage: ", ad_voltage)
    if name == ActuatorNames.ELEVATION:
        # substitute with calibration data when ready
        # rough calc for now
        # 4.1v == 32767 max reading (15 bit), pot uses a 3.3v ref voltage, max raw == 26373 == MAX_ELEVATION_DEGREES
        # raw 0 == MIN_ELEVATION_DEGREES
        # y = mx+b
        m = (MAX_ELEVATION_DEGREES - MIN_ELEVATION_DEGREES) / 26373
        # 85deg = m*26373 + 25deg -> 60 / 26373 = m -> 0.002085466196489
        #return round(ad_voltage * m + MIN_ELEVATION_DEGREES, 1)
        return round(ad_voltage * ELV_SLOPE + ELV_OFFSET, 1)
    else:
        return round(ad_voltage * AZI_SLOPE + AZI_OFFSET, 1)


def get_current_solar_data_for_timestamp(ts):
    print("get_current_solar_data_for_timestamp = " + ts)


def update_ui_with_solar_data(sol_data_now, elv_sys_pos, azi_sys_pos):
    elv = sol_data_now.apparent_elevation
    azi = sol_data_now.azimuth
    azimuthAngleTabOne['text'] = str(round(azi, 1)) + "\N{DEGREE SIGN}"
    elevationAngleTabOne['text'] = str(round(elv, 1)) + "\N{DEGREE SIGN}"
    elv_err = elv_sys_pos['degrees'] - elv
    elevationErrorLabelTabOne['text'] = str(round(elv_err, 1)) + "\N{DEGREE SIGN}"
    azi_err = azi_sys_pos['degrees'] - azi
    azimuthErrorLabelTabOne['text'] = str(round(azi_err, 1)) + "\N{DEGREE SIGN}"


def heartbeat():
    global last_check_of_today
    global solar_data
    today = pd.to_datetime('today').date()
    if today != last_check_of_today:
        last_check_of_today = today
        solar_data = get_todays_solar_data()
    positionController.update()
    tab_parent.after(LOOP_INTERVAL_MS, heartbeat)


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
buttonWash.grid(row=0, column=0, padx=15, pady=15)

buttonStormy = tk.Button(tab2, text="Stormy Position", command=set_stormy_position)
buttonStormy.grid(row=0, column=1, padx=15, pady=15)

tab_parent.pack(expand=1, fill='both')

form.protocol("WM_DELETE_WINDOW", on_closing)

form.mainloop()

#except:
#print("exception occurred")
#elevation_power.start(0)
#azimuth_power.start(0)
