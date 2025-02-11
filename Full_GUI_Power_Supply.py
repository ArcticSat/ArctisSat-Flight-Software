# Imports for overall GUI
import pygame
import serial
import gif_pygame
import time
import serial.tools.list_ports

# Imports for orbit sim and orbit animation
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import numpy as np
import math
import matplotlib.pyplot as plt
import cartopy.crs as ccrs

# Imports for Power Supply
from pyHM310T import PowerSupply

# Imports to make GUI faster
import threading

#########################################################################################################
#####################                         GUI SETUP                       ###########################
#########################################################################################################
# Initialize Pygame
pygame.init()

indicator_corner_anchor_x = 50
indicator_corner_anchor_y = 500

voltage_anchor_x = 200
current_anchor_x = 200
voltage_anchor_y = 500
current_anchor_y = 550
GUI_loop_rate_anchor_x = 950
GUI_loop_rate_anchor_y = 60

# Set up the display
width, height = 1200, 800
pygame.mixer.init()
sound1 = pygame.mixer.Sound('catsong.mp3')  # Load a sound.

window = pygame.display.set_mode((width, height))
pygame.display.set_caption("HAXSat")

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GREY = (160, 160, 160)
OFF_RED = (240, 128, 128)
ORANGE = (252, 186, 3)
PURPLE = (100, 0, 125)

armCount = 0

# Set up the font
font = pygame.font.Font(None, 20)

def draw_labels():
    power_label = font.render("Power Status", True, BLACK)
    adcs_label = font.render("ADCS Status", True, BLACK)
    comms_label = font.render("CDH Status", True, BLACK)
    window.blit(power_label, (indicator_corner_anchor_x + 25, indicator_corner_anchor_y))
    window.blit(adcs_label, (indicator_corner_anchor_x + 25, indicator_corner_anchor_y + 25))
    window.blit(comms_label, (indicator_corner_anchor_x + 25, indicator_corner_anchor_y + 50))

actuatorColor = RED

def armActuators():
    global actuatorColor
    global armCount
    print("POG")
    actuatorColor = GREEN
    armCount = 300


class Button:
    def __init__(self, x, y, width, height, color, message, text, action=None, group=None):
        self.rect = pygame.Rect(x, y, width, height)
        self.color = color
        self.message = message
        self.text = text
        self.action = action
        self.group = group

    def draw(self, surface):
        pygame.draw.rect(surface, self.color, self.rect)
        text_surface = font.render(self.text, True, BLACK)
        text_rect = text_surface.get_rect(center=self.rect.center)
        surface.blit(text_surface, text_rect)

    def update(self, newColor):
        self.color = newColor

    def is_clicked(self, pos):
        return self.rect.collidepoint(pos)

    def on_click(self):
        if self.action:
            self.action()
        else:
            if (self.color != RED) and ser:
                ser.write(self.message)


# Define buttons with unique serial messages and text

buttons = [
    Button(50, 20, 100, 20, RED, "bruh", "ARM", armActuators, "ARMING"),
    Button(50, 50, 100, 50, RED, bytearray(b'\x01\x08\x00'), "Left Out", None, "ACTUATORS"),
    Button(50, 110, 100, 50, GREEN, bytearray(b'\x01\x0A\x00'), "Left In", None, "ACTUATORS"),
    Button(350, 50, 100, 50, RED, bytearray(b'\x01\x0C\x00'), "Both Out", None, "ACTUATORS"),
    Button(200, 50, 100, 50, RED, bytearray(b'\x01\x09\x00'), "Right Out", None, "ACTUATORS"),
    Button(200, 110, 100, 50, GREEN, bytearray(b'\x01\x0B\x00'), "Right In", None, "ACTUATORS"),
    Button(350, 110, 100, 50, GREEN, bytearray(b'\x01\x0D\x00'), "Both In", None, "ACTUATORS"),
    Button(50, 170, 100, 50, PURPLE, bytearray(b'\x00\x00'), "Ping Power", None, "POWER"),
    Button(200, 170, 100, 50, PURPLE, bytearray(b'\x00\x01\x01'), "Get V and C", None, "POWER"),
    Button(350, 170, 100, 50, GREY, bytearray(b'\x01\x01\x01'), "Sync"),
    Button(500, 50, 100, 50, GREY, bytearray(b'\x01\x02\x0D'), "Gyro"),
    Button(500, 110, 100, 50, GREY, bytearray(b'\x01\x03\x0A'), "Mag"),
    Button(500, 170, 100, 50, GREY, bytearray(b'\x01\x07\x03'), "Temp"),
    Button(650, 50, 100, 50, GREY, bytearray(b'\x01\x04\x15'), "Lat/Long"),
    Button(650, 110, 100, 50, GREY, bytearray(b'\x01\x05\x11'), "Time"),
    Button(650, 170, 100, 50, GREY, bytearray(b'\x01\x06\x05'), "Num Sats"),
    Button(800, 110, 100, 50, GREY, bytearray(b'\x01\x0E\x00'), "Reset GNSS"),
    Button(800, 170, 100, 50, GREY, bytearray(b'\x01\x10\x06'), "Get data rate"),
    Button(950, 100, 50, 50, GREEN, bytearray(b'\x00\x04\x31\x01'), "ON"),
    Button(1000, 100, 50, 50, RED, bytearray(b'\x00\x04\x31\x00'), "OFF"),
    Button(1050, 100, 50, 50, GREY, bytearray(b'\x00\x04\x31\x03'), "DATA"),
    Button(950, 160, 50, 50, GREEN, bytearray(b'\x00\x04\x30\x01'), "ON"),
    Button(1000, 160, 50, 50, RED, bytearray(b'\x00\x04\x30\x00'), "OFF"),
    Button(1050, 160, 50, 50, GREY, bytearray(b'\x00\x04\x30\x02'), "DATA"),
    Button(950, 220, 50, 50, GREEN, bytearray(b'\x00\x04\x03\x01'), "ON"),
    Button(1000, 220, 50, 50, RED, bytearray(b'\x00\x04\x03\x00'), "OFF"),
    Button(1050, 220, 50, 50, GREY, bytearray(b'\x00\x04\x31\x02'), "DATA"),
    Button(950, 280, 50, 50, GREEN, bytearray(b'\x00\x04\x03\x01'), "ON"),
    Button(1000, 280, 50, 50, RED, bytearray(b'\x00\x04\x03\x00'), "OFF"),
    Button(1050, 280, 50, 50, GREY, bytearray(b'\x00\x04\x31\x02'), "DATA"),
]

# GUI elements for power supply voltage and current
ps_voltage_label = font.render("Power Supply Voltage (V):", True, BLACK)
ps_current_label = font.render("Power Supply Current (A):", True, BLACK)
ps_voltage_display = font.render("0.0", True, BLACK)
ps_current_display = font.render("0.0", True, BLACK)
ps_power_label = font.render("Power Supply Power (W):", True, BLACK)
ps_power_display = font.render("0.0", True, BLACK)
ps_output_status_label = font.render("Power Supply Status:", True, BLACK)
ps_output_status_display = font.render("Disabled", True, BLACK)
ps_output_button_display = "Enable Output"

# Positioning the elements
ps_output_status_anchor_x = 50
ps_output_status_anchor_y = 600
ps_voltage_anchor_x = 50
ps_voltage_anchor_y = 625
ps_current_anchor_x = 50
ps_current_anchor_y = 650
ps_power_anchor_x = 50
ps_power_anchor_y = 675

# Input fields for setting voltage and current, and orbit parameters
ps_voltage_input = pygame.Rect(50, 700, 100, 25)
ps_current_input = pygame.Rect(50, 725, 100, 25)
orbital_radius_input = pygame.Rect(400, 600, 100, 25)
eccentricity_input = pygame.Rect(400, 625, 100, 25)
inclination_input = pygame.Rect(400, 650, 100, 25)
raan_input = pygame.Rect(400, 675, 100, 25)
arg_periapsis_input = pygame.Rect(400, 700, 100, 25)
orbit_sim_speed_input = pygame.Rect(400, 725, 100, 25)
ground_station_latitude_input = pygame.Rect(400, 550, 100, 25)
ground_station_longitude_input = pygame.Rect(400, 575, 100, 25)
ps_voltage_input_text = ''
ps_current_input_text = ''
orbital_radius_input_text = ''
eccentricity_input_text = ''
inclination_input_text = ''
raan_input_text = ''
arg_periapsis_input_text = ''
orbit_sim_speed_input_text = ''
ground_station_latitude_input_text = ''
ground_station_longitude_input_text = ''

# Buttons for setting voltage and current, and orbital parameters
ps_set_voltage_button = Button(150, 700, 100, 25, GREY, None, "Set Voltage", lambda: set_ps_voltage(ps_voltage_input_text))
ps_set_current_button = Button(150, 725, 100, 25, GREY, None, "Set Current", lambda: set_ps_current(ps_current_input_text))
ps_set_output_button = Button(70, 765, 160, 25, GREY, None, ps_output_button_display, lambda: toggle_ps_output())
set_orbital_radius_button = Button(500, 600, 125, 25, GREY, None, "Set Radius", lambda: set_orbital_radius(orbital_radius_input_text))
set_eccentricity_button = Button(500, 625, 125, 25, GREY, None, "Set Eccentricity", lambda: set_eccentricity(eccentricity_input_text))
set_inclination_button = Button(500, 650, 125, 25, GREY, None, "Set Inclination", lambda: set_inclination(inclination_input_text))
set_raan_button = Button(500, 675, 125, 25, GREY, None, "Set RAAN", lambda: set_raan(raan_input_text))
set_arg_periapsis_button = Button(500, 700, 125, 25, GREY, None, "Set Arg. Periapsis", lambda: set_arg_periapsis(arg_periapsis_input_text))
set_orbit_sim_speed_button = Button(500, 725, 125, 25, GREY, None, "Set Sim. Speed", lambda: set_orbit_sim_speed(orbit_sim_speed_input_text))
set_ground_station_latitude_button = Button(500, 550, 125, 25, GREY, None, "Set Gnd. St. Lat", lambda: set_ground_station_latitude(ground_station_latitude_input_text))
set_ground_station_longitude_button = Button(500, 575, 125, 25, GREY, None, "Set Gnd. St. Lon", lambda: set_ground_station_longitude(ground_station_longitude_input_text))
# Button to allow orbit simulation to control power supply
orbit_sim_control_button = Button(285, 765, 340, 25, GREY, None, "Allow Orbit Sim. to Control Power Supply", lambda: toggle_orbit_sim_control())

# Connect buttons for the serial COM ports
connect_serial_button = Button(width-575, 5, 125, 25, GREY, None, "Connect Radio", lambda: toggle_serial_connection())
connect_ps_button = Button(width-250, 5, 100, 25, GREY, None, "Connect PS", lambda: toggle_ps_connection())

def toggle_serial_connection():
    global ser
    if ser:
        ser.close()
        ser = None
        connect_serial_button.text = "Connect Radio"
        connect_serial_button.color = GREY
    else:
        initialize_serial_connection()
        if ser:
            connect_serial_button.text = "Disconnect Radio"
            connect_serial_button.color = GREEN

def toggle_ps_connection():
    global ps
    if ps:
        ps.close()
        ps = None
        connect_ps_button.text = "Connect PS"
        connect_ps_button.color = GREY
    else:
        initialize_ps_connection()
        if ps:
            connect_ps_button.text = "Disconnect PS"
            connect_ps_button.color = GREEN
def list_com_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

# Dropdown for COM port selection
com_ports = list_com_ports()
selected_com_port = None
dropdown_open = False

def draw_dropdown_items(items, x, y, width, height, selected_item):
    for i, item in enumerate(items):
        item_rect = pygame.Rect(x, y + (i + 1) * height, width, height)
        pygame.draw.rect(window, WHITE, item_rect)
        pygame.draw.rect(window, BLACK, item_rect, 2)
        item_surface = font.render(item, True, BLACK)
        window.blit(item_surface, (item_rect.x + 5, item_rect.y + 5))

def draw_com_port_dropdown():
    global selected_com_port, dropdown_open
    dropdown_rect = pygame.Rect(width-450, 5, 150, 25)
    pygame.draw.rect(window, WHITE, dropdown_rect)
    pygame.draw.rect(window, BLACK, dropdown_rect, 2)
    if selected_com_port:
        text_surface = font.render(selected_com_port, True, BLACK)
    else:
        text_surface = font.render("Select COM Port", True, BLACK)
    window.blit(text_surface, (dropdown_rect.x + 5, dropdown_rect.y + 5))

    if dropdown_open:
        draw_dropdown_items(com_ports, width-450, 5, 150, 25, selected_com_port)

def handle_com_port_selection(event):
    global selected_com_port, dropdown_open
    dropdown_rect = pygame.Rect(width-450, 5, 150, 25)
    if event.type == pygame.MOUSEBUTTONDOWN:
        if dropdown_rect.collidepoint(event.pos):
            dropdown_open = not dropdown_open
        elif dropdown_open:
            for i, port in enumerate(com_ports):
                item_rect = pygame.Rect(width-450, 5 + (i + 1) * 25, 150, 25)
                if item_rect.collidepoint(event.pos):
                    selected_com_port = port
                    dropdown_open = False
                    break

# Add a dropdown for Power Supply COM port selection
ps_com_ports = list_com_ports()
selected_ps_com_port = None
ps_dropdown_open = False
def draw_ps_com_port_dropdown():
    global selected_ps_com_port, ps_dropdown_open
    dropdown_rect = pygame.Rect(width-150, 5, 150, 25)
    pygame.draw.rect(window, WHITE, dropdown_rect)
    pygame.draw.rect(window, BLACK, dropdown_rect, 2)
    if selected_ps_com_port:
        text_surface = font.render(selected_ps_com_port, True, BLACK)
    else:
        text_surface = font.render("Select PS COM Port", True, BLACK)
    window.blit(text_surface, (dropdown_rect.x + 5, dropdown_rect.y + 5))

    if ps_dropdown_open:
        draw_dropdown_items(ps_com_ports, width-150, 5, 150, 25, selected_ps_com_port)

def handle_ps_com_port_selection(event):
    global selected_ps_com_port, ps_dropdown_open
    dropdown_rect = pygame.Rect(width-150, 5, 150, 25)
    if event.type == pygame.MOUSEBUTTONDOWN:
        if dropdown_rect.collidepoint(event.pos):
            ps_dropdown_open = not ps_dropdown_open
        elif ps_dropdown_open:
            for i, port in enumerate(ps_com_ports):
                item_rect = pygame.Rect(width-150, 5 + (i + 1) * 25, 150, 25)
                if item_rect.collidepoint(event.pos):
                    selected_ps_com_port = port
                    ps_dropdown_open = False
                    break

ser = None
def initialize_serial_connection():
    global ser
    if selected_com_port:
        ser = serial.Serial(selected_com_port, 115200)
        ser.timeout = 0.5
        print("Serial connected...")

ps = None
def initialize_ps_connection():
    global ps
    if selected_ps_com_port:
        try:
            ps = PowerSupply(selected_ps_com_port)
            print("Connected to HM310T")
            initialize_ps_settings()
        except Exception as e:
            print(f"Failed to connect to {selected_ps_com_port}: {e}")

def check_serial_connections():
    global ser, ps
    if ser:
        try:
            ser.in_waiting
        except (OSError, serial.SerialException):
            ser.close()
            ser = None
            connect_serial_button.text = "Connect Radio"
            connect_serial_button.color = GREY
    if ps:
        try:
            ps.get_voltage()
            ser.in_waiting
        except (OSError, serial.SerialException):
            ps.close()
            ps = None
            connect_ps_button.text = "Connect PS"
            connect_ps_button.color = GREY

def update_com_ports():
    global com_ports, ps_com_ports
    com_ports = list_com_ports()
    ps_com_ports = list_com_ports()

def check_serial_connections_thread():
    global running
    while running:
        update_com_ports()
        check_serial_connections()

def update_orbit_and_PS_GUI_elements_in_loop():
    # Draw power supply related variables
    window.blit(ps_output_status_label, (ps_output_status_anchor_x, ps_output_status_anchor_y))
    window.blit(ps_output_status_display, (ps_output_status_anchor_x + 175, ps_output_status_anchor_y))
    window.blit(ps_voltage_label, (ps_voltage_anchor_x, ps_voltage_anchor_y))
    window.blit(ps_voltage_display, (ps_voltage_anchor_x + 175, ps_voltage_anchor_y))
    window.blit(ps_current_label, (ps_current_anchor_x, ps_current_anchor_y))
    window.blit(ps_current_display, (ps_current_anchor_x + 175, ps_current_anchor_y))
    window.blit(ps_power_label, (ps_power_anchor_x, ps_power_anchor_y))
    window.blit(ps_power_display, (ps_power_anchor_x + 175, ps_power_anchor_y))
    # Draw input fields and buttons for setting voltage and current
    pygame.draw.rect(window, BLACK, ps_voltage_input, 2)  # Border for voltage input
    pygame.draw.rect(window, WHITE, ps_voltage_input.inflate(-4, -4))  # Inner rectangle for voltage input
    pygame.draw.rect(window, BLACK, ps_current_input, 2)  # Border for current input
    pygame.draw.rect(window, WHITE, ps_current_input.inflate(-4, -4))  # Inner rectangle for current input
    voltage_input_surface = font.render(ps_voltage_input_text, True, BLACK)
    current_input_surface = font.render(ps_current_input_text, True, BLACK)
    window.blit(voltage_input_surface, (ps_voltage_input.x + 5, ps_voltage_input.y + 5))
    window.blit(current_input_surface, (ps_current_input.x + 5, ps_current_input.y + 5))
    ps_set_voltage_button.draw(window)
    ps_set_current_button.draw(window)
    ps_set_output_button.draw(window)

    pygame.draw.rect(window, BLACK, orbital_radius_input, 2)  # Border for orbital radius input
    pygame.draw.rect(window, WHITE, orbital_radius_input.inflate(-4, -4))  # Inner rectangle for orbital radius input
    pygame.draw.rect(window, BLACK, eccentricity_input, 2)  # Border for eccentricity input
    pygame.draw.rect(window, WHITE, eccentricity_input.inflate(-4, -4))  # Inner rectangle for eccentricity input
    pygame.draw.rect(window, BLACK, inclination_input, 2)  # Border for inclination input
    pygame.draw.rect(window, WHITE, inclination_input.inflate(-4, -4))  # Inner rectangle for inclination input
    pygame.draw.rect(window, BLACK, raan_input, 2)  # Border for RAAN input
    pygame.draw.rect(window, WHITE, raan_input.inflate(-4, -4))  # Inner rectangle for RAAN input
    pygame.draw.rect(window, BLACK, arg_periapsis_input, 2)  # Border for Arg. Periapsis input
    pygame.draw.rect(window, WHITE, arg_periapsis_input.inflate(-4, -4))  # Inner rectangle for Arg. Periapsis input
    pygame.draw.rect(window, BLACK, orbit_sim_speed_input, 2)  # Border for Orbit Sim. Speed input
    pygame.draw.rect(window, WHITE, orbit_sim_speed_input.inflate(-4, -4))  # Inner rectangle for Orbit Sim. Speed input
    pygame.draw.rect(window, BLACK, ground_station_latitude_input, 2)  # Border for ground station latitude input
    pygame.draw.rect(window, WHITE,
                     ground_station_latitude_input.inflate(-4, -4))  # Inner rectangle for ground station latitude input
    pygame.draw.rect(window, BLACK, ground_station_longitude_input, 2)  # Border for ground station longitude input
    pygame.draw.rect(window, WHITE, ground_station_longitude_input.inflate(-4,
                                                                           -4))  # Inner rectangle for ground station longitude input
    # Draw input fields and buttons for setting orbital parameters
    orbital_radius_input_surface = font.render(orbital_radius_input_text, True, BLACK)
    eccentricity_input_surface = font.render(eccentricity_input_text, True, BLACK)
    inclination_input_surface = font.render(inclination_input_text, True, BLACK)
    raan_input_surface = font.render(raan_input_text, True, BLACK)
    arg_periapsis_input_surface = font.render(arg_periapsis_input_text, True, BLACK)
    orbit_sim_speed_input_surface = font.render(orbit_sim_speed_input_text, True, BLACK)
    ground_station_latitude_input_surface = font.render(ground_station_latitude_input_text, True, BLACK)
    ground_station_longitude_input_surface = font.render(ground_station_longitude_input_text, True, BLACK)
    # Draw orbital parameters variables
    window.blit(orbital_radius_input_surface, (orbital_radius_input.x + 5, orbital_radius_input.y + 5))
    window.blit(eccentricity_input_surface, (eccentricity_input.x + 5, eccentricity_input.y + 5))
    window.blit(inclination_input_surface, (inclination_input.x + 5, inclination_input.y + 5))
    window.blit(raan_input_surface, (raan_input.x + 5, raan_input.y + 5))
    window.blit(arg_periapsis_input_surface, (arg_periapsis_input.x + 5, arg_periapsis_input.y + 5))
    window.blit(orbit_sim_speed_input_surface, (orbit_sim_speed_input.x + 5, orbit_sim_speed_input.y + 5))
    window.blit(ground_station_latitude_input_surface,
                (ground_station_latitude_input.x + 5, ground_station_latitude_input.y + 5))
    window.blit(ground_station_longitude_input_surface,
                (ground_station_longitude_input.x + 5, ground_station_longitude_input.y + 5))

    set_orbital_radius_button.draw(window)
    set_eccentricity_button.draw(window)
    set_inclination_button.draw(window)
    set_raan_button.draw(window)
    set_arg_periapsis_button.draw(window)
    set_orbit_sim_speed_button.draw(window)
    set_ground_station_latitude_button.draw(window)
    set_ground_station_longitude_button.draw(window)
    orbit_sim_control_button.draw(window)



#########################################################################################################
###################              ORBIT SIMULATION AND ANIMATION              ############################
#########################################################################################################
# Constants
EARTH_RADIUS = 6371  # in km
EARTH_MU = 398600  # in km^3/s^2
SUN_RADIUS = 695700  # in km
AU = 149597870.7  # Astronomical Unit in km
SECONDS_IN_MINUTE = 60
MINUTES_IN_DAY = 1440
EARTH_ROTATION_RATE = 360 / (24 * 60)  # degrees per minute
allow_orbit_sim_control = 0 # start with not allowing orbit simulation to control power supply
ps_previous_state = 1 # start with assuming PS is off

def calculate_position(semi_major_axis, eccentricity, inclination, raan, arg_periapsis, mean_anomaly):
    inclination = np.radians(inclination)
    raan = np.radians(raan)
    arg_periapsis = np.radians(arg_periapsis)
    mean_anomaly = np.radians(mean_anomaly)
    E = mean_anomaly
    for _ in range(10):
        E = mean_anomaly + eccentricity * np.sin(E)
    true_anomaly = 2 * np.arctan2(np.sqrt(1 + eccentricity) * np.sin(E / 2), np.sqrt(1 - eccentricity) * np.cos(E / 2))
    r = semi_major_axis * (1 - eccentricity ** 2) / (1 + eccentricity * np.cos(true_anomaly))
    x_orbital = r * np.cos(true_anomaly)
    y_orbital = r * np.sin(true_anomaly)
    R1 = np.array([[np.cos(raan), -np.sin(raan), 0], [np.sin(raan), np.cos(raan), 0], [0, 0, 1]])
    R2 = np.array([[1, 0, 0], [0, np.cos(inclination), -np.sin(inclination)], [0, np.sin(inclination), np.cos(inclination)]])
    R3 = np.array([[np.cos(arg_periapsis), -np.sin(arg_periapsis), 0], [np.sin(arg_periapsis), np.cos(arg_periapsis), 0], [0, 0, 1]])
    rotation_matrix = R1 @ R2 @ R3
    position_orbital = np.array([x_orbital, y_orbital, 0])
    position_spacecraft = rotation_matrix @ position_orbital
    return position_spacecraft

def is_in_sunlight(position_spacecraft):
    sun_position = np.array([AU, 0, 0])
    vector_to_sun = sun_position - position_spacecraft
    distance_to_earth_center = np.linalg.norm(position_spacecraft)
    distance_to_sun_center = np.linalg.norm(vector_to_sun)
    angle_earth_sun_spacecraft = math.acos(np.dot(position_spacecraft, vector_to_sun) / (distance_to_earth_center * distance_to_sun_center))
    earth_shadow_radius = EARTH_RADIUS / distance_to_earth_center
    sun_radius_at_spacecraft_distance = SUN_RADIUS / distance_to_sun_center
    if angle_earth_sun_spacecraft < earth_shadow_radius + sun_radius_at_spacecraft_distance:
        return False
    else:
        return True

def calculate_ground_pass(position_spacecraft):
    latitude_spacecraft = math.degrees(math.asin(position_spacecraft[2] / np.linalg.norm(position_spacecraft)))
    longitude_spacecraft = math.degrees(math.atan2(position_spacecraft[1], position_spacecraft[0]))
    return latitude_spacecraft, longitude_spacecraft

def is_over_ground_station(latitude_spacecraft, longitude_spacecraft, ground_station_latitude, ground_station_longitude):
    global distance_threshold_km
    ground_station_latitude = np.radians(ground_station_latitude)
    ground_station_longitude = np.radians(ground_station_longitude)
    latitude_spacecraft_rad = np.radians(latitude_spacecraft)
    longitude_spacecraft_rad = np.radians(longitude_spacecraft)
    delta_latitude_rad = latitude_spacecraft_rad - ground_station_latitude
    delta_longitude_rad = longitude_spacecraft_rad - ground_station_longitude
    a = (np.sin(delta_latitude_rad / 2) ** 2 + np.cos(ground_station_latitude) * np.cos(latitude_spacecraft_rad) * np.sin(delta_longitude_rad / 2) ** 2)
    c = 2 * math.atan2(np.sqrt(a), np.sqrt(1 - a))
    distance_km = EARTH_RADIUS * c
    return distance_km <= distance_threshold_km

def simulate_and_animate_day(semi_major_axis, eccentricity, inclination, raan, arg_periapsis, orbit_sim_speed, ground_station_latitude, ground_station_longitude):
    global distance_threshold_km
    latitudes = []
    longitudes = []
    telemetry_statuses = []
    sunlight_statuses = []
    orbital_period_minutes = int((2 * math.pi * math.sqrt(semi_major_axis ** 3 / EARTH_MU)) // SECONDS_IN_MINUTE)
    num_frames_per_orbit = int(orbital_period_minutes*SECONDS_IN_MINUTE// orbit_sim_speed)
    for second in range(0, MINUTES_IN_DAY * SECONDS_IN_MINUTE, orbit_sim_speed):  # Simulate every # second
        mean_anomaly = (360 * (second % (orbital_period_minutes * SECONDS_IN_MINUTE)) / (orbital_period_minutes * SECONDS_IN_MINUTE)) % 360
        position_spacecraft = calculate_position(semi_major_axis, eccentricity, inclination, raan, arg_periapsis, mean_anomaly)
        latitude_spacecraft, longitude_spacecraft = calculate_ground_pass(position_spacecraft)
        longitude_spacecraft = (longitude_spacecraft + EARTH_ROTATION_RATE * (second / SECONDS_IN_MINUTE)) % 360
        latitudes.append(latitude_spacecraft)
        longitudes.append(longitude_spacecraft)
        telemetry_statuses.append(is_over_ground_station(latitude_spacecraft, longitude_spacecraft, ground_station_latitude, ground_station_longitude))
        sunlight_statuses.append(is_in_sunlight(position_spacecraft))

    fig, ax = plt.subplots(figsize=(5*1.2, 3*1.1), subplot_kw={'projection': ccrs.PlateCarree()})
    fig.patch.set_alpha(0)  # Make the figure's background transparent
    ax.set_facecolor((0, 0, 0, 0))  # Make the axes' background transparent
    ax.stock_img()
    line, = ax.plot([], [], 'r', transform=ccrs.Geodetic(), label='Spacecraft Trajectory')
    ground_station_marker, = ax.plot(ground_station_longitude, ground_station_latitude, 'ro', ms=10, transform=ccrs.Geodetic(), label='Ground Station')
    power_supply_marker, = ax.plot([], [], 'yD', ms=8, transform=ccrs.Geodetic(), label='Power Supply')
    # plt.legend(loc='lower left')
    plt.title('Spacecraft Trajectory and Ground Station')
    plt.tight_layout()

    def update(frame):
        global ps_previous_state, allow_orbit_sim_control, current_orbit_start_index
        # Clear trajectory if starting a new orbit
        if frame % num_frames_per_orbit == 0:
            current_orbit_start_index = frame

        line.set_data(longitudes[current_orbit_start_index:frame], latitudes[current_orbit_start_index:frame])
        if telemetry_statuses[frame]:
            ground_station_marker.set_color('#228B22')
        else:
            ground_station_marker.set_color('#DC143C')
        if sunlight_statuses[frame]:
            power_supply_marker.set_data([longitudes[frame]], [latitudes[frame]])
            power_supply_marker.set_color('yellow')
            if ps_previous_state == 1:
                ps_previous_state = 0
        else:
            power_supply_marker.set_data([longitudes[frame]], [latitudes[frame]])
            power_supply_marker.set_color('black')
            if ps_previous_state == 0:
                ps_previous_state = 1
        return line, ground_station_marker, power_supply_marker

    return fig, update, len(latitudes)


def toggle_orbit_sim_control():
    global allow_orbit_sim_control
    if allow_orbit_sim_control == 0:
        allow_orbit_sim_control = 1
        orbit_sim_control_button.text = "Disable Orbit Sim. from Controlling Power Supply"
        orbit_sim_control_button.color = GREEN
    else:
        allow_orbit_sim_control = 0
        orbit_sim_control_button.text = "Allow Orbit Sim. to Control Power Supply"
        orbit_sim_control_button.color = GREY

def set_orbital_radius(radius_text):
    global orbital_radius
    if is_number(radius_text):
        orbital_radius = float(radius_text)
        update_orbit_simulation()
    else:
        print("Invalid radius input")

def set_eccentricity(eccentricity_text):
    global eccentricity
    if is_number(eccentricity_text):
        eccentricity = float(eccentricity_text)
        update_orbit_simulation()
    else:
        print("Invalid eccentricity input")

def set_inclination(inclination_text):
    global inclination
    if is_number(inclination_text):
        inclination = float(inclination_text)
        update_orbit_simulation()
    else:
        print("Invalid inclination input")

def set_raan(raan_text):
    global raan
    if is_number(raan_text):
        raan = float(raan_text)
        update_orbit_simulation()
    else:
        print("Invalid RAAN input")

def set_arg_periapsis(arg_periapsis_text):
    global arg_periapsis
    if is_number(arg_periapsis_text):
        arg_periapsis = float(arg_periapsis_text)
        update_orbit_simulation()
    else:
        print("Invalid Arg. Periapsis input")

def set_orbit_sim_speed(orbit_sim_speed_input_text):
    global orbit_sim_speed
    if is_number(orbit_sim_speed_input_text):
        orbit_sim_speed = abs(int(orbit_sim_speed_input_text))
        update_orbit_simulation()
    else:
        print("Invalid Orbit Simulation Speed input")

def set_ground_station_latitude(ground_station_latitude_input_text):
    global ground_station_latitude
    if is_number(ground_station_latitude_input_text):
        ground_station_latitude = float(ground_station_latitude_input_text)
        update_orbit_simulation()
    else:
        print("Invalid Ground Station Latitude input")

def set_ground_station_longitude(ground_station_longitude_input_text):
    global ground_station_longitude
    if is_number(ground_station_longitude_input_text):
        ground_station_longitude = float(ground_station_longitude_input_text)
        update_orbit_simulation()
    else:
        print("Invalid Ground Station Longitude input")

def update_orbit_simulation():
    global fig, update, frames, canvas, renderer, raw_data
    global semi_major_axis, eccentricity, inclination, raan, arg_periapsis
    fig, update, frames = simulate_and_animate_day(semi_major_axis, eccentricity, inclination, raan, arg_periapsis, orbit_sim_speed, ground_station_latitude, ground_station_longitude)
    canvas = FigureCanvas(fig)
    canvas.draw()
    renderer = canvas.get_renderer()
    raw_data = renderer.buffer_rgba()

# Render default orbit parameters in the GUI
def render_default_orbit_params():
    global orbital_radius_input_text, eccentricity_input_text, inclination_input_text, raan_input_text, arg_periapsis_input_text, orbit_sim_speed_input_text
    global ground_station_longitude_input_text, ground_station_latitude_input_text
    orbital_radius_input_text = str(orbital_radius)
    eccentricity_input_text = str(eccentricity)
    inclination_input_text = str(inclination)
    raan_input_text = str(raan)
    arg_periapsis_input_text = str(arg_periapsis)
    orbit_sim_speed_input_text = str(orbit_sim_speed)
    ground_station_longitude_input_text = str(ground_station_longitude)
    ground_station_latitude_input_text = str(ground_station_latitude)


#########################################################################################################
#####################                    POWER SUPPLY SETUP                   ###########################
#########################################################################################################
# Initialize Power Supply
active_input = None
ps_default_voltage = 6.4
ps_default_current = 2

def initialize_ps_settings():
    global ps_voltage_input_text, ps_current_input_text
    try:
        ps.disable_output()
        ps.set_voltage(ps_default_voltage)
        ps.set_current(ps_default_current)
        ps.set_ocp(True)
        ps.set_ovp(True)
        ps.set_opp(True)
        ps_voltage_input_text = str(ps_default_voltage)
        ps_current_input_text = str(ps_default_current)
    except Exception as e:
        print(f"Failed to initialize settings: {e}")


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def set_ps_voltage(voltage_text):
    if is_number(voltage_text):
        voltage = float(voltage_text)
        ps.set_voltage(voltage)
    else:
        print("Invalid voltage input")

def set_ps_current(current_text):
    if is_number(current_text):
        current = float(current_text)
        ps.set_current(current)
    else:
        print("Invalid current input")

def toggle_ps_output():
    try:
        if ps.is_output_enabled():
            ps.disable_output()
        else:
            ps.enable_output()
    except Exception as e:
        print(f"Output toggle error: {e}")

def update_ps_status_thread():
    global running, ps_protection_status, ps_voltage_display, ps_current_display, ps_power_display, ps_output_status_display
    global ps_previous_state, allow_orbit_sim_control
    while running:
        if ps:
            ps_voltage = ps.get_voltage_display()
            ps_current = ps.get_current_display()
            ps_power = ps.get_power_display()
            ps_protection_status = ps.get_protection_status()
            ps_voltage_display = font.render(f"{ps_voltage:.2f}", True, BLACK)
            ps_current_display = font.render(f"{ps_current:.2f}", True, BLACK)
            ps_power_display = font.render(f"{ps_power:.2f}", True, BLACK)
            ps_output_status_display = font.render("Enabled" if ps.is_output_enabled() else "Disabled", True, GREEN if ps.is_output_enabled() else BLACK)

            if allow_orbit_sim_control and (ps_previous_state == 0):
                ps.enable_output()
            elif allow_orbit_sim_control and (ps_previous_state == 1):
                ps.disable_output()
        time.sleep(0.05)



#########################################################################################################
######################                 MAIN LOOP AND CODE                    ############################
#########################################################################################################
# Orbit parameters
orbital_radius = 416
semi_major_axis = EARTH_RADIUS + orbital_radius  # in km
eccentricity = 0.0002197
inclination = 51.6392  # in degrees
raan = 100.7922  # Right Ascension of Ascending Node in degrees
arg_periapsis = 50.6796  # Argument of Periapsis in degrees
orbit_sim_speed = 5 # speed in seconds for each simulation time step
ground_station_latitude = 45 # latitude in degrees
ground_station_longitude = -75 # longitude in degrees
distance_threshold_km = 3000 # radius to trigger ground station pass
render_default_orbit_params()

# Orbit animation
fig, update, frames = simulate_and_animate_day(semi_major_axis, eccentricity, inclination, raan, arg_periapsis, orbit_sim_speed, ground_station_latitude, ground_station_longitude)
canvas = FigureCanvas(fig)
canvas.draw()
renderer = canvas.get_renderer()
raw_data = renderer.buffer_rgba()
frame = 0  # Counter to update orbit sim. animation

# Main loop and GUI setup
running = True
text_window = pygame.Rect(50, 250, 380, 200)
received_data = []
max_lines = 10
example_gif = gif_pygame.load("cat.gif")
clock = pygame.Clock()
power_status = GREEN
adcs_status = GREEN
power_indicator = pygame.Rect(indicator_corner_anchor_x, indicator_corner_anchor_y, 20, 20)
adcs_indicator = pygame.Rect(indicator_corner_anchor_x, indicator_corner_anchor_y + 25, 20, 20)
comms_indicator = pygame.Rect(indicator_corner_anchor_x, indicator_corner_anchor_y + 50, 20, 20)
actuatorStatus = RED
misses = 0
oldPowerStatus = GREEN
oldADCSStatus = GREEN
voltage = 0
current = 0
windowcol = WHITE
tic = time.time()
playOnce = True
GUI_fps = 30

# Power supply status update thread
ps_status_thread = threading.Thread(target=update_ps_status_thread)
ps_status_thread.start()
# Serial connections check thread
serial_check_thread = threading.Thread(target=check_serial_connections_thread)
serial_check_thread.start()

while running:
    if (armCount > 0):
        actuatorColor = GREEN
    else:
        actuatorColor = RED

    toc = time.time()
    if (toc - tic > 0.5):
        if ser:
            ser.write(bytearray(b'\xAB'))
        tic = time.time()
        misses += 1
        armCount -= 1

    if (armCount < 0):
        armCount = 0

    clock.tick(GUI_fps)
    current_fps = round(clock.get_fps(),2)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            handle_com_port_selection(event)
            handle_ps_com_port_selection(event)
            if connect_serial_button.is_clicked(event.pos):
                connect_serial_button.on_click()
            if connect_ps_button.is_clicked(event.pos):
                connect_ps_button.on_click()
            for button in buttons:
                if button.is_clicked(event.pos):
                    button.on_click()
            if ps_voltage_input.collidepoint(event.pos):
                active_input = 'voltage'
            elif ps_current_input.collidepoint(event.pos):
                active_input = 'current'
            elif orbital_radius_input.collidepoint(event.pos):
                active_input = 'orbital_radius'
            elif eccentricity_input.collidepoint(event.pos):
                active_input = 'eccentricity'
            elif inclination_input.collidepoint(event.pos):
                active_input = 'inclination'
            elif raan_input.collidepoint(event.pos):
                active_input = 'raan'
            elif arg_periapsis_input.collidepoint(event.pos):
                active_input = 'arg_periapsis'
            elif orbit_sim_speed_input.collidepoint(event.pos):
                active_input = 'orbit_sim_speed'
            elif ground_station_latitude_input.collidepoint(event.pos):
                active_input = 'ground_station_latitude'
            elif ground_station_longitude_input.collidepoint(event.pos):
                active_input = 'ground_station_longitude'
            else:
                active_input = None
            if ps:
                if ps_set_voltage_button.is_clicked(event.pos):
                    ps_set_voltage_button.on_click()
                if ps_set_current_button.is_clicked(event.pos):
                    ps_set_current_button.on_click()
                if ps_set_output_button.is_clicked(event.pos):
                    ps_set_output_button.on_click()
                    if ps.is_output_enabled():
                        ps_set_output_button.text = "Output Enabled: Click to Disable Output"
                        ps_set_output_button.color = GREEN
                    else:
                        ps_set_output_button.text = "Output Disabled: Click to Enable Output"
                        ps_set_output_button.color = GREY
                if orbit_sim_control_button.is_clicked(event.pos):
                    orbit_sim_control_button.on_click()
            if set_orbital_radius_button.is_clicked(event.pos):
                set_orbital_radius_button.on_click()
            if set_eccentricity_button.is_clicked(event.pos):
                set_eccentricity_button.on_click()
            if set_inclination_button.is_clicked(event.pos):
                set_inclination_button.on_click()
            if set_raan_button.is_clicked(event.pos):
                set_raan_button.on_click()
            if set_arg_periapsis_button.is_clicked(event.pos):
                set_arg_periapsis_button.on_click()
            if set_orbit_sim_speed_button.is_clicked(event.pos):
                set_orbit_sim_speed_button.on_click()
            if set_ground_station_latitude_button.is_clicked(event.pos):
                set_ground_station_latitude_button.on_click()
            if set_ground_station_longitude_button.is_clicked(event.pos):
                set_ground_station_longitude_button.on_click()
        elif event.type == pygame.KEYDOWN:
            if active_input == 'voltage':
                if event.key == pygame.K_RETURN:
                    if is_number(ps_voltage_input_text):
                        set_ps_voltage(float(ps_voltage_input_text))
                elif event.key == pygame.K_BACKSPACE:
                    ps_voltage_input_text = ps_voltage_input_text[:-1]
                else:
                    ps_voltage_input_text += event.unicode
            elif active_input == 'current':
                if event.key == pygame.K_RETURN:
                    if is_number(ps_current_input_text):
                        set_ps_current(float(ps_current_input_text))
                elif event.key == pygame.K_BACKSPACE:
                    ps_current_input_text = ps_current_input_text[:-1]
                else:
                    ps_current_input_text += event.unicode
            elif active_input == 'orbital_radius':
                if event.key == pygame.K_RETURN:
                    if is_number(orbital_radius_input_text):
                        set_orbital_radius(float(orbital_radius_input_text))
                elif event.key == pygame.K_BACKSPACE:
                    orbital_radius_input_text = orbital_radius_input_text[:-1]
                else:
                    orbital_radius_input_text += event.unicode
            elif active_input == 'eccentricity':
                if event.key == pygame.K_RETURN:
                    if is_number(eccentricity_input_text):
                        set_eccentricity(float(eccentricity_input_text))
                elif event.key == pygame.K_BACKSPACE:
                    eccentricity_input_text = eccentricity_input_text[:-1]
                else:
                    eccentricity_input_text += event.unicode
            elif active_input == 'inclination':
                if event.key == pygame.K_RETURN:
                    if is_number(inclination_input_text):
                        set_inclination(float(inclination_input_text))
                elif event.key == pygame.K_BACKSPACE:
                    inclination_input_text = inclination_input_text[:-1]
                else:
                    inclination_input_text += event.unicode
            elif active_input == 'raan':
                if event.key == pygame.K_RETURN:
                    if is_number(raan_input_text):
                        set_raan(float(raan_input_text))
                elif event.key == pygame.K_BACKSPACE:
                    raan_input_text = raan_input_text[:-1]
                else:
                    raan_input_text += event.unicode
            elif active_input == 'arg_periapsis':
                if event.key == pygame.K_RETURN:
                    if is_number(arg_periapsis_input_text):
                        set_arg_periapsis(float(arg_periapsis_input_text))
                elif event.key == pygame.K_BACKSPACE:
                    arg_periapsis_input_text = arg_periapsis_input_text[:-1]
                else:
                    arg_periapsis_input_text += event.unicode
            elif active_input == 'orbit_sim_speed':
                if event.key == pygame.K_RETURN:
                    if is_number(orbit_sim_speed_input_text) and int(orbit_sim_speed_input_text) <= 60:
                        set_orbit_sim_speed(int(orbit_sim_speed_input_text))
                    elif is_number(orbit_sim_speed_input_text) and int(orbit_sim_speed_input_text) > 60:
                        orbit_sim_speed_input_text = '60'
                        set_orbit_sim_speed(int(orbit_sim_speed_input_text))
                elif event.key == pygame.K_BACKSPACE:
                    orbit_sim_speed_input_text = orbit_sim_speed_input_text[:-1]
                else:
                    orbit_sim_speed_input_text += event.unicode
            elif active_input == 'ground_station_latitude':
                if event.key == pygame.K_RETURN:
                    if is_number(ground_station_latitude_input_text):
                        if (float(ground_station_latitude_input_text) > 90):
                            ground_station_latitude_input_text = '90'
                        elif (float(ground_station_latitude_input_text) < -90):
                            ground_station_latitude_input_text = '-90'
                        set_ground_station_latitude(float(ground_station_latitude_input_text))
                elif event.key == pygame.K_BACKSPACE:
                    ground_station_latitude_input_text = ground_station_latitude_input_text[:-1]
                else:
                    ground_station_latitude_input_text += event.unicode
            elif active_input == 'ground_station_longitude':
                if event.key == pygame.K_RETURN:
                    if is_number(ground_station_longitude_input_text):
                        if (float(ground_station_longitude_input_text) > 180):
                            ground_station_longitude_input_text = '180'
                        elif (float(ground_station_longitude_input_text) < -180):
                            ground_station_longitude_input_text = '-180'
                        set_ground_station_longitude(float(ground_station_longitude_input_text))
                elif event.key == pygame.K_BACKSPACE:
                    ground_station_longitude_input_text = ground_station_longitude_input_text[:-1]
                else:
                    ground_station_longitude_input_text += event.unicode

    if misses > 50:
        comms_status = RED
        windowcol = OFF_RED
        playOnce = False
    else:
        windowcol = WHITE
        comms_status = GREEN

    if ser and ser.in_waiting > 0:
        try:
            misses = 0
            new_data = ser.read_all()
            new_data = new_data.decode('utf-8')
            if new_data == "POWER COMM LOST\n":
                power_status = RED
            elif new_data == "POWER COMM RESTORED\n" or "POWINFO" in new_data:
                power_status = GREEN
            if new_data == "ADCS COMM LOST\n":
                adcs_status = RED
            elif new_data == "ADCS COMM RESTORED\n":
                adcs_status = GREEN
            elif new_data == "0xAB":
                misses = 0
                continue

            if "POWINFO" in new_data:
                temp = new_data.split(" ")
                voltage = temp[1]
                current = temp[2]
                continue

            print(new_data)
            for line in new_data.splitlines():
                if '\r' in line:
                    received_data[-1] = line.replace('\r', '')
                else:
                    received_data.append(line)
                if len(received_data) > max_lines:
                    received_data = received_data[-max_lines:]
        except:
            print(new_data)
            print("bruh")
    else:
        misses += 1

    window.fill(windowcol)
    example_gif.render(window, (500, 230))

    newColor = GREEN
    for button in buttons:
        button.draw(window)
        if button.group == "ACTUATORS":
            newColor = actuatorColor
            button.update(newColor)
        elif button.group == "POWER":
            button.update(power_status)
        elif button.group == "ADCS":
            button.update(adcs_status)

    voltage_label = font.render("Voltage", True, BLACK)
    current_label = font.render("Current", True, BLACK)
    window.blit(voltage_label, (voltage_anchor_x, voltage_anchor_y))
    window.blit(current_label, (current_anchor_x, current_anchor_y))
    voltagestr = "" + str(voltage)
    currentstr = "" + str(current)
    voltage_num = font.render(voltagestr, True, BLACK)
    current_num = font.render(currentstr, True, BLACK)
    window.blit(voltage_num, (voltage_anchor_x + 55, voltage_anchor_y))
    window.blit(current_num, (current_anchor_x + 55, current_anchor_y))

    # Display the GUI main loop rate - helps to identify if GUI is slowing down
    GUI_loop_rate_color = BLACK
    if (current_fps-GUI_fps+2) < 0:
        GUI_loop_rate_color = GREEN
    GUI_loop_rate_label = font.render("GUI Update Rate (Hz):", True, GUI_loop_rate_color)
    GUI_loop_rate_num = font.render(str(round(clock.get_fps(), 2)), True, GUI_loop_rate_color)
    window.blit(GUI_loop_rate_label, (GUI_loop_rate_anchor_x, GUI_loop_rate_anchor_y))
    window.blit(GUI_loop_rate_num, (GUI_loop_rate_anchor_x + 140, GUI_loop_rate_anchor_y))

    pygame.draw.rect(window, BLACK, text_window, 2)
    y_offset = text_window.y + 5
    for line in received_data:
        try:
            text_surface = font.render(line, True, BLACK)
            window.blit(text_surface, (text_window.x + 5, y_offset))
            y_offset += font.get_height()
        except:
            received_data.remove(line)

    draw_labels()
    pygame.draw.rect(window, power_status, power_indicator)
    pygame.draw.rect(window, adcs_status, adcs_indicator)
    pygame.draw.rect(window, comms_status, comms_indicator)

    update_orbit_and_PS_GUI_elements_in_loop()

    # # Update the orbit simulation frame
    update(frame)
    frame = (frame + 1) % frames

    # Render the Matplotlib figure to a Pygame surface
    canvas.draw()
    raw_data = canvas.get_renderer().buffer_rgba()
    size = canvas.get_width_height()
    surf = pygame.image.frombuffer(raw_data.tobytes(), size, "RGBA")
    window.blit(surf, (width - size[0]+10, height - size[1]+6))

    # Update dropdown menu for serial connections
    draw_com_port_dropdown()
    draw_ps_com_port_dropdown()
    connect_serial_button.draw(window)
    connect_ps_button.draw(window)

    pygame.display.flip()


running = False
ps_status_thread.join()
serial_check_thread.join()

if ser:
    ser.close()
if ps:
    ps.close()
pygame.quit()

