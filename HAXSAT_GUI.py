import datetime
import pygame
import serial
import serial.tools.list_ports
import time
import threading
import os

# Imports for orbit sim and orbit animation
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import numpy as np
import math
import matplotlib.pyplot as plt
import cartopy.crs as ccrs

# Imports for Power Supply
from pyHM310T import PowerSupply

# Initialize Pygame
pygame.init()

indicator_corner_anchor_x = 750
indicator_corner_anchor_y = 290

voltage_anchor_x = 750
current_anchor_x = 750
voltage_anchor_y = 420
current_anchor_y = 445
GUI_loop_rate_anchor_x = 450
GUI_loop_rate_anchor_y = 15

# Set up the display
width, height = 1250, 825

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
# # Create a directory to store images
# if not os.path.exists('Downlinked Images'):
#     os.makedirs('Downlinked Images')

pygame.mixer.init()
pygame.display.set_caption("HAXSat")
window = pygame.display.set_mode((width, height))
font = pygame.font.Font(None, 20)
small_font = pygame.font.Font(None, 12)

text_window = pygame.Rect(50, 250, 380, 200)
tic = time.time()
tic1 = time.time()
clock = pygame.Clock()

power_indicator = pygame.Rect(indicator_corner_anchor_x, indicator_corner_anchor_y, 20, 20)
adcs_indicator = pygame.Rect(indicator_corner_anchor_x, indicator_corner_anchor_y + 25, 20, 20)
comms_indicator = pygame.Rect(indicator_corner_anchor_x, indicator_corner_anchor_y + 50, 20, 20)

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GREY = (160, 160, 160)
DISABLED_GREY = (100, 100, 100)
BACKGROUND_GREY = (200, 200, 200)
OFF_RED = (240, 128, 128)
ORANGE = (252, 186, 3)
PURPLE = (100, 0, 125)

# Set up the serial connection
# ser = serial.Serial('COM9', 115200)  # Change 'COM3' to your COM port
# ser.timeout = 0.5

# actuator arming logic variables
actuatorColor = RED
armCount = 0
received_data = []

misses = 0
voltage = 0
current = 0

running = True
max_lines = 10

# Indicators for power and ADCS status
power_status = RED
adcs_status = RED
comms_status = RED
actuatorStatus = RED
oldPowerStatus = GREEN
oldADCSStatus = GREEN
payload_status = GREEN
solar_charging_status = GREEN

windowcol = WHITE

dataLength = 64
headerIndex = 0
typeIndex = 1
lengthIndex = 2
continuedIndex = 3
indexIndex = 4
dataIndex = 6
crcIndex = dataIndex + dataLength
footerIndex = crcIndex + 4
totalDataLength = footerIndex + 1


def updateStatus():
    power_label = font.render("Power Status", True, BLACK)
    adcs_label = font.render("ADCS Status", True, BLACK)
    comms_label = font.render("CDH Status", True, BLACK)
    window.blit(power_label, (indicator_corner_anchor_x + 25, indicator_corner_anchor_y + 2.5))
    window.blit(adcs_label, (indicator_corner_anchor_x + 25, indicator_corner_anchor_y + 27.5))
    window.blit(comms_label, (indicator_corner_anchor_x + 25, indicator_corner_anchor_y + 52.5))
    pygame.draw.rect(window, power_status, power_indicator)
    pygame.draw.rect(window, adcs_status, adcs_indicator)
    pygame.draw.rect(window, comms_status, comms_indicator)


def drawVoltageAndCurrent(voltage, current):
    voltage_label = font.render("Voltage [V]:", True, BLACK)
    current_label = font.render("Current [mA]:", True, BLACK)
    window.blit(voltage_label, (voltage_anchor_x, voltage_anchor_y))
    window.blit(current_label, (current_anchor_x, current_anchor_y))

    current_GUI = round((sum(current)/len(current))/3.5, 2)

    voltagestr = "" + str(voltage)
    currentstr = "" + str(current_GUI)
    voltage_num = font.render(voltagestr, True, BLACK)
    current_num = font.render(currentstr, True, BLACK)
    window.blit(voltage_num, (voltage_anchor_x + 90, voltage_anchor_y))
    window.blit(current_num, (current_anchor_x + 90, current_anchor_y))

def drawLoopRate(set_GUI_fps, current_fps):
    # Display the GUI main loop rate - helps to identify if GUI is slowing down
    GUI_loop_rate_color = BLACK
    if ((current_fps-set_GUI_fps)/set_GUI_fps) < -0.1:
        GUI_loop_rate_color = RED
    GUI_loop_rate_label = font.render("GUI Update Rate [Hz]:", True, GUI_loop_rate_color)
    GUI_loop_rate_num = font.render(str(round(clock.get_fps(), 2)), True, GUI_loop_rate_color)
    window.blit(GUI_loop_rate_label, (GUI_loop_rate_anchor_x, GUI_loop_rate_anchor_y))
    window.blit(GUI_loop_rate_num, (GUI_loop_rate_anchor_x + 140, GUI_loop_rate_anchor_y))

def drawTextToScreen(received_data):
    # Draw the text window
    pygame.draw.rect(window, BLACK, text_window, 2)
    y_offset = text_window.y + 5
    if(len(received_data) > max_lines):
        received_data.pop(0)
    for line in received_data:
        try:
            text_surface = font.render(line, True, BLACK)
            window.blit(text_surface, (text_window.x + 5, y_offset))
            y_offset += font.get_height()
        except:
            received_data.remove(line)


def armActuators():
    global actuatorColor
    global armCount
    actuatorColor = GREEN
    armCount = 300

def sendTime():
    t = datetime.datetime.now()
    header = 0xDD
    year = t.year % 100
    month = t.month
    day = t.day
    hour = t.hour
    minute = t.minute
    second = t.second
    s = bytearray([header, year, month, day, hour, minute, second])
    ser.write(s)
    print(s)

def sendAck():
    ser.write(bytearray(b'\xCC\xAA'))


def sendNack():
    ser.write(bytearray(b'\xCC\xBB'))


class Button:
    def __init__(self, x, y, width, height, color, message, text, action = None, group = None):
        self.rect = pygame.Rect(x, y, width, height)
        self.color = color
        self.message = message
        self.text = text
        self.action = action
        self.group = group
        self.disabled = False
        self.defaultColor = color

    def draw(self, surface):
        pygame.draw.rect(surface, self.color, self.rect)
        text_surface = font.render(self.text, True, BLACK)
        text_rect = text_surface.get_rect(center=self.rect.center)
        surface.blit(text_surface, text_rect)

    def disable(self):
        self.disabled = True
        self.defaultColor = self.color
        self.color = DISABLED_GREY

    def enable(self):
        self.disabled = False
        self.color = self.defaultColor

    def update(self, newColor):
        self.color = newColor

    def is_clicked(self, pos):
        return self.rect.collidepoint(pos)

    def on_click(self):
        self.defaultColor = self.color

        if self.action:
            self.action()
        else:
            if(self.color != RED):
                ser.write(self.message)

    def off_click(self):
        self.color = self.defaultColor


class indicator:
    def __init__(self, x, y, color):
        self.rect = pygame.Rect(x, y, 20, 20)
        self.color = color

    def draw(self, surface):
        pygame.draw.rect(surface, self.color, self.rect)

    def update(self, newColor):
        self.color = newColor


class label:
    def __init__(self, x, y, text, font_size=20):
        self.font = pygame.font.Font(None, font_size)
        self.x = x
        self.y = y
        self.text = text

    def draw(self, surface):
        text_surface = self.font.render(self.text, True, BLACK)
        text_rect = text_surface.get_rect(center=(self.x, self.y))
        surface.blit(text_surface, text_rect)


class scrollable_textbox:
    def __init__(self, x, y, width, height, color):
        self.rect = pygame.Rect(x, y, width, height)
        self.color = color

    def draw(self, surface):
        pygame.draw.rect(surface, self.color, self.rect)


class ScrollableTextbox:
    def __init__(self, x, y, width, height, color, font):
        self.rect = pygame.Rect(x, y, width, height)
        self.color = color
        self.font = font
        self.text_lines = []
        self.scroll_offset = 0
        self.max_lines = height // font.get_height()
        self.snap_to_bottom = True
        self.checkbox = Checkbox(x + width - 20, y, 20, "Snap", font)
        self.clearbox = Checkbox(x + width - 20, y + 20, 20, "Clear", font)

    def append_text(self, text):
        t = datetime.datetime.now()
        s = t.strftime('%H:%M:%S.%f')
        text = s[:-3] + " - " + text
        maxLength = 48
        if len(text) > maxLength:  # Assuming 50 characters as the max length for a single line
            self.text_lines.append(text[:maxLength])
            self.text_lines.append(text[maxLength:])
        else:
            self.text_lines.append(text)
        self.snap_to_bottom = self.checkbox.checked
        if self.snap_to_bottom:
            if len(self.text_lines) > self.max_lines:
                self.scroll_offset = len(self.text_lines) - self.max_lines
            else:
                self.scroll_offset = 0

    def scroll(self, direction):
        if direction == 1 and self.scroll_offset > 0:
            self.scroll_offset -= 1
        elif direction == -1 and self.scroll_offset < len(self.text_lines) - self.max_lines:
            self.scroll_offset += 1

    def draw(self, surface):
        if (self.clearbox.checked):
            self.text_lines = []
            self.clearbox.checked = False
        pygame.draw.rect(surface, self.color, self.rect)
        pygame.draw.rect(surface, BLACK, self.rect, 2)  # Add a border
        self.checkbox.draw(surface)
        self.clearbox.draw(surface)
        y_offset = self.rect.y
        for i in range(self.scroll_offset, min(len(self.text_lines), self.scroll_offset + self.max_lines)):
            text_surface = self.font.render(self.text_lines[i], True, BLACK)
            surface.blit(text_surface, (self.rect.x + 5, y_offset))
            y_offset += self.font.get_height()

        # Draw the scrollbar
        if len(self.text_lines) > self.max_lines:
            scrollbar_height = self.rect.height * (self.max_lines / len(self.text_lines))
            scrollbar_y = self.rect.y + (self.scroll_offset / len(self.text_lines)) * self.rect.height
            scrollbar_rect = pygame.Rect(self.rect.right - 10, scrollbar_y, 10, scrollbar_height)
            pygame.draw.rect(surface, GREY, scrollbar_rect)


class Checkbox:
    def __init__(self, x, y, size, text, font, checked=True):
        self.rect = pygame.Rect(x, y, size, size)
        self.text = text
        self.font = font
        self.checked = checked

    def draw(self, surface):
        pygame.draw.rect(surface, BLACK, self.rect, 2)
        if self.checked:
            pygame.draw.line(surface, BLACK, (self.rect.left, self.rect.top), (self.rect.right, self.rect.bottom), 2)
            pygame.draw.line(surface, BLACK, (self.rect.right, self.rect.top), (self.rect.left, self.rect.bottom), 2)
        text_surface = self.font.render(self.text, True, BLACK)
        surface.blit(text_surface, (self.rect.right + 10, self.rect.centery - text_surface.get_height() // 2))

    def is_clicked(self, pos):
        return self.rect.collidepoint(pos)

    def toggle(self):
        self.checked = not self.checked


# Define buttons with unique serial messages and text
buttons = [
    Button(50, 80, 400, 50, RED, "bruh", "ARM", armActuators, "ARMING"),
    Button(50, 140, 100, 50, RED, bytearray(b'\x01\x08\x00'), "Left Out", None, "ACTUATORS"),
    Button(50, 200, 100, 50, GREEN, bytearray(b'\x01\x0A\x00'), "Left In", None, "ACTUATORS"),
    Button(350, 140, 100, 50, RED, bytearray(b'\x01\x0C\x00'), "Both Out", None, "ACTUATORS"),
    Button(200, 140, 100, 50, RED, bytearray(b'\x01\x09\x00'), "Right Out", None, "ACTUATORS"),
    Button(200, 200, 100, 50, GREEN, bytearray(b'\x01\x0B\x00'), "Right In", None, "ACTUATORS"),
    Button(350, 200, 100, 50, GREEN, bytearray(b'\x01\x0D\x00'), "Both In", None, "ACTUATORS"),
    Button(950, 80, 100, 50, PURPLE, bytearray(b'\x00\x00'), "Ping Power", None, "POWER"),
    Button(1100, 80, 100, 50, PURPLE, bytearray(b'\x00\x01\x01'), "Get V and C", None, "POWER"),
    Button(800, 80, 100, 50, GREY, bytearray(b'\x01\x01\x02'), "Sync", None, "ADCS"),
    Button(500, 80, 100, 50, GREY, bytearray(b'\x01\x02\x0D'), "Gyro", None, "ADCS"),
    Button(500, 140, 100, 50, GREY, bytearray(b'\x01\x03\x0B'), "Mag", None, "ADCS"),
    Button(500, 200, 100, 50, GREY, bytearray(b'\x01\x07\x03'), "Temp", None, "ADCS"),
    Button(650, 80, 100, 50, GREY, bytearray(b'\x01\x04\x15'), "Lat/Long/Alt", None, "ADCS"),
    Button(650, 140, 100, 50, GREY, bytearray(b'\x01\x05\x11'), "Time", None, "ADCS"),
    Button(650, 200, 100, 50, GREY, bytearray(b'\x01\x06\x05'), "Num Sats", None, "ADCS"),
    Button(800, 140, 100, 50, GREY, bytearray(b'\x01\x0E\x00'), "Reset GNSS", None, "ADCS"),
    Button(800, 200, 100, 50, GREY, bytearray(b'\x01\x0F\x02'), "Get data rate", None, "ADCS"),
    Button(1000, 140, 50, 50, GREY, bytearray(b'\x00\x04\x02\x01'), "ON", None, "CCLSM"),#CDH
    Button(1050, 140, 50, 50, GREY, bytearray(b'\x00\x04\x02\x00'), "OFF", None, "CCLSM"),
    Button(1100, 140, 50, 50, GREY, bytearray(b'\x00\x04\x02\x03'), "DATA", None, "CCLSM"),
    Button(1000, 200, 50, 50, GREY, bytearray(b'\x00\x04\x01\x01'), "ON", None, "CCLSM"), #ADCS
    Button(1050, 200, 50, 50, GREY, bytearray(b'\x00\x04\x01\x00'), "OFF", None, "CCLSM"),
    Button(1100, 200, 50, 50, GREY, bytearray(b'\x00\x04\x01\x02'), "DATA", None, "CCLSM"),
    Button(1000, 260, 50, 50, GREY, bytearray(b'\x01\x12\x00'), "ON", None, "CCLSM"), #CAMERA
    Button(1050, 260, 50, 50, GREY, bytearray(b'\x01\x11\x00'), "OFF", None, "CCLSM"),
    Button(850, 480, 75, 150, GREEN, bytearray(b'\x02\x01'), "Take\nImage", None, "BALLIN"),
    Button(850, 650, 75, 150, GREEN, bytearray(b'\x02\x02'), "Downlink\nImage", None, "BALLIN"),
    Button(950, 345, 70, 75, GREEN, bytearray(b'\x99\x99'), "Format\nFS", None, "BALLIN"),
    Button(1040, 345, 70, 75, GREEN, bytearray(b'\x02\x03'), "Get\nCDH\ntime", None, "BALLIN"),
    Button(1130, 345, 70, 75, GREEN, bytearray(b'\x02\x05'), "Set\nCDH\ntime", sendTime, "BALLIN"),
]

get_telem_button = Button(570, 570, 75, 105, GREEN, bytearray(b'\x08\x08'), "Get\nTelem", None, "BALLIN")
stop_telem_button = Button(570, 700, 75, 105, GREEN, bytearray(b'\x09\x09'), "Stop\nTelem", None, "BALLIN")
buttons.append(get_telem_button)
buttons.append(stop_telem_button)

indicators = {
    "CDH_CCLSM":indicator(1000, 140, RED),
    "ADCS_CCLSM":indicator(1000, 200, RED),
    "CAMERA_CCLSM":indicator(1000, 260, RED),
    # "POWER_STATUS":indicator(800, 410, RED),
    # "ADCS_STATUS":indicator(800, 410, RED),
}

labels = [
    label(970, 165, "CDH\nPOWER", 15),
    label(970, 225, "ADCS\nPOWER", 15),
    # label(970, 285, "OTHER\nPOWER", 15),
    label(970, 285, "CAMERA\nPOWER", 15),
    label(90, 270, "Messages from CDH", 15),
    label(980, 455, "Downlinked Image", 15),
    label(80, 65, "For Deployables", 15),
    label(515, 65, "For ADCS", 15),
    label(965, 65, "For Power", 15),
    label(indicator_corner_anchor_x+45, 270, "Communication Status", 15),
    label(voltage_anchor_x+40, 400, "Power Consumption", 15),
]

textbox = ScrollableTextbox(40, 280, 600, 200, WHITE, font)


def crc32b(tmpString):
    crc = 0xFFFFFFFF
    string = list(tmpString)
    for char in string:
        ch = char % 256
        for i in range(8):
            b = (ch ^ crc) & 1
            crc = crc>>1
            if(b):
                crc = crc ^ 0x04C11DB7
            ch = ch>>1
    return (~crc) % (1 << 32)


new_data = []


def SerialRx():
    global ser
    global new_data
    global running
    while(running) and ser:
        if ser.in_waiting >= 79:
            new_data = ser.read_all()
            try:
                ser.write(b'\xAA\xBB\xCC')
            except:
                pass

def draw_image_black_box_outline():
    outline_rect = pygame.Rect(935, 465, 270, 350)
    pygame.draw.rect(window, BLACK, outline_rect, 2)

def draw_Deployables_buttons_black_box_outline():
    outline_rect = pygame.Rect(40, 75, 420, 180)
    pygame.draw.rect(window, BLACK, outline_rect, 2)

def draw_ADCS_buttons_black_box_outline():
    outline_rect = pygame.Rect(490, 75, 420, 180)
    pygame.draw.rect(window, BLACK, outline_rect, 2)

def draw_Power_buttons_black_box_outline():
    outline_rect = pygame.Rect(940, 75, 270, 240)
    pygame.draw.rect(window, BLACK, outline_rect, 2)

def draw_Communication_Status_black_box_outline():
    outline_rect = pygame.Rect(indicator_corner_anchor_x-10, indicator_corner_anchor_y - 10, 170, 90)
    pygame.draw.rect(window, BLACK, outline_rect, 2)

def draw_Power_Consumption_status_black_box_outline():
    outline_rect = pygame.Rect(voltage_anchor_x-10, voltage_anchor_y-10, 170, 60)
    pygame.draw.rect(window, BLACK, outline_rect, 2)

##################################################################################################
# Button to allow orbit simulation to control power supply
orbit_sim_control_button = Button(40, 495, 522.5, 30, GREY, None, "Allow Orbit Sim. to Control Power Supply & Auto Telemetry Downlink", lambda: toggle_orbit_sim_control())

# Connect buttons for the serial COM ports
connect_serial_button = Button(width-575, 5, 125, 25, GREY, None, "Connect Radio", lambda: toggle_serial_connection())
connect_ps_button = Button(width-250, 5, 100, 25, GREY, None, "Connect PS", lambda: toggle_ps_connection())

def toggle_serial_connection():
    global ser, running, serialRX_thread, thread_started
    if ser:
        running = False
        serialRX_thread.join()
        ser.close()
        ser = None
        connect_serial_button.text = "Connect Radio"
        connect_serial_button.color = GREY
        running = True  # Reset running for the next connection
        thread_started = 1
    else:
        initialize_serial_connection()
        if ser:
            serialRX_thread = threading.Thread(target=SerialRx)  # Reinitialize the thread
            serialRX_thread.start()
            connect_serial_button.text = "Disconnect Radio"
            connect_serial_button.color = GREEN
            thread_started = 0

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

##################################################################################################
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
allow_auto_telemetry_downlink_previous_state = 1 # start with enabling auto telemetry downlink when over ground station
get_downlink_telem_status = 0

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

    fig, ax = plt.subplots(figsize=(5*1.1, 3*1), subplot_kw={'projection': ccrs.PlateCarree()})
    fig.patch.set_alpha(0)  # Make the figure's background transparent
    ax.set_facecolor((0, 0, 0, 0))  # Make the axes' background transparent
    ax.stock_img()
    line, = ax.plot([], [], 'r', transform=ccrs.Geodetic(), label='Spacecraft Trajectory')
    ground_station_marker, = ax.plot(ground_station_longitude, ground_station_latitude, 'ro', ms=10, transform=ccrs.Geodetic(), label='Ground Station')
    power_supply_marker, = ax.plot([], [], 'yD', ms=8, transform=ccrs.Geodetic(), label='Power Supply')
    # plt.legend(loc='lower left')
    plt.title('Spacecraft Trajectory and Ground Station', fontsize=9)
    plt.tight_layout()

    def update(frame):
        global ps_previous_state, allow_orbit_sim_control, current_orbit_start_index, allow_auto_telemetry_downlink_previous_state
        # Clear trajectory if starting a new orbit
        if frame % num_frames_per_orbit == 0:
            current_orbit_start_index = frame

        line.set_data(longitudes[current_orbit_start_index:frame], latitudes[current_orbit_start_index:frame])
        if telemetry_statuses[frame]:
            ground_station_marker.set_color('#228B22') # not over ground station
            if allow_auto_telemetry_downlink_previous_state == 1:
                allow_auto_telemetry_downlink_previous_state = 0
        else:
            ground_station_marker.set_color('#DC143C') # over ground station
            if allow_auto_telemetry_downlink_previous_state == 0:
                allow_auto_telemetry_downlink_previous_state = 1

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
        orbit_sim_control_button.text = "Disable Orbit Sim. to Control Power Supply & Auto Telemetry Downlink"
        orbit_sim_control_button.color = GREEN
    else:
        allow_orbit_sim_control = 0
        orbit_sim_control_button.text = "Allow Orbit Sim. to Control Power Supply & Auto Telemetry Downlink"
        orbit_sim_control_button.color = GREY

def update_oribt_animation():
    global frame, size, surf
    # Update the orbit simulation frame
    update(frame)
    frame = (frame + 1) % frames
    # Render the Matplotlib figure to a Pygame surface
    canvas.draw()
    raw_data = canvas.get_renderer().buffer_rgba()
    size = canvas.get_width_height()
    surf = pygame.image.frombuffer(raw_data.tobytes(), size, "RGBA")

def auto_telemetry_downlink():
    global ser, allow_auto_telemetry_downlink_previous_state, allow_orbit_sim_control, get_downlink_telem_status
    if ser and allow_orbit_sim_control and (allow_auto_telemetry_downlink_previous_state==0) and (get_downlink_telem_status == 0):
        get_telem_button.on_click()
        get_downlink_telem_status = 1
        print("Over Ground Station. Downlinking Data")
    elif ser and allow_orbit_sim_control and (allow_auto_telemetry_downlink_previous_state==1) and (get_downlink_telem_status == 1):
        stop_telem_button.on_click()
        get_downlink_telem_status = 0
        print("NOT Over Ground Station. Downlinking Data STOPPED")

##################################################################################################
#########################################################################################################
#####################                    POWER SUPPLY SETUP                   ###########################
#########################################################################################################
# Initialize Power Supply
active_input = None
ps_default_voltage = 8.4 # set power supply to default to 8.2V
ps_default_current = 2 # set power supply to output maximum 1A

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
orbit_sim_speed = 10 # speed in seconds for each simulation time step
ground_station_latitude = 45 # latitude in degrees
ground_station_longitude = -75 # longitude in degrees
distance_threshold_km = 3000 # radius to trigger ground station pass

# Orbit animation
fig, update, frames = simulate_and_animate_day(semi_major_axis, eccentricity, inclination, raan, arg_periapsis, orbit_sim_speed, ground_station_latitude, ground_station_longitude)
canvas = FigureCanvas(fig)
canvas.draw()
renderer = canvas.get_renderer()
raw_data = renderer.buffer_rgba()
frame = 0  # Counter to update orbit sim. animation
update_oribt_animation()

# Power supply status update thread
ps_status_thread = threading.Thread(target=update_ps_status_thread)
ps_status_thread.start()
# Serial connections check thread
serial_check_thread = threading.Thread(target=check_serial_connections_thread)
serial_check_thread.start()

# Thread to receive serial data
serialRX_thread = threading.Thread(target=SerialRx)


##################################################################################################
current_list = [0]
buildUpData = []
imageData = []
expectedIndex = 0

telemetry_save_directory = 'Downlinked Telemetry'
image_save_directory = 'Downlinked Images'
os.makedirs(image_save_directory, exist_ok=True)
os.makedirs(telemetry_save_directory, exist_ok=True)
filename = "received_image_" + time.strftime("%Y%m%d_%H%M%S") + ".jpg"
telemetry_filename = "telemetry_" + time.strftime("%Y%m%d_%H%M%S") + ".txt"

set_GUI_fps = 60
thread_started = 1
while running:
    if ser and thread_started: # start SerialRx thread only once we've connected to the radio serial port
        serialRX_thread.start()
        thread_started = 0

    clock.tick(set_GUI_fps)
    current_fps = round(clock.get_fps(), 2)

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
                    try:
                        button.on_click()
                    except:
                        pass
            if textbox.checkbox.is_clicked(event.pos):
                textbox.checkbox.toggle()
            if textbox.clearbox.is_clicked(event.pos):
                textbox.clearbox.toggle()
            if orbit_sim_control_button.is_clicked(event.pos):
                orbit_sim_control_button.on_click()
        elif event.type == pygame.MOUSEBUTTONUP:
            for button in buttons:
                if button.is_clicked(event.pos):
                    button.off_click()
        elif event.type == pygame.MOUSEWHEEL:
            if text_window.collidepoint(pygame.mouse.get_pos()):
                textbox.scroll(event.y)

    # check timeout on arm count
    misses = misses + 1
    armCount = armCount - 1
    if(armCount < 0):
        actuatorColor = RED
        armCount = 0
    else:
        actuatorColor = GREEN

    # ping CDH periodically
    toc = time.time()
    if(toc - tic > 0.25) and ser:
        try:
            ser.write(bytearray(b'\xAB'))
        except:
            pass
        tic = time.time()

    # if CDH hasn't responded in a while, change status to red
    if misses > 100:
        comms_status = RED
        adcs_status = RED
        power_status = RED
        indicators["ADCS_CCLSM"].update(RED)
        indicators["CAMERA_CCLSM"].update(RED)
        indicators["CDH_CCLSM"].update(RED)
        windowcol = OFF_RED
        if ser:
            ser.write(b'\xAA\xBB\xCC')
            time.sleep(0.1)
    else:
        comms_status = GREEN
        windowcol = WHITE

    # Read serial data
    if(len(new_data) >= 79):
        try:
            if(new_data[0] == 0xAA):
                misses = 0
                type = new_data[1]
                myLen = new_data[2]
                # rxCRC = new_data[76-5:76-1]
                # crc = crc32b(new_data[0:67])
                index = (new_data[3] << 8) | new_data[4]

                match type:
                    case 0x05:  #powinfo
                        stringData = new_data[6:6 + myLen - 1]
                        stringData = stringData.decode('utf-8')
                        stringData = stringData.split(' ')
                        voltage = stringData[0]
                        current = stringData[1]
                        current_list.append(float(current))
                        if len(current_list) > 5:
                            current_list.pop(0)
                        bits = stringData[2]
                        # CAMERACCLSMRx = (int(bits) & 0x02) >> 1
                        CDHCCLSMRx = (int(bits) & 0x02)
                        ADCSCCLSMRx = (int(bits) & 0x01)

                        indicators["ADCS_CCLSM"].update(RED if ADCSCCLSMRx == 0 else GREEN)
                        indicators["CDH_CCLSM"].update(RED if CDHCCLSMRx == 0 else GREEN)
                        pass
                    case 0x0A:  #printable
                        stringData = new_data[6:6+myLen-1]
                        textbox.append_text(stringData.decode('utf-8'))
                        print(stringData.decode('utf-8'))
                        pass
                    case 0x10:  #raw hex data to print
                        hexString = ""
                        temp = new_data[6:6+myLen-1]
                        for char in temp:
                            hexString = hexString + hex(char) + " "
                        hexString = hexString + "\n"
                        textbox.append_text(hexString)
                        print(hexString)
                        pass
                    case 0x19:  # telemetry data
                        hexString = ""
                        temp = new_data[6:6 + myLen - 1]
                        for char in temp:
                            hexString = hexString + hex(char) + " "
                        hexString = hexString + "\n"
                        # textbox.append_text(hexString)
                        # print(hexString)
                        print(temp)
                        telemetry_file_path = os.path.join(telemetry_save_directory, telemetry_filename)
                        with open(telemetry_file_path, "ab") as telem_file:
                            telem_file.write(temp)
                        pass

                    case 0xAB:  #ping with status
                        powerStatusRx = new_data[6]
                        ADCSStatusRx = new_data[7]
                        CAMERACCLSMRx = new_data[8]
                        CDHCCLSMRx = new_data[9]
                        ADCSCCLSMRx = new_data[10]
                        power_status = RED if powerStatusRx == 0 else GREEN
                        adcs_status = RED if ADCSStatusRx == 0 else GREEN
                        indicators["CAMERA_CCLSM"].update(RED if CAMERACCLSMRx == 0 else GREEN)
                        pass
                    case 0x75:  #CCLSM info
                        if(new_data[continuedIndex] == 1):  #this packet is split into multiple packets
                            buildUpData.append(new_data[dataIndex:crcIndex])
                        else:
                            buildUpData.append(new_data[dataIndex:crcIndex])
                            print(buildUpData)
                        pass
                    case 0x99:  #imageData
                        imageData.append(new_data[dataIndex:crcIndex])
                        combined_image_data = b''.join(imageData)
                        file_path = os.path.join(image_save_directory, filename)
                        with open(file_path, "wb") as img_file:
                            img_file.write(combined_image_data)
                        pass
                    case 0x88:  #image start info
                        print("Image status RX!!")
                        imageData = []
                        filename = "received_image_" + time.strftime("%Y%m%d_%H%M%S") + ".jpg"
                        imageSizeBytes = new_data[6:10]
                        pass
                    case 0x89:  #image end info
                        print("Image RX done!!")
                        pass
                    case 0x45:  #power telemetry values

                        pass
                    case _:
                        pass
        except Exception as e:
            print(e)
        new_data = []

    # Draw everything
    window.fill(windowcol)
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
        elif button.group == "BALLIN":
            button.update(comms_status)
        elif button.group == "CCLSM":
            button.update(power_status)

    for indicator in indicators.values():
        indicator.draw(window)

    for label in labels:
        label.draw(window)
    textbox.draw(window)

    toc1 = time.time()
    if (toc1 - tic1 > 0.25):
        update_oribt_animation()
        tic1 = time.time()
    window.blit(surf, (25, height - size[1] + 6))

    # Ground pass auto telemetry downlink
    auto_telemetry_downlink()

    # Update dropdown menu for serial connections
    draw_com_port_dropdown()
    draw_ps_com_port_dropdown()
    connect_serial_button.draw(window)
    connect_ps_button.draw(window)
    orbit_sim_control_button.draw(window)

    drawLoopRate(set_GUI_fps, current_fps)

    # drawTextToScreen(received_data)
    drawVoltageAndCurrent(voltage, current_list)
    updateStatus()
    draw_image_black_box_outline()
    draw_ADCS_buttons_black_box_outline()
    draw_Deployables_buttons_black_box_outline()
    draw_Power_buttons_black_box_outline()
    draw_Communication_Status_black_box_outline()
    draw_Power_Consumption_status_black_box_outline()
    try:
        received_image = pygame.image.load(file_path)
        received_image = pygame.transform.scale(received_image, (640/2, 480/2))
        image = pygame.transform.rotate(received_image, 90)
        window.blit(image, (950, 480))
    except:
        pass

    pygame.display.flip()

running = False
serialRX_thread.join()
ps_status_thread.join()
serial_check_thread.join()

# Clean up
if ser:
    ser.close()
if ps:
    ps.close()
pygame.quit()