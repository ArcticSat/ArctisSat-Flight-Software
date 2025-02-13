import datetime
import pygame
import serial
import gif_pygame
import time
import string
import threading

import os

# Initialize Pygame
pygame.init()

indicator_corner_anchor_x = 50
indicator_corner_anchor_y = 500

voltage_anchor_x = 200
current_anchor_x = 200
voltage_anchor_y = 500
current_anchor_y = 550

# Set up the display
width, height = 1500, 900

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

pygame.mixer.init()
pygame.display.set_caption("HAXSat")
window = pygame.display.set_mode((width, height))
font = pygame.font.Font(None, 20)
small_font = pygame.font.Font(None, 12)

text_window = pygame.Rect(50, 250, 380, 200)
tic = time.time()
clock = pygame.Clock()
example_gif = gif_pygame.load("cat.gif") # Loads a .gif file

sound1 = pygame.mixer.Sound('catsong.mp3')  # Load a sound.

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
OFF_RED = (250, 0, 0)
ORANGE = (252, 186, 3)
PURPLE = (100, 0, 125)

# Set up the serial connection
ser = serial.Serial('COM7', 115200)  # Change 'COM3' to your COM port
ser.timeout = 0.5

#actuator arming logic variables
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
    window.blit(power_label, (indicator_corner_anchor_x + 25, indicator_corner_anchor_y))
    window.blit(adcs_label, (indicator_corner_anchor_x + 25, indicator_corner_anchor_y + 25))
    window.blit(comms_label, (indicator_corner_anchor_x + 25, indicator_corner_anchor_y + 50))
    pygame.draw.rect(window, power_status, power_indicator)
    pygame.draw.rect(window, adcs_status, adcs_indicator)
    pygame.draw.rect(window, comms_status, comms_indicator)

def drawVoltageAndCurrent(voltage, current):
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
        self.clearbox = Checkbox(x + width - 20, y+20, 20, "Clear", font)

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
        if(self.clearbox.checked):
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
    Button(50, 20, 100, 20, RED, "bruh", "ARM", armActuators, "ARMING"),
    Button(50, 50, 100, 50, RED, bytearray(b'\x01\x08\x00'), "Left Out", None, "ACTUATORS"),
    Button(50, 110, 100, 50, GREEN, bytearray(b'\x01\x0A\x00'), "Left In", None, "ACTUATORS"),
    Button(350, 50, 100, 50, RED, bytearray(b'\x01\x0C\x00'), "Both Out", None, "ACTUATORS"),
    Button(200, 50, 100, 50, RED, bytearray(b'\x01\x09\x00'), "Right Out", None, "ACTUATORS"),
    Button(200, 110, 100, 50, GREEN, bytearray(b'\x01\x0B\x00'), "Right In", None, "ACTUATORS"),
    Button(350, 110, 100, 50, GREEN, bytearray(b'\x01\x0D\x00'), "Both In", None, "ACTUATORS"),
    Button(50, 170, 100, 50, PURPLE, bytearray(b'\x00\x00'), "Ping Power", None, "POWER"),
    Button(200, 170, 100, 50, PURPLE, bytearray(b'\x00\x01\x01'), "Get V and C", None, "POWER"),
    Button(350, 170, 100, 50, GREY, bytearray(b'\x01\x01\x02'), "Sync", None, "ADCS"),
    Button(500, 50, 100, 50, GREY, bytearray(b'\x01\x02\x0D'), "Gyro", None, "ADCS"),
    Button(500, 110, 100, 50, GREY, bytearray(b'\x01\x03\x0B'), "Mag", None, "ADCS"),
    Button(500, 170, 100, 50, GREY, bytearray(b'\x01\x07\x03'), "Temp", None, "ADCS"),
    Button(650, 50, 100, 50, GREY, bytearray(b'\x01\x04\x15'), "Lat/Long", None, "ADCS"),
    Button(650, 110, 100, 50, GREY, bytearray(b'\x01\x05\x11'), "Time", None, "ADCS"),
    Button(650, 170, 100, 50, GREY, bytearray(b'\x01\x06\x05'), "Num Sats", None, "ADCS"),
    Button(800, 110, 100, 50, GREY, bytearray(b'\x01\x11\x00'), "Reset GNSS", None, "ADCS"),
    Button(800, 170, 100, 50, GREY, bytearray(b'\x01\x12\x00'), "Get data rate", None, "ADCS"),
    Button(800, 230, 50, 50, GREY, bytearray(b'\x00\x04\x01\x01'), "ON", None, "CCLSM"),
    Button(850, 230, 50, 50, GREY, bytearray(b'\x00\x04\x01\x00'), "OFF", None, "CCLSM"),
    Button(900, 230, 50, 50, GREY, bytearray(b'\x00\x04\x01\x03'), "DATA", None, "CCLSM"),
    Button(800, 290, 50, 50, GREY, bytearray(b'\x00\x04\x00\x01'), "ON", None, "CCLSM"),
    Button(850, 290, 50, 50, GREY, bytearray(b'\x00\x04\x00\x00'), "OFF", None, "CCLSM"),
    Button(900, 290, 50, 50, GREY, bytearray(b'\x00\x04\x00\x02'), "DATA", None, "CCLSM"),
    Button(800, 350, 50, 50, GREY, bytearray(b'\x00\x04\x03\x01'), "ON", None, "CCLSM"),
    Button(850, 350, 50, 50, GREY, bytearray(b'\x00\x04\x03\x00'), "OFF", None, "CCLSM"),
    Button(900, 350, 50, 50, GREY, bytearray(b'\x00\x04\x03\x02'), "DATA", None, "CCLSM"),
    Button(800, 410, 50, 50, GREY, bytearray(b'\x01\x12\x00'), "ON", None, "CCLSM"),
    Button(850, 410, 50, 50, GREY, bytearray(b'\x01\x11\x00'), "OFF", None, "CCLSM"),
    # Button(900, 410, 50, 50, GREY, bytearray(b'\x00\x04\x31\x02'), "DATA", None, "CCLSM"),
    Button(1050, 410, 75, 50, GREEN, bytearray(b'\x02\x01'), "Take\nImage", None, "BALLIN"),
    Button(1200, 410, 75, 50, GREEN, bytearray(b'\x02\x02'), "Downlink\nImage", None, "BALLIN"),
    Button(1050, 470, 75, 50, GREEN, bytearray(b'\x02\x05'), "Downlink\nImage", None, "BALLIN"),
    Button(1200, 470, 75, 50, GREEN, bytearray(b'\x02\x05'), "Request\ntelemetry", None, "BALLIN"),
    Button(1050, 530, 75, 50, GREEN, bytearray(b'\x02\x05'), "File system\nstatus", None, "BALLIN"),
    Button(1200, 530, 75, 50, GREEN, bytearray(b'\x02\x05'), "More\nbuttons!", None, "BALLIN"),
    Button(1050, 590, 75, 50, GREEN, bytearray(b'\x02\x05'), "Even\nmore!", None, "BALLIN"),
    Button(1200, 590, 75, 50, GREEN, bytearray(b'\x02\x05'), "Big\nswag!!", None, "BALLIN"),
]

indicators = {
    "CDH_CCLSM":indicator(800, 230, RED),
    "ADCS_CCLSM":indicator(800, 290, RED),
    "OTHER_CCLSM":indicator(800, 350, RED),
    "CAMERA_CCLSM":indicator(800, 410, RED),
    # "POWER_STATUS":indicator(800, 410, RED),
    # "ADCS_STATUS":indicator(800, 410, RED),
}

labels = [
    label(760, 260, "CDH\nPOWER", 15),
    label(760, 320, "ADCS\nPOWER", 15),
    label(760, 380, "OTHER\nPOWER", 15),
    label(760, 440, "CAMERA\nPOWER", 15),
    label(70, 240, "Messages from CDH", 15),
]

textbox = ScrollableTextbox(50, 250, 380, 200, WHITE, font)

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
    while(running):
        if ser.in_waiting >= 79:
            new_data = ser.read_all()
            try:
                ser.write(b'\xAA\xBB\xCC')
            except:
                pass

buildUpData = []
imageData = []
expectedIndex = 0
t1 = threading.Thread(target=SerialRx)
t1.start()

filename = "received_image_" + time.strftime("%Y%m%d_%H%M%S") + ".jpg"

while running:    
    clock.tick(60)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            for button in buttons:
                if button.is_clicked(event.pos):
                    button.on_click()
            if textbox.checkbox.is_clicked(event.pos):
                textbox.checkbox.toggle()
            if textbox.clearbox.is_clicked(event.pos):
                textbox.clearbox.toggle()
        elif event.type == pygame.MOUSEBUTTONUP:
            for button in buttons:
                if button.is_clicked(event.pos):
                    button.off_click()
        elif event.type == pygame.MOUSEWHEEL:
            if text_window.collidepoint(pygame.mouse.get_pos()):
                textbox.scroll(event.y)

    #check timeout on arm count
    misses = misses + 1
    armCount = armCount - 1
    if(armCount < 0):
        actuatorColor = RED
        armCount = 0
    else:
        actuatorColor = GREEN

    #ping CDH periodically
    toc = time.time()
    if(toc - tic > 0.25):
        try:
            ser.write(bytearray(b'\xAB'))
        except:
            pass
        tic = time.time()
    
    #if CDH hasn't responded in a while, change status to red
    if misses > 100:
        comms_status = RED
        windowcol = RED
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
                    case 0x05: #powinfo
                        stringData = new_data[6:6+myLen-1]
                        stringData = stringData.decode('utf-8')
                        stringData = stringData.split(' ')
                        voltage = stringData[0]
                        current = stringData[1]
                        bits =stringData[2]
                        CDHCCLSMRx = 1
                        # CAMERACCLSMRx = (int(bits) & 0x02) >> 1
                        ADCSCCLSMRx = (int(bits) & 0x01)
                        indicators["ADCS_CCLSM"].update(RED if ADCSCCLSMRx == 0 else GREEN)
                        indicators["CDH_CCLSM"].update(RED if CDHCCLSMRx == 0 else GREEN)
                        pass
                    case 0x0A: #printable
                        stringData = new_data[6:6+myLen-1]
                        textbox.append_text(stringData.decode('utf-8'))
                        print(stringData.decode('utf-8'))
                        pass
                    case 0x10: #raw hex data to print
                        hexString = ""
                        temp = new_data[6:6+myLen-1]
                        for char in temp:
                            hexString = hexString + hex(char) + " "
                        hexString = hexString + "\n"
                        textbox.append_text(hexString)
                        print(hexString)
                        pass
                    case 0xAB: #ping with status
                        powerStatusRx = new_data[6]
                        ADCSStatusRx = new_data[7]
                        CAMERACCLSMRx = new_data[8]
                        CDHCCLSMRx = new_data[9]
                        ADCSCCLSMRx = new_data[10]
                        power_status = RED if powerStatusRx == 0 else GREEN
                        adcs_status = RED if ADCSStatusRx == 0 else GREEN
                        indicators["CAMERA_CCLSM"].update(RED if CAMERACCLSMRx == 0 else GREEN)
                        pass
                    case 0x75: #CCLSM info
                        if(new_data[continuedIndex] == 1): #this packet is split into multiple packets
                            buildUpData.append(new_data[dataIndex:crcIndex])
                        else:
                            buildUpData.append(new_data[dataIndex:crcIndex])
                            print(buildUpData)
                        pass
                    case 0x99: #imageData
                        imageData.append(new_data[dataIndex:crcIndex])
                        combined_image_data = b''.join(imageData)
                        with open(filename, "wb") as img_file:
                            img_file.write(combined_image_data)
                        pass
                    case 0x88: #image start info
                        print("Image status RX!!")
                        imageData = []
                        filename = "received_image_" + time.strftime("%Y%m%d_%H%M%S") + ".jpg"
                        imageSizeBytes = new_data[6:10]
                        pass
                    case 0x89: #image end info
                        print("Image RX done!!")
                        pass
                    case 0x45: #power telemetry values

                        pass
                    case _:
                        pass
        except Exception as e:
            print(e)
        new_data = []


    # Draw everything
    window.fill(windowcol)
    example_gif.render(window, (480, 230))
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

    for indicator in indicators.values():
        indicator.draw(window)

    for label in labels:
        label.draw(window)
    textbox.draw(window)

    # drawTextToScreen(received_data)
    drawVoltageAndCurrent(voltage, current)
    updateStatus()
    try:
        received_image = pygame.image.load(filename)
        received_image = pygame.transform.scale(received_image, (640/2, 480/2))
        image = pygame.transform.rotate(received_image, -90)
        window.blit(image, (1050, 50))
    except:
        pass
    pygame.display.flip()

t1.join()

# Clean up
ser.close()
pygame.quit()
