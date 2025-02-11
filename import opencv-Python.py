import pygame
import serial
import gif_pygame
import time
import string

# Initialize Pygame
pygame.init()

indicator_corner_anchor_x = 50
indicator_corner_anchor_y = 500

voltage_anchor_x = 200
current_anchor_x = 200
voltage_anchor_y = 500
current_anchor_y = 550

# Set up the display
width, height = 1000, 600

pygame.mixer.init()
pygame.display.set_caption("HAXSat")
window = pygame.display.set_mode((width, height))
font = pygame.font.Font(None, 20)
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
OFF_RED = (250, 0, 0)
ORANGE = (252, 186, 3)
PURPLE = (100, 0, 125)

# Set up the serial connection
ser = serial.Serial('COM7', 115200)  # Change 'COM3' to your COM port
ser.timeout = 0.05

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
power_status = GREEN
adcs_status = GREEN
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
    Button(800, 110, 100, 50, GREY, bytearray(b'\x01\x0E\x00'), "Reset GNSS", None, "ADCS"),
    Button(800, 170, 100, 50, GREY, bytearray(b'\x01\x11\x06'), "Get data rate", None, "ADCS"),
    Button(800, 230, 50, 50, GREEN, bytearray(b'\x00\x04\x31\x01'), "ON", None, "POWER"),
    Button(850, 230, 50, 50, OFF_RED, bytearray(b'\x00\x04\x31\x00'), "OFF", None, "POWER"),
    Button(900, 230, 50, 50, GREY, bytearray(b'\x00\x04\x31\x03'), "DATA", None, "POWER"),
    Button(800, 290, 50, 50, GREEN, bytearray(b'\x00\x04\x30\x01'), "ON", None, "POWER"),
    Button(850, 290, 50, 50, OFF_RED, bytearray(b'\x00\x04\x30\x00'), "OFF", None, "POWER"),
    Button(900, 290, 50, 50, GREY, bytearray(b'\x00\x04\x30\x02'), "DATA", None, "POWER"),
    Button(800, 350, 50, 50, GREEN, bytearray(b'\x00\x04\x03\x01'), "ON", None, "POWER"),
    Button(850, 350, 50, 50, OFF_RED, bytearray(b'\x00\x04\x03\x00'), "OFF", None, "POWER"),
    Button(900, 350, 50, 50, GREY, bytearray(b'\x00\x04\x31\x02'), "DATA", None, "POWER"),
    Button(800, 410, 50, 50, GREEN, bytearray(b'\x00\x04\x03\x01'), "ON", None, "POWER"),
    Button(850, 410, 50, 50, OFF_RED, bytearray(b'\x00\x04\x03\x00'), "OFF", None, "POWER"),
    Button(900, 410, 50, 50, GREY, bytearray(b'\x00\x04\x31\x02'), "DATA", None, "POWER"),
]

def crc32b(message):
    crc = 0xFFFFFFFF
    for byte in message:
        crc = crc ^ byte
        for _ in range(8):
            mask = -(crc & 1)
            crc = (crc >> 1) ^ (0xEDB88320 & mask)
    return ~crc & 0xFFFFFFFF

buildUpData = []
imageData = []
expectedIndex = 0

while running:    
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            for button in buttons:
                if button.is_clicked(event.pos):
                    button.on_click()
        elif event.type == pygame.MOUSEBUTTONUP:
            for button in buttons:
                if button.is_clicked(event.pos):
                    button.off_click()

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
        ser.write(bytearray(b'\xAB'))
        tic = time.time()
    
    #if CDH hasn't responded in a while, change status to red
    if misses > 60:
        comms_status = RED
        windowcol = RED
    else:
        comms_status = GREEN
        windowcol = WHITE
    
    # Read serial data
    if ser.in_waiting >= 74:
        try:
            new_data = ser.read_all()
            misses = 0
            if(new_data[0] == 0xAA):
                type = new_data[1]
                len = new_data[2]
                rxCRC = new_data[76-5:76-1]
                crc = crc32b(new_data[0:67])
                index = (new_data[3] << 8) | new_data[4] 
                match type:
                    case 0x05: #powinfo
                        stringData = new_data[6:6+len-1]
                        stringData = stringData.decode('utf-8')
                        stringData = stringData.split(' ')
                        voltage = stringData[0]
                        current = stringData[1]
                        pass
                    case 0x0A: #printable
                        stringData = new_data[6:6+len-1]
                        received_data.append(stringData.decode('utf-8'))
                        pass
                    case 0x10: #raw hex data to print
                        hexString = ""
                        temp = new_data[6:6+len-1]
                        for char in temp:
                            hexString = hexString + hex(char) + " "
                        hexString = hexString + "\n"
                        received_data.append(hexString)
                        pass
                    case 0xAB: #ping with status
                        powerStatusRx = new_data[6]
                        ADCSStatusRx = new_data[7]
                        power_status = RED if powerStatusRx == 0 else GREEN
                        adcs_status = RED if ADCSStatusRx == 0 else GREEN
                        pass
                    case 0x75: #CCLSM info
                        if(new_data[continuedIndex] == 1): #this packet is split into multiple packets
                            buildUpData.append(new_data[dataIndex:crcIndex])
                        else:
                            buildUpData.append(new_data[dataIndex:crcIndex])
                            print(buildUpData)
                        pass
                    case 0x99: #imageData
                        if(index == expectedIndex):
                            imageData = new_data[dataIndex:crcIndex]
                            expectedIndex = expectedIndex + 1
                            sendAck()
                        else:
                            print("image data out of order")    
                            sendNack()
                        pass
                    case _:
                        pass
        except Exception as e:
            print(e)

    # Draw everything
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

    drawTextToScreen(received_data)
    drawVoltageAndCurrent(voltage, current)
    updateStatus()
    pygame.display.flip()

# Clean up
ser.close()
pygame.quit()
