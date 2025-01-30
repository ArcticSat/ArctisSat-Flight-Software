import pygame
import serial
import gif_pygame
import time

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
OFF_RED = (250, 0, 0)
ORANGE = (252, 186, 3)
PURPLE = (100, 0, 125)

# Set up the serial connection
ser = serial.Serial('COM7', 115200)  # Change 'COM3' to your COM port
ser.timeout = 0.5

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
    def __init__(self, x, y, width, height, color, message, text, action = None, group = None):
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
            if(self.color != RED):    
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
    Button(800, 230, 50, 50, GREEN, bytearray(b'\x00\x04\x31\x01'), "ON"),
    Button(850, 230, 50, 50, OFF_RED, bytearray(b'\x00\x04\x31\x00'), "OFF"),
    Button(900, 230, 50, 50, GREY, bytearray(b'\x00\x04\x31\x03'), "DATA"),
    Button(800, 290, 50, 50, GREEN, bytearray(b'\x00\x04\x30\x01'), "ON"),
    Button(850, 290, 50, 50, OFF_RED, bytearray(b'\x00\x04\x30\x00'), "OFF"),
    Button(900, 290, 50, 50, GREY, bytearray(b'\x00\x04\x30\x02'), "DATA"),
    Button(800, 350, 50, 50, GREEN, bytearray(b'\x00\x04\x03\x01'), "ON"),
    Button(850, 350, 50, 50, OFF_RED, bytearray(b'\x00\x04\x03\x00'), "OFF"),
    Button(900, 350, 50, 50, GREY, bytearray(b'\x00\x04\x31\x02'), "DATA"),
    Button(800, 410, 50, 50, GREEN, bytearray(b'\x00\x04\x03\x01'), "ON"),
    Button(850, 410, 50, 50, OFF_RED, bytearray(b'\x00\x04\x03\x00'), "OFF"),
    Button(900, 410, 50, 50, GREY, bytearray(b'\x00\x04\x31\x02'), "DATA"),
]

# Main loop
running = True
# Text window to display serial data
text_window = pygame.Rect(50, 250, 380, 200)
received_data = []
max_lines = 10

example_gif = gif_pygame.load("cat.gif") # Loads a .gif file

clock = pygame.Clock()

# Indicators for power and ADCS status
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

while running:    

    if(armCount > 0):
        actuatorColor = GREEN
    else:
        actuatorColor = RED

    toc = time.time()
    if(toc - tic > 0.5):
        ser.write(bytearray(b'\xAB'))
        tic = time.time()
        misses = misses + 1

    armCount = armCount - 1
    if(armCount < 0):
        armCount = 0
    
    clock.tick(30)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            for button in buttons:
                if button.is_clicked(event.pos):
                    button.on_click()

    if misses > 50:
        comms_status = RED
        windowcol = RED
        playOnce = False
    else:
        # if not playOnce:
        #     playOnce = True
        #     sound1.play()
        #     time.sleep(5)
        #     sound1.stop()
        windowcol = WHITE
        comms_status = GREEN

    # Read serial data
    if ser.in_waiting > 0:
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
            elif new_data ==  "0xAB":
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
        misses = misses + 1

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
    
    draw_labels()



    # Draw indicators
    pygame.draw.rect(window, power_status, power_indicator)
    pygame.draw.rect(window, adcs_status, adcs_indicator)
    pygame.draw.rect(window, comms_status, comms_indicator)

    pygame.display.flip()

# Clean up
ser.close()
pygame.quit()
