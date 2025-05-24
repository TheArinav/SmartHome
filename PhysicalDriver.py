# PhysicalDriver.py

import RPi.GPIO as GPIO
import time
import subprocess

gpio_mode_set = False

#region PINS

OUT_PIN_LED_BLUE   = 14
OUT_PIN_LED_GREEN  = 15
OUT_PIN_LED_YELLOW = 18
OUT_PIN_LED_ORANGE = 23
OUT_PIN_LED_RED    = 24
OUT_PIN_BTTN_ROW_1 = 25
OUT_PIN_BTTN_ROW_2 = 8
OUT_PIN_BTTN_ROW_3 = 7
OUT_PIN_BTTN_ROW_4 = 1
OUT_PIN_SPEAKER    = 12
OUT_PIN_LED_IR     = 16
IN_PIN_IR          = 20
IN_PIN_BTTN_COL_1  = 21
IN_PIN_BTTN_COL_2  = 7
IN_PIN_BTTN_COL_3  = 17
IN_PIN_SDA         = 2
IN_PIN_SCL         = 3

#endregion

#region GENERAL

# --- Setup phase ---
def setup_gpio():
    global gpio_mode_set
    if gpio_mode_set:
        return

    GPIO.setmode(GPIO.BCM)

    # Output pins
    led_pins = [OUT_PIN_LED_BLUE, OUT_PIN_LED_GREEN, OUT_PIN_LED_YELLOW,
                OUT_PIN_LED_ORANGE, OUT_PIN_LED_RED, OUT_PIN_SPEAKER, OUT_PIN_LED_IR]
    for pin in led_pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

    gpio_mode_set = True
    print("[GPIO] Initialized")

# --- IR signal sending (placeholder using irsend) ---
def send_ir_signal(code: str):
    print(f"[IR] Sending IR code: {code}")
    try:
        subprocess.run(["irsend", "SEND_ONCE", "custom_remote", code], check=True)
    except Exception as e:
        print(f"[IR ERROR] Failed to send code {code}: {e}")

# --- LED control ---
def set_led(color: str):
    setup_gpio()
    # First, turn off all LEDs
    for pin in [OUT_PIN_LED_RED, OUT_PIN_LED_ORANGE, OUT_PIN_LED_YELLOW,
                OUT_PIN_LED_GREEN, OUT_PIN_LED_BLUE]:
        GPIO.output(pin, GPIO.LOW)

    pin_map = {
        "red": OUT_PIN_LED_RED,
        "orange": OUT_PIN_LED_ORANGE,
        "yellow": OUT_PIN_LED_YELLOW,
        "green": OUT_PIN_LED_GREEN,
        "blue": OUT_PIN_LED_BLUE
    }

    pin = pin_map.get(color.lower())
    if pin:
        GPIO.output(pin, GPIO.HIGH)
    else:
        print(f"[LED] Unknown color: {color}")

# --- Buzzer via PWM ---
def play_buzzer(sound_type: str):
    setup_gpio()

    pwm = GPIO.PWM(OUT_PIN_SPEAKER, 1000)  # 1kHz base frequency
    pwm.start(50)  # 50% duty cycle

    duration = 0.2 if sound_type == "short" else 0.6
    time.sleep(duration)

    pwm.stop()
    GPIO.output(OUT_PIN_SPEAKER, GPIO.LOW)

# --- Display text via LCD_I2C ---
def display_text(text: str):
    setup_gpio()
    if not hasattr(display_text, "initialized"):
        lcd_init()
        display_text.initialized = True

    lines = text.split("|")
    lcd_write(lines[0], 1)
    if len(lines) > 1:
        lcd_write(lines[1], 2)

# --- Action dispatcher ---
def perform_action(action: str):
    setup_gpio()

    try:
        kind, value = action.split(":", 1)
    except ValueError:
        print(f"[ERROR] Invalid action format: {action}")
        return

    if kind == "ir":
        send_ir_signal(value)
    elif kind == "display":
        display_text(value)
    elif kind == "led":
        set_led(value)
    elif kind == "buzzer":
        play_buzzer(value)
    else:
        print(f"[WARN] Unknown action type: {kind}")

#endregion

#region LCD
from smbus2 import SMBus
import time

I2C_LCD_ADDR = 0x27
LCD_WIDTH = 16
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0

LCD_BACKLIGHT = 0x08
ENABLE = 0b00000100

bus = SMBus(1)  # I2C-1 on Raspberry Pi

def lcd_byte(bits, mode):
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

    bus.write_byte(I2C_LCD_ADDR, bits_high)
    lcd_toggle_enable(bits_high)

    bus.write_byte(I2C_LCD_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    time.sleep(0.0005)
    bus.write_byte(I2C_LCD_ADDR, (bits | ENABLE))
    time.sleep(0.0005)
    bus.write_byte(I2C_LCD_ADDR, (bits & ~ENABLE))
    time.sleep(0.0005)

def lcd_init():
    lcd_byte(0x33, 0)
    lcd_byte(0x32, 0)
    lcd_byte(0x06, 0)
    lcd_byte(0x0C, 0)
    lcd_byte(0x28, 0)
    lcd_byte(0x01, 0)
    time.sleep(0.005)
    print("[LCD] Initialized")

def lcd_write(message, line=1):
    if line == 1:
        lcd_byte(LCD_LINE_1, 0)
    else:
        lcd_byte(LCD_LINE_2, 0)

    message = message.ljust(LCD_WIDTH)
    for char in message:
        lcd_byte(ord(char), 1)

#endregion

#region BUTTONS
# --- Button Matrix Scanning ---
button_layout = [
    ['1', '2', '3'],
    ['4', '5', '6'],
    ['7', '8', '9'],
    ['*', '0', '#']
]

button_row_pins = [OUT_PIN_BTTN_ROW_1, OUT_PIN_BTTN_ROW_2, OUT_PIN_BTTN_ROW_3, OUT_PIN_BTTN_ROW_4]
button_col_pins = [IN_PIN_BTTN_COL_1, IN_PIN_BTTN_COL_2, IN_PIN_BTTN_COL_3]

def init_button_matrix():
    setup_gpio()
    for row in button_row_pins:
        GPIO.setup(row, GPIO.OUT)
        GPIO.output(row, GPIO.LOW)

    for col in button_col_pins:
        GPIO.setup(col, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def scan_buttons():
    """Returns a list of pressed button symbols, or empty list."""
    pressed = []
    for row_index, row_pin in enumerate(button_row_pins):
        GPIO.output(row_pin, GPIO.HIGH)
        for col_index, col_pin in enumerate(button_col_pins):
            if GPIO.input(col_pin) == GPIO.HIGH:
                pressed.append(button_layout[row_index][col_index])
        GPIO.output(row_pin, GPIO.LOW)
    return pressed

def wait_for_button(timeout=None):
    """Blocks until a button is pressed (or timeout in seconds). Returns the symbol."""
    print("[BTN] Waiting for button press...")
    init_button_matrix()
    start = time.time()
    while True:
        buttons = scan_buttons()
        if buttons:
            print(f"[BTN] Detected: {buttons}")
            return buttons[0]  # Only return the first for now
        if timeout and (time.time() - start) > timeout:
            print("[BTN] Timeout")
            return None
        time.sleep(0.05)

#endregion

#region IR_IN
def init_ir_receiver():
    setup_gpio()
    GPIO.setup(IN_PIN_IR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print("[IR_RX] Ready")

def wait_for_ir_signal(timeout=None):
    """Blocks until IR signal is received or timeout. Returns placeholder IR string."""
    init_ir_receiver()
    print("[IR_RX] Waiting for signal...")

    start_time = time.time()

    while True:
        if GPIO.input(IN_PIN_IR) == GPIO.LOW:
            # Placeholder: real decoding should replace this
            print("[IR_RX] Signal detected!")
            time.sleep(0.2)  # debounce delay
            return "0xDEADBEEF"  # placeholder IR code
        if timeout and (time.time() - start_time) > timeout:
            print("[IR_RX] Timeout")
            return None
        time.sleep(0.01)

#endregion