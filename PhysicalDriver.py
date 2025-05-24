import RPi.GPIO as GPIO
import pigpio
import time
import threading
from smbus2 import SMBus
from queue import Queue, Empty
from collections import namedtuple

gpio_mode_set = False
pi = pigpio.pi()

# --- PINS ---
OUT_PIN_LED_BLUE = 27
OUT_PIN_LED_GREEN = 22
OUT_PIN_LED_YELLOW = 18
OUT_PIN_LED_ORANGE = 23
OUT_PIN_LED_RED = 24
OUT_PIN_BTTN_ROW_1 = 25
OUT_PIN_BTTN_ROW_2 = 8
OUT_PIN_BTTN_ROW_3 = 7
OUT_PIN_BTTN_ROW_4 = 26
OUT_PIN_SPEAKER = 12
OUT_PIN_LED_IR = 16
IN_PIN_IR = 20
IN_PIN_BTTN_COL_1 = 21
IN_PIN_BTTN_COL_2 = 4
IN_PIN_BTTN_COL_3 = 17
IN_PIN_SDA = 2
IN_PIN_SCL = 3

ir_input_queue = Queue()
ir_output_queue = Queue()
button_event_queue = Queue()


def setup_gpio():
    global gpio_mode_set
    if gpio_mode_set:
        return
    GPIO.setmode(GPIO.BCM)
    for pin in [OUT_PIN_LED_BLUE, OUT_PIN_LED_GREEN, OUT_PIN_LED_YELLOW,
                OUT_PIN_LED_ORANGE, OUT_PIN_LED_RED, OUT_PIN_SPEAKER]:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    gpio_mode_set = True
    print("[GPIO] Initialized")


def nec_encode(code):
    def pulse(on, duration):
        return pigpio.pulse(
            1 << OUT_PIN_LED_IR if on else 0,
            0 if on else 1 << OUT_PIN_LED_IR,
            duration
        )

    seq = [pulse(True, 9000), pulse(False, 4500)]
    for i in range(32):
        bit = (code >> (31 - i)) & 1
        seq.append(pulse(True, 560))
        seq.append(pulse(False, 1690 if bit else 560))
    seq.append(pulse(True, 560))
    return seq


def send_ir_thread():
    while True:
        try:
            code_hex = ir_output_queue.get(timeout=0.5)
            if not pi.connected:
                print("[IR_TX] pigpio not connected")
                continue
            try:
                code = int(code_hex, 16)
            except ValueError:
                print(f"[IR ERROR] Invalid hex code: {code_hex}")
                continue
            print(f"[IR_TX] Sending NEC code: {code_hex}")
            pi.set_mode(OUT_PIN_LED_IR, pigpio.OUTPUT)
            pi.write(OUT_PIN_LED_IR, 0)
            pi.wave_clear()
            pi.wave_add_generic(nec_encode(code))
            wid = pi.wave_create()
            if wid >= 0:
                pi.wave_send_once(wid)
                while pi.wave_tx_busy():
                    time.sleep(0.01)
                pi.wave_delete(wid)
            pi.write(OUT_PIN_LED_IR, 0)
        except Empty:
            continue


def receive_ir_thread():
    while True:
        if not pi.connected:
            time.sleep(1)
            continue
        pulses = []

        def cb_func(gpio, level, tick):
            nonlocal pulses
            if len(pulses) >= 1000:
                return
            pulses.append((level, tick))

        cb = pi.callback(IN_PIN_IR, pigpio.EITHER_EDGE, cb_func)
        time.sleep(0.5)
        cb.cancel()
        if len(pulses) < 2:
            continue
        durations = [(pulses[i - 1][0], pigpio.tickDiff(pulses[i - 1][1], pulses[i][1]))
                     for i in range(1, len(pulses))]
        bits = []
        i = 0
        while i < len(durations) and durations[i][1] < 8500:
            i += 1
        i += 2
        while i + 1 < len(durations) and len(bits) < 32:
            mark, space = durations[i][1], durations[i + 1][1]
            if 400 <= mark <= 700:
                if 400 <= space <= 700:
                    bits.append("0")
                elif 1500 <= space <= 1800:
                    bits.append("1")
                else:
                    break
            i += 2
        if len(bits) == 32:
            bit_str = "".join(bits)
            value = int(bit_str, 2)
            hex_code = f"0x{value:08X}"
            print(f"[IR_RX] NEC code received: {hex_code}")
            ir_input_queue.put(hex_code)


button_layout = [['1', '2', '3'], ['4', '5', '6'], ['7', '8', '9'], ['*', '0', '#']]
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
    pressed = []
    for row_index, row_pin in enumerate(button_row_pins):
        GPIO.output(row_pin, GPIO.HIGH)
        for col_index, col_pin in enumerate(button_col_pins):
            if GPIO.input(col_pin) == GPIO.HIGH:
                pressed.append(button_layout[row_index][col_index])
        GPIO.output(row_pin, GPIO.LOW)
    return pressed


def button_scan_thread():
    init_button_matrix()
    while True:
        result = scan_buttons()
        if result:
            button_event_queue.put(result[0])
            print(f"[BTN] Detected: {result[0]}")
            time.sleep(0.3)
        time.sleep(0.05)


def perform_action(action):
    setup_gpio()
    try:
        kind, value = action.split(":", 1)
    except ValueError:
        print(f"[ERROR] Invalid action format: {action}")
        return
    if kind == "ir":
        ir_output_queue.put(value)
    elif kind == "display":
        display_text(value)
    elif kind == "led":
        set_led(value)
    elif kind == "buzzer":
        play_buzzer(value)
    else:
        print(f"[WARN] Unknown action type: {kind}")

# --- LED Control ---
def set_led(color):
    setup_gpio()
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
    if color.lower() in pin_map:
        GPIO.output(pin_map[color.lower()], GPIO.HIGH)
    else:
        print(f"[LED] Unknown color: {color}")

# --- Buzzer Control ---
def play_buzzer(sound_type):
    setup_gpio()
    pwm = GPIO.PWM(OUT_PIN_SPEAKER, 1000)
    pwm.start(50)
    time.sleep(0.2 if sound_type == "short" else 0.6)
    pwm.stop()
    GPIO.output(OUT_PIN_SPEAKER, GPIO.LOW)

# --- LCD Display ---
I2C_LCD_ADDR = 0x27
LCD_WIDTH = 16
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
LCD_BACKLIGHT = 0x08
ENABLE = 0b00000100
bus = SMBus(1)

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

def display_text(text):
    setup_gpio()
    if not hasattr(display_text, "initialized"):
        lcd_init()
        display_text.initialized = True
    lines = text.split("|")
    lcd_write(lines[0], 1)
    if len(lines) > 1:
        lcd_write(lines[1], 2)

def lcd_write(message, line=1):
    lcd_byte(LCD_LINE_1 if line == 1 else LCD_LINE_2, 0)
    message = message.ljust(LCD_WIDTH)
    for char in message:
        lcd_byte(ord(char), 1)