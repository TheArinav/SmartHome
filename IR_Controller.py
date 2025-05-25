from PhysicalDriver import (
    perform_action,
    wait_for_button,
    wait_for_ir_signal,
    init_button_matrix,
    setup_gpio,
    lcd_init,
    send_ir_thread,
    receive_ir_thread,
    button_scan_thread
)

import time
import threading
import socket

# DFA Definition
DFA = {
    "idle": {
        "on_enter": ["display:Idle", "led:green"],
        "transitions": {
            "Button:2": "turn_on_tv",
            "Remote:Power": "turn_on_ac",
            "IR_Received:0x00FF30CF": "handle_tv_remote"
        }
    },
    "turn_on_tv": {
        "on_enter": ["ir:0xAABBCCDD", "display:TV ON", "led:blue", "buzzer:short"],
        "transitions": {
            "Button:#": "idle",
            "Remote:Back" : "idle"
        }
    },
    "turn_on_ac": {
        "on_enter": ["ir:0x11223344", "display:AC ON", "led:orange", "buzzer:short"],
        "transitions": {
            "Button:3": "idle",
            "Remote:Back": "idle"
        }
    },
    "handle_tv_remote": {
        "on_enter": ["display:Remote Control", "led:yellow"],
        "transitions": {
            "Button:0": "idle",
            "Remote:Back" : "idle"
        }
    }
}

current_state = "idle"

import json

def handle_command(message: str) -> str:
    global current_state, DFA

    if message.startswith("Remote:"):
        event = message
        print(f"[DFA] Current state: {current_state} | Event: {event}")
        state_def = DFA.get(current_state)
        if not state_def:
            return f"[ERROR] Undefined state: {current_state}"
        next_state = resolve_event_trigger(state_def, event)
        if not next_state:
            return f"[DFA] No transition for event '{event}' in state '{current_state}'"
        print(f"[DFA] Transition: {current_state} → {next_state}")
        current_state = next_state
        execute_state(current_state)
        return f"[DFA] State changed to {current_state}"

    else:
        try:
            new_dfa = json.loads(message)
            if not isinstance(new_dfa, dict):
                return "[ERROR] Parsed DFA is not a dictionary."
            DFA = new_dfa
            current_state = list(DFA.keys())[0]  # Use the first key as initial state
            print("[DFA] New DFA loaded.")
            execute_state(current_state)
            return f"[DFA] New DFA loaded. Initial state: {current_state}"
        except Exception as e:
            return f"[ERROR] Failed to load DFA: {e}"


def resolve_event_trigger(state_def, event):
    transitions = state_def.get("transitions", {})
    if event in transitions:
        return transitions[event]
    for trigger_key in transitions:
        if trigger_key.endswith(":*"):
            prefix = trigger_key[:-2]
            if event.startswith(prefix + ":"):
                return transitions[trigger_key]
    return None

def execute_state(state_name: str):
    state = DFA.get(state_name)
    if not state:
        print(f"[DFA] Invalid state: {state_name}")
        return
    actions = state.get("on_enter", [])
    print(f"[DFA] Executing {len(actions)} actions for state '{state_name}'")
    for action in actions:
        perform_action(action)

# Socket server logic
HOST = '0.0.0.0'
PORT = 5000

def handle_client(conn, addr):
    print(f"[CONNECTED] {addr}")
    try:
        data = conn.recv(1024).decode().strip()
        if data:
            print(f"[COMMAND] Received from {addr}: {data}")
            response = handle_command(data)
            conn.sendall(response.encode())
        else:
            conn.sendall(b"Invalid command.")
    except Exception as e:
        print(f"[ERROR] {e}")
        conn.sendall(b"Error occurred.")
    finally:
        conn.close()

def start_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen()
    print(f"[LISTENING] Server running on {HOST}:{PORT}")
    while True:
        conn, addr = server.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

def start_dfa_loop():
    # Start all threads
    threading.Thread(target=send_ir_thread, daemon=True).start()
    threading.Thread(target=receive_ir_thread, daemon=True).start()
    threading.Thread(target=button_scan_thread, daemon=True).start()
    threading.Thread(target=start_server, daemon=True).start()

    execute_state(current_state)  # Initial state entry
    while True:
        button = wait_for_button(timeout=0.1)
        if button:
            handle_command(f"Button:{button}")
            continue
        ir_signal = wait_for_ir_signal(timeout=0.1)
        if ir_signal:
            handle_command(f"IR_Received:{ir_signal}")
        time.sleep(0.05)

if __name__ == "__main__":
    setup_gpio()
    lcd_init()
    init_button_matrix()
    start_dfa_loop()