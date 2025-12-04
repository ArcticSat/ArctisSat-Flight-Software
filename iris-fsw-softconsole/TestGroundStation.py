import curses
import time
import threading
import queue
import serial

SERIAL_PORT = "COM14"
BAUD_RATE = 115200

rx_queue = queue.Queue()

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)



def sanitize_bytes(data: bytes) -> str:
    """Convert raw bytes into a printable safe string for curses."""
    out = []
    for b in data:
        if b == 0x00:
            out.append("")
        elif 32 <= b <= 126:   # printable ASCII
            out.append(chr(b))
        else:
            out.append(f"\\x{b:02x}")
    return "".join(out)


def serial_reader():
    """Reads raw data from COM14 and sends lines or chunks to UI."""

    buffer = b""
    while True:
        chunk = ser.read(1024)
        if chunk:
            buffer += chunk

            # split on newline if present
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                rx_queue.put(sanitize_bytes(line))

        # If no newline, push partial data occasionally
        if buffer and len(buffer) > 256:
            rx_queue.put(sanitize_bytes(buffer))
            buffer = b""

        time.sleep(0.001)


def main(stdscr):

    curses.curs_set(1)
    stdscr.nodelay(True)
    stdscr.clear()

    max_y, max_x = stdscr.getmaxyx()

    log_win_height = max_y - 3
    log_win = curses.newwin(log_win_height, max_x, 0, 0)
    input_win = curses.newwin(3, max_x, max_y - 3, 0)

    logs = []
    input_buffer = ""

    # serial writer (open once)
    while True:
        # === 1. Pull serial data ===
        while not rx_queue.empty():
            logs.append(rx_queue.get())
            if len(logs) > (log_win_height - 2):
                logs.pop(0)

        # === 2. Draw logs ===
        log_win.erase()
        log_win.box()

        for i, line in enumerate(logs):
            if i < log_win_height - 2:
                safe_line = line[: max_x - 3]
                log_win.addstr(i + 1, 1, safe_line)

        log_win.refresh()

        # === 3. Draw input window ===
        input_win.erase()
        input_win.box()
        input_win.addstr(1, 1, input_buffer[: max_x - 3])
        input_win.refresh()

        # === 4. Keyboard handling ===
        ch = stdscr.getch()
        if ch != -1:
            if ch in (10, 13):  # Enter
                ser.write((input_buffer + "\n").encode())
                logs.append(f">> {sanitize_bytes(input_buffer.encode())}")
                if len(logs) > (log_win_height - 2):
                    logs.pop(0)
                input_buffer = ""

            elif ch in (8, 127):  # Backspace
                input_buffer = input_buffer[:-1]

            elif 32 <= ch <= 126:
                input_buffer += chr(ch)

        time.sleep(0.01)


if __name__ == "__main__":
    threading.Thread(target=serial_reader, daemon=True).start()
    curses.wrapper(main)
