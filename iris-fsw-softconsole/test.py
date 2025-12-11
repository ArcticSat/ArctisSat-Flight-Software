import socket
from time import sleep
import ccsdspy
from ccsdspy import PacketField, PacketArray
from ccsdspy.utils import split_packet_bytes
from ccsdspy.converters import StringifyBytesConverter
import threading
import serial
import queue
import sys
from io import BytesIO

# Configuration
SERIAL_PORT = 'COM14'  # Change to your serial port
BAUD_RATE = 115200
OUTPUT_FILE = './mypackets.bin'

# Queue to pass data between threads
data_queue = queue.Queue()
stop_event = threading.Event()

fileLock = threading.Lock()

newData = 0

doneReading = 0

file = None

def keyboard_sender():
    """Thread 3: Read from keyboard and send over serial"""
    print("Starting keyboard sender thread...")
    try:
        ser_write = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        while not stop_event.is_set():
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                stop_event.set()
                break
            if line.strip().lower() in ('exit', 'quit'):
                stop_event.set()
                break
            try:
                ser_write.write(line.encode('utf-8') + b'\n')
                ser_write.flush()
            except Exception as e:
                print(f"Serial write error: {e}")
                break
    except Exception as e:
        print(f"Keyboard sender error: {e}")
    finally:
        try:
            ser_write.close()
        except:
            pass


def read_serial():
    """Thread 1: Read from serial port and write to file"""
    global newData
    global file
    global fileLock
    global doneReading
    print("Starting serial read thread...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        with open(OUTPUT_FILE, 'wb') as file:
            while not stop_event.is_set():
                sleep(0.05)
                data = ser.read(1024)
                if data:
                    fileLock.acquire(timeout=0)
                    file.write(data)
                    file.flush()
                    fileLock.release()
                    if(file.tell() > 6):
                        newData = 1

    except Exception as e:
        print(f"Serial read error: {e}")

def split_by_apid():
    """Thread 2: Read from queue and split packets by APID"""
    global newData
    global file
    global fileLock
    global doneReading
    print("Starting packet split thread...")
    while not stop_event.is_set():
        try:
            if( newData == 0):
                continue
            newData = 0
            pkt = ccsdspy.VariableLength([
                PacketArray(
                name="data",
                data_type="uint",
                bit_length=8,
                array_shape="expand",   # makes the data field expand
                ),
            ])
            fileLock.acquire(timeout=0)
            # result = pkt.load('mypackets.bin', include_primary_header = True)
            # print(ccsdspy.utils.count_packets('mypackets.bin', return_missing_bytes=False))
            packets = ccsdspy.utils.split_packet_bytes('mypackets.bin', include_primary_header=False)
            print(ccsdspy.utils.split_by_apid('mypackets.bin'))
            packetsWithHeader = ccsdspy.utils.split_packet_bytes('mypackets.bin', include_primary_header=True)
            
            tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            for packet in packetsWithHeader:
                #write to UDP port 10015
                # packet = bytes("BIG CHUNGUS", "utf-8")
                # ensure packet is bytes and append 0x00
                packet = bytes(packet) + b'\x00'
                tm_socket.sendto(packet, ("127.0.0.1", 10015))



            for packet in packets:
                print(packet.decode("utf-8", errors='ignore'), end='')

            file.truncate(0)
            file.flush()
            file.seek(0)
            file.flush()
            fileLock.release()

            # print(result)
            # print()

        except queue.Empty:
            continue
        except Exception as e:
            print(f"Packet processing error: {e}")

if __name__ == '__main__':
    try:
        t1 = threading.Thread(target=read_serial, daemon=True)
        t2 = threading.Thread(target=split_by_apid, daemon=True)
        
        t1.start()
        t2.start()
        
        while True:
            pass  # Keep the main thread alive
    except KeyboardInterrupt:
        print("Exiting program...")
        stop_event.set()  # Signal threads to stop
        t1.join()
        t2.join()
        sys.exit(0)