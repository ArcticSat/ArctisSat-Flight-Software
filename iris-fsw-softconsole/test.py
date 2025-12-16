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

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.001)


def udp_to_serial():
    global ser
    """Thread 3: Read from localhost:10025 UDP and write to serial port"""
    print("Starting UDP->Serial thread...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 10025))
        sock.settimeout(1.0)
        while not stop_event.is_set():
            try:
                data, _ = sock.recvfrom(4096)
                if data:
                    print(data)
                    ser.write(data)
                    ser.flush()
            except socket.timeout:
                continue
            except Exception as e:
                print(f"UDP->Serial error: {e}")
                break
    except Exception as e:
        print(f"UDP->Serial setup error: {e}")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        try:
            sock.close()
        except Exception:
            pass


def read_serial():
    """Thread 1: Read from serial port and write to file"""
    global newData
    global file
    global fileLock
    global doneReading
    global ser

    print("Starting serial read thread...")
    try:
        with open(OUTPUT_FILE, 'wb') as file:
            while not stop_event.is_set():
                sleep(0.01)
                data = ser.read(1024)
                if data:
                    fileLock.acquire(timeout=10000)
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
            fileLock.acquire(timeout=10000)
            # result = pkt.load('mypackets.bin', include_primary_header = True)
            # print(ccsdspy.utils.count_packets('mypackets.bin', return_missing_bytes=False))
            packets = ccsdspy.utils.split_packet_bytes('mypackets.bin', include_primary_header=False)
            print(ccsdspy.utils.split_by_apid('mypackets.bin'))
            packetsWithHeader = ccsdspy.utils.split_packet_bytes('mypackets.bin', include_primary_header=True)
            
            tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            for packet in packetsWithHeader:
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
        t3 = threading.Thread(target=udp_to_serial, daemon=True)

        t1.start()
        t2.start()
        t3.start()
        
        while True:
            pass  # Keep the main thread alive
    except KeyboardInterrupt:
        print("Exiting program...")
        stop_event.set()  # Signal threads to stop
        t1.join()
        t2.join()
        t3.join()
        sys.exit(0)