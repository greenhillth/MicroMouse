import serial
import threading

LOGGING = 0
DATA = 1

def read_serial(port, baud_rate):
    ser = serial.Serial(port, baud_rate)
    logFile = open('micromouse.log', 'w')
    motorPlotFile = open('motor.csv', 'w')
    file = logFile
    mode = LOGGING
    while True:
        line = ser.readline().decode().strip()

        if 'BEGIN DATA TRANSMISSION' in line:
            file.write(line + '\n')
            outFile = './plots/data/' + ser.readline().decode().strip()
            file = open(outFile, 'w')
            mode = DATA
            continue
        elif 'END DATA TRANSMISSION' in line:
            file = logFile
            mode = LOGGING
        
        
        if mode==LOGGING:
            print(line)

        file.write(line + '\n')

def write_serial(port, baud_rate):
    ser = serial.Serial(port, baud_rate)
    while True:
        data_to_write = input()
        ser.write(data_to_write.encode())

if __name__ == "__main__":
    port = '/dev/ttyACM0'  # Replace 'COM1' with your COM port
    baud_rate = 115200  # Adjust baud rate if necessary

    # Create threads for reading and writing
    read_thread = threading.Thread(target=read_serial, args=(port, baud_rate))
    write_thread = threading.Thread(target=write_serial, args=(port, baud_rate))

    # Start the threads
    read_thread.start()
    write_thread.start()

    # Wait for threads to finish
    read_thread.join()
    write_thread.join()