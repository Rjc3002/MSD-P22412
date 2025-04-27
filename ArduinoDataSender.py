import serial
import time
import csv

# Configure the software serial port settings
software_port = 'COM4'  # Use the same port defined in the Arduino sketch
baud_rate = 9600

# Initialize the serial connection
#ser = serial.Serial(software_port, baud_rate, timeout=1)

# Function to send data to Arduino and receive response
def send_receive_data(data, ser):
    ser.write(data.encode())
    time.sleep(0.1)  # Delay to allow Arduino to process the data
    response = ser.readline().decode().strip()
    return response

def just_receive_data(ser):
    time.sleep(0.1)  # Delay to allow Arduino to process the data
    response = ser.readline().decode().strip()
    return response

#print("Reading File: 'data.csv', Type 'start' to begin sending. \nType 'path -[new file path] to change csv location. Type 'end' to end.")


def main():
    print("starting")
    user_input = "data.csv"
    file_path = 'data.csv'
    while user_input.lower() != "end" and user_input.lower() != "quit" and user_input.lower() != "start":
        user_splice = user_input.split()
        if len(user_splice) >= 2 and user_splice[0] == "file":
            file_path = user_splice[1]
        disp_string = "Type 'path [new file path] to change csv location. Type 'end' to end. \nSelected File: '",  file_path,"', Type 'start' to begin sending. "
        user_input = input(disp_string)
        # Perform any processing with the input here
        # For example, you can store the input in a list or process it in some way
        # Here, we are just printing the input back
        print("You entered:", user_input)

    print("User input ended.")

    if user_input =="start":
        ser = serial.Serial(software_port, baud_rate, timeout=1)
        # Read data from a CSV file
        with open('data.csv', mode='r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                data_to_send = ','.join(row)  # Convert CSV row to a string
                print("Sending to Arduino: " + data_to_send)

                # Send data to Arduino and receive response
                response = send_receive_data(data_to_send, ser)
                print("Received from Arduino: " + response)

        # Close the serial connection

        should_end = False
        show_recieves = True
        while show_recieves and not should_end:
            if user_input.lower() == "end": # can't actually do this, just do ctrl+c
                should_end = True
            response = just_receive_data(ser)
            print("Received from Arduino: " + response)
        ser.close()
    

if __name__ == "__main__":
    main()
