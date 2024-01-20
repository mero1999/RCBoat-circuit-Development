import pygame
import serial
from serial.tools import list_ports
import time

class Controller:
    def __init__(self):
        self.lat = 0.00
        self.lon = 0.00
        self.alt = 0.00
        self.re = 0
        self.last_idle_time = time.time()
        self.Gear = "N"
        self.Speed = 0  
        self.setSpeed = 0
        self.Stearing = 90
        self.Clamp = 90
        self.msg = self.Gear + "," + str(self.Speed) + "," + str(self.Stearing) + "," + str(self.Clamp)
        # Configure the serial port using the found port
        self.serial_port = self.find_serial_port()
        self.ser = serial.Serial(self.serial_port, 9600, timeout=1)
    
    def find_serial_port(self):
        # Get a list of available serial ports
        ports = list(list_ports.comports())
        # Filter ports based on some criteria (you may adjust this based on your needs)
        filtered_ports = [port.device for port in ports if "USB" in port.description]

        while not filtered_ports:
            input("Device port not found, reconnect and press enter>>>")
            # Get a list of available serial ports
            ports = list(list_ports.comports())
            # Filter ports based on some criteria (you may adjust this based on your needs)
            filtered_ports = [port.device for port in ports if "USB" in port.description]

        if len(filtered_ports) == 1:
            return filtered_ports[0]
        else:
            print("Multiple serial ports found. Please choose one:")
            for i, port in enumerate(filtered_ports, 1):
                print(f"{i}. {port}")
            
            selected_index = int(input("Enter the number of the desired serial port: ")) - 1

            while selected_index < 0 or selected_index >= len(filtered_ports):
                print("Invalid selection, retry:")
                selected_index = int(input("Enter the number of the desired serial port: ")) - 1

            return filtered_ports[selected_index]
    
    def handle_controller(self, controller):
        axes_names = ["LX", "LY", "RX", "RY", "L2", "R2"]
        buttons_names = ["Cross", "Circle", "Square", "Triangle", "Share", "PS",
                        "Options", "L3", "R3", "L1", "R1", "UP", "Down", "Left",
                        "Right", "Touchpad"]
        axes = [controller.get_axis(i) for i in range(controller.get_numaxes())]
        buttons = [controller.get_button(i) for i in range(controller.get_numbuttons())]
        values = dict(zip(axes_names, axes))
        values.update(dict(zip(buttons_names, buttons)))
        
        for key, value in values.items():
            if key == "R2":
                if value > 0.5 and self.Gear == "N":
                    self.Gear = "D"
                    self.Speed = self.setSpeed
                elif value <= 0.5 and self.Gear == "D":
                    self.Gear = "N"
                    self.Speed = 0

            if key == "L2":
                if value > 0.5 and self.Gear == "N":
                    self.Gear = "R"
                    self.Speed = self.setSpeed
                elif value <= 0.5 and self.Gear == "R":
                    self.Gear = "N"
                    self.Speed = 0
            
            if key == "LX":
                if int((value + 1) * 90) in range(0,5):
                    self.Stearing = str(0)
                elif int((value + 1) * 90) in range(5,87):
                    self.Stearing = str(45)
                elif int((value + 1) * 90) in range(93,175):
                    self.Stearing = str(135)
                elif int((value + 1) * 90) in range(175,180):
                    self.Stearing = str(180)
                else:
                    self.Stearing = str(int((value + 1) * 90))
                    
            if key == "UP" and value == 1 and self.setSpeed < 1024:
                self.setSpeed += 16
                if self.Gear == "D" or self.Gear == "R":
                    self.Speed = self.setSpeed   
            if key == "Down" and value == 1 and self.setSpeed > 0:
                self.setSpeed -= 16
                if self.Gear == "D" or self.Gear == "R":
                    self.Speed = self.setSpeed
                    
            if key == "R1" and value == 1:
                self.Clamp = 180
            elif key == "R1" and value == 0 and self.Clamp > 90:
                self.Clamp = 90
            if key == "L1" and value == 1:
                self.Clamp = 0
            elif key == "L1" and value == 0 and self.Clamp < 90:
                self.Clamp = 90

        nmsg = self.Gear + "," + str(self.Speed) + "," + str(self.Stearing) + "," + str(self.Clamp) + "\n"
        if nmsg != self.msg:
            print(nmsg)
            self.ser.write(nmsg.encode())
            self.msg = nmsg     
    
    def read_serial(self):
        try:
            while self.ser.in_waiting > 0:
                received_data = self.ser.readline().decode('utf-8').strip()
                print(f"Received from serial: {received_data}")
                self.re = 0
                if (received_data.split()[0] == "Location:"):
                    print(f"Got location, lat: " + received_data.split()[1][:-1]+ ", lon: " + received_data.split()[2][:-1] + ", alt: " + received_data.split()[3][:-1])
                    self.lat = received_data.split()[1][:-1]
                    self.lon = received_data.split()[2][:-1]
                    self.alt = received_data.split()[3][:-1]        
        except:
            if self.re >3:
                self.ser.close()
                input("transmitter communication lost reconnect and press enter>>>")
                # Configure the serial port using the found port
                self.serial_port = self.find_serial_port()
                self.ser = serial.Serial(self.serial_port, 9600, timeout=1)
                self.re = 0
            else:
                self.re+= 1
    
    def run(self):
        pygame.init()
        pygame.joystick.init()

        controller_found = False
        while not controller_found:
            try:
                controller = pygame.joystick.Joystick(0)
                controller.init()
                controller_found = True
                print("PS4 Controller Connected")
            except pygame.error:
                input("No PS4 Controller found. press enter to retry>>>")
        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        return
                    elif event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP or event.type == pygame.JOYAXISMOTION or event.type == pygame.JOYHATMOTION:
                        self.handle_controller(controller)
                time.sleep(0.05)
                
                # Check for idle conditions and send serial request
                current_time = time.time()
                if (current_time - self.last_idle_time) > 3 and self.re ==0:
                    print("Sending serial request (idle)")
                    self.ser.write("request \n".encode())
                    self.last_idle_time = current_time

                # Read and print data from the serial port
                self.read_serial()
        finally:
            # Close the serial port before exiting
            self.ser.close()       

a = Controller()
a.run()