import time, sys
import threading
import numpy as np
import serial
import struct
import signal
from typing import List




''''ESTRUCTURA DE TELEGRAMAS/DATAGRAMA'''
CODE_TELEGRAM ={
#___________________________________________
#|1 |  2 | 3 - 4 | 5 - 6 |    7   |   8   |
#|ID|CODE|ADD_REG|NUM_REG|CRC_HIGH|CRC_LOW|
"READ": bytearray([0x03]),
#______________________________________________________________________
#|1 |  2 | 3 - 4 | 5 - 6 |    7   | 8 - 9| 10-11|...|  N-1   |   N   |
#|ID|CODE|ADD_REG|NUM_REG|NUM_BYTE|DATA_1|DATA_2|...|CRC_HIGH|CRC_LOW|
"WRITE": bytearray([0x10]),
#___________________________________________
#|1 |  2 |    3    |    5   |    7  |
#|ID|CODE|EXCEPTION|CRC_HIGH|CRC_LOW|
"ERROR_READ": bytearray([0x86]),
#____________________________________
#|1 |  2 |    3    |    5   |    7  |
#|ID|CODE|EXCEPTION|CRC_HIGH|CRC_LOW|
"ERROR_WRITE": bytearray([0x90])
}

class EXCEPTION_TELEGRAM(Exception):
    pass

error_messages=["", "Invalid function code", "Invalid register address", "Invalid data value"]


DRIVER_REGISTERS = {
    #"SAVE": bytearray([0x31, 0x00]),                # uint16//0=Do not save parameters to FLASH; 1: Save to FLASH
    "ID": bytearray([0x30, 0x01]),                  # ID
    "SOFTWARE_VERSION": bytearray([0x30, 0x02]),          # uint16//
    "HARDWARE_VERSION": bytearray([0x30, 0x03]),          # uint16//
    "BOOTLOADER_VERSION": bytearray([0x30, 0x04]),          # uint16//
    "PRODUCT_ID": bytearray([0x30, 0x05]),          # uint16//
    "COMUNICATION_TYPE": bytearray([0x30, 0x08]),     # uint16//
    "SET_STATUS": bytearray([0x53, 0x00]),          # uint16//SET DRIVER STATUS 0=STOP; 1=START; 2=CLEAN ALARMS
    "GET_STATUS": bytearray([0x54, 0x00]),          # DRIVER STATUS 0=STOP; 1=RUN
    "SET_SPEED": bytearray([0x53, 0x04]),           # int16//TARGET SPEED
    "GET_SPEED": bytearray([0x54, 0x10]),           # int16//CURRENT SPEED
    "GET_CURRENT": bytearray([0x54, 0x14]),         # int16//CURRENT CURRENT
    "GET_POSITION": bytearray([0x54, 0x18]),         # int32//CURRENT POSITION
    "GET_ERROR": bytearray([0x54, 0x20]),           # int32//CURRENT ERROR
    "GET_MOTOR_TEMPERATURE": bytearray([0x54, 0x04]),# int16//MOTOR TEMPERATURE
    "GET_DRIVE_TEMPERATURE": bytearray([0x21, 0x35]),# int16//DRIVER TEMPERATURE
    #dont work "MAX_SPEED": bytearray([0x50, 0x1C]), # uint16//MAXIMUM SPEED
    "POLE_PAIRS": bytearray([0x50, 0x18]),          # uint16//NUMNER OF POLE PAIRS
    "MAX_CURRENT": bytearray([0x50, 0x20]),         # uint16//MAXIMUM CURRENT
    "DIRECTION": bytearray([0x50, 0x28]),           # uint16//MOTOR DIRECTION 0=NORMAL; 1=INVERT
    "MAX_ACCELERATION": bytearray([0x51, 0x08]),    # uint16//MAXIMUM ACCELERATION
    "MAX_DECELATION": bytearray([0x51, 0x0C]),      # uint16//MAXIMUM DECELERATION   
    "CURVE_ACCELERATION": bytearray([0x51, 0x10]),  # uint16//S-CURVE ACCELERATION "Speed smoothing time S-type acceleration time"
    "MODE": bytearray([0x51, 0x00]),                 # MODE
    # "KP": bytearray([0x52, 0x00]),                 # FLOAT 32 Speed Kp
    # "KI": bytearray([0x52, 0x08]),                 # FLOAT 32 Speed Ki
    # "KD": bytearray([0x52, 0x10])                 # FLOAT 32 Speed Kd
}        
# Invertir el diccionario
INVERTED_DICT = {tuple(value): key for key, value in DRIVER_REGISTERS.items()}

ERROR_CODES = {
    0:"Abnormal current sampling",
    1:"Abnormal overcurrent protection circuit",
    2:"Abnormal drive motor cable",
    3:"Bus voltage is too high or too low",
    4:"Drive temperature detected abnormal often",
    5:"Drive 12V abnorma",
    6:"Drive 5V abnormal",
    7:"Motor circuit open",
    8:"Drive temperature is too high",
    9:"Motor temperature is too high",
    10:"Motor overcurrent protection",
    11:"Motor overload protection",
    12:"Overvoltage protection",
    13:"Undervoltage protection",
    14:"Encoder input abnormal",
    15:"wrong hardware version",
    17:"Physic emergency stop"
}


''''VARIABLES DE COMUNICACCIÓN'''
MSL = 0.015 * 2                                  #Maximum Segment Lifetime             
NUM_ATTEMPT = 3                                 #Número de intentos de la conexión con el driver

MAX_COUNT_SPEED = 4

class SVD48V:
    """
    Driver class to handle interaction with a motor controller device.

    Attributes:
        port: The serial port the driver is connected to.
        IDs: The identifiers for the motors driver.
        wheelRadius: The radius of the wheels for the motor.
        rpmMaxSpeed: The maximum speed in RPM.
    """

    def __init__(self, port:str="/dev/ttyUSB0", IDs:List[int]=[1,2], wheelRadius:int=6, maxSpeed:int=800, maxAcceleration:int=1000, maxDeceleration:int=1500, maxCurrent:int=6, polePairs:int=10, noSafe=False):
        """
        Initializes an instance of the class.

        Args:
            port (str): The port to be used. Defaults to "/dev/ttyUSB0".
            IDs (List[int]): A list containing the IDs. Defaults to [1,2].
            ereased -> polePairs(int): The number of motor pole pairs 
            wheelRadius (int): The radius of the wheel. Defaults to 6.
            maxSpeed (int): The maximum speed mm/s. Defaults to 800.
            maxAcceleration (int): The maximum acceleration mm/s2. Defaults to 1000.
            maxDeceleration (int): The maximum deceleration mm/s2. Defaults to 1500.
            maxCurrent (int): The maximum curren motor in Amperes. Defaults to 6.
        """
        print("Initializing SVD48V")
        self.port = port
        self.ids = IDs
        self.wheel_radius = wheelRadius
        self.mms2rpm = 60 / (2 * np.pi * self.wheel_radius)
        self.rad2mm = (2 * np.pi * self.wheel_radius)
        self.set_max_speed(maxSpeed)
        self.set_acceleration(maxAcceleration)
        self.set_deceleration(maxDeceleration)
        self.mutex = threading.Lock()

        self.accuracy_com = {"MSL" : 0.0, "CRC" : 0.0, "CODE" : 0.0, "OK" : 0.0}
        self.time_com = []

        self.driver = None
        self.thSecurity = None
        self.turnedOn = False
        self.safety = False
        self.counterSpeed = 0

        while self.driver is None:
            print('Opening serial port with the SVD48V')
            try:
                self.driver = serial.Serial(
                    port=self.port,
                    baudrate=115200,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=0
                )
            except Exception:
                sys.exit("Failed to open serial port "+ self.port +" with the SVD48V")

            if self.driver.isOpen():
                time.sleep(1)
                #Print initial message
                print_info = True
                flag = False
                timeout=0
                while print_info and timeout<25:
                    text = self.driver.readline()
                    if len(text) == 0 and flag:
                        print_info = False
                    elif len(text) > 0:
                        #print(text)
                        flag = True
                    timeout += 1
            else:
                sys.exit("Failed to open serial port "+ self.port +" with the SVD48V")

            if not self.check_connect():
                self.driver.close()
                self.driver = None
                print("Failed to connect motor drivers, restarting...")
                

        print("Starting SVD48V")
        self.show_params(True)
        
        ########Security pole pairs identification######
        motorPolePairs = self._get_motor_data(DRIVER_REGISTERS['POLE_PAIRS'])
        if not noSafe:
            if np.all(motorPolePairs == -np.inf):

                def timeout_handler(signum, frame):
                    raise TimeoutError("Input timeout reached")

                timeout_seconds = 10
                warning_msg = "\033[93m⚠ WARNING: Pole pairs not found!\033[0m\n"
                warning_msg += "\033[93mMake sure you calibrate the motors and drivers.\033[0m"
                signal.signal(signal.SIGALRM, timeout_handler)
                signal.alarm(timeout_seconds)  
                
                try:
                    response = input("\033[93mDo you want to continue? [y/n] (timeout: 10s): \033[0m")
                    signal.alarm(0)  
                    if response.lower() != "y":
                        exit(-1)
                except TimeoutError:
                    print("\n\033[91m✖ Timeout reached. Defaulting to 'NO'.\033[0m")
                    exit(-1)
            else:
                assert np.all(motorPolePairs == polePairs), f"\033[91mPole pairs don't match {motorPolePairs} != {polePairs}. Make sure that you calibrate the motors and drivers; it could be dangerous.\033[0m"

        
        self._set_motor_data(DRIVER_REGISTERS['MAX_ACCELERATION'], np.array([self.rpm_max_acceleration]*(len(self.ids)*2),dtype=np.int16))
        self._set_motor_data(DRIVER_REGISTERS['MAX_DECELATION'], np.array([self.rpm_max_deceleration]*(len(self.ids)*2), dtype=np.int16))
        self._set_motor_data(DRIVER_REGISTERS['MAX_CURRENT'], np.array([maxCurrent]*(len(self.ids)*2), dtype=np.int16))
        self._set_motor_data(DRIVER_REGISTERS['MODE'], np.array([0]*(len(self.ids)*2), dtype=np.int16))
        self._set_motor_data(DRIVER_REGISTERS['DIRECTION'], np.array([0, 1]*len(self.ids),dtype=np.int16))
            
        self.enable_driver()

        self.show_params(True)
        time.sleep(0.5)
        self.safety = True
        print("SVD48V started successfully")
        
    
    def __del__(self):
        if self.driver is not None and self.driver.isOpen():
            print("Turning off SVD48V")
            self.disable_driver()

            print("Closing serial connection")
            self.driver.close()
            print("-------------------------------COMMUNICATION STATISTICS--------------------------------")

            print("COMMUNICATION ERRORS:", self.accuracy_com, "ACCURACY:",self.accuracy_com["OK"]*100/sum(self.accuracy_com.values()) )
            print("AVERAGE COMMUNICATION TIME: ", np.mean(self.time_com))
            self.thSecurity.join(timeout=2)
        print("SVD48V Deleted")

    def check_connect(self):
        """
        Check de conection motor driver.

        """
        return not (None in self._get_motor_data( DRIVER_REGISTERS['GET_STATUS']))


    def enable_driver(self) -> bool:
        """
        Enable the motor driver.
        """
        if self.driver.isOpen():
            print("Starting motors")
            for id in self.ids:
                if 0 in self._read_register(id, DRIVER_REGISTERS["GET_STATUS"]):
                    # print("errorrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr",self.get_error())
                    print("start motor", id)
                    self._write_register(id, DRIVER_REGISTERS["SET_STATUS"], [np.int16(2),np.int16(2)])
                    self._write_register(id, DRIVER_REGISTERS['SET_SPEED'], [np.int16(0),np.int16(0)])
                    self._write_register(id, DRIVER_REGISTERS['SET_STATUS'], [np.int16(1),np.int16(1)])
            if not self.turnedOn:
                self.turnedOn = True
                self.thSecurity = threading.Thread(target=self.security_thread,daemon=True)
                self.thSecurity.start()
            print("Motors started ")
            return True

        else:
            sys.exit("PORT NOT OPEN. Has the SVD48V possibly been disconnected?")
            return False

    def disable_driver(self) -> bool:
        """
        Disable the motor driver.
        """
        print("Stopping motors")
        # Stop the drivers with speed 0
        self.turnedOn = False
        self._set_motor_data(DRIVER_REGISTERS['SET_STATUS'], np.array([0]*(len(self.ids)*2), dtype=np.int16))
        self._set_motor_data(DRIVER_REGISTERS['SET_SPEED'], np.array([0]*(len(self.ids)*2), dtype=np.int16))
        status = self._get_motor_data(DRIVER_REGISTERS['GET_STATUS'])
        if 1 in status:
            print("Error when stopping the motors")
            return False
        print("Motors stopped")
        return True
    def _generate_telegram(self, id, action_code, add_register, data=None, num_registers = 2):
        """
        Generate a telegram to be sent to the motor driver.
        
        Args:
            id (int): The id of the motor driver. Defaults to 0xee.
            action_code (str): The action to be performed ("READ" or "WRITE"). Defaults to "READ".
            add_register (Register): The register to read from or write to. Defaults to DRIVER_REGISTERS["GET_STATUS"].
            single (bool): Whether to read/write only a single register. Defaults to False.
            data (list): The data to write to the register. Required if action_code is "WRITE".
            
        Returns:
            bytearray: The generated telegram.
        """
        telegram = bytearray([id])                             #Direction
        telegram.extend(CODE_TELEGRAM[action_code])            #Action
        telegram.extend(add_register)                          #Registers

        if action_code == "WRITE":                             #telegram write
            if len(data) == num_registers:
                telegram.extend([0, len(data), len(data) * 2]) #Length Registers
            else:
                print("Missing or extra register data")
                return None
            for item in data:
                telegram.extend(self._short_to_2bytes(int(item)))#Set Data
        else:                                                   #Telegram Read
            telegram.extend([0, num_registers])       #Length registers
        telegram.extend(self._short_to_2bytes(self._calculate_crc(telegram))) 

        return telegram
    

    def _process_telegram(self, telegram, action_code="READ"):
        """
        Process/Send a telegram from the motor driver.
        
        Args:
            telegram (bytearray): The telegram to process/send.
            action_code (str): The action performed ("READ" or "WRITE"). Defaults to "READ".
            
        Returns:
            list: A list of data read from the register. [] if the action_code is "WRITE". None in case of failure.
        """
        try:
            data = []
            if self.driver.isOpen():
                self.mutex.acquire()      
                #numbers attempt
                for _ in range(NUM_ATTEMPT):
                    self.driver.flushInput()
                    # print(f"\033[34msend telegram {list(telegram)}\033[0m")
                    self.driver.write(telegram) #Send telegram
                    self.driver.flush()
                    t1 = time.time()            #get time for MSL
                    #Listening
                    while True:
                        time.sleep(0.004)
                        if action_code == "READ":
                            # Lee cabecera: dirección + código + longitud (3 bytes)
                            header = bytearray()
                            while len(header) < 3:
                                if time.time() - t1 > MSL:
                                    break
                                chunk = self.driver.read(3 - len(header))
                                header.extend(chunk)

                            if len(header) < 3:
                                print(f"MSL FAILURE en cabecera. RE-{action_code}")
                                self.accuracy_com["MSL"] += 1
                                break

                            # Calcula los bytes restantes: datos + 2 bytes de CRC
                            data_len = header[2]
                            remaining = 3 + data_len + 2  # datos + CRC_high + CRC_low

                            body = header
                            while len(body) < remaining:
                                if time.time() - t1 > MSL:
                                    break
                                chunk = self.driver.read(remaining - len(body))
                                body.extend(chunk)

                            if len(body) < remaining:
                                print(f"MSL FAILURE en body. RE-{action_code}")
                                self.accuracy_com["MSL"] += 1
                                break

                            reply = body
                        else:
                            reply = bytearray (self.driver.readline())#get telegram of reply

                        #Have telegram of reply?
                        if len(reply) > 1:
                            # print(f"\033[36mReply telegram {list(reply)}\033[0m")
                            #Have same number of action?
                            if reply[1] != CODE_TELEGRAM[action_code][0] :
                                if reply[1] != CODE_TELEGRAM["ERROR_" + action_code][0]:
                                    self.mutex.release()
                                    raise EXCEPTION_TELEGRAM(error_messages[reply[2]])
                                print(f"UNSUITABLE TELEGRAM. RE-{action_code}")
                                self.accuracy_com["CODE"] += 1
                                break
                            if action_code == "READ" and reply[2] != len(reply)-5:
                                print(f"\033[34msend telegram {list(telegram)}\033[0m")
                                print(f"\033[36mReply telegram {list(reply)}\033[0m")
                                print(f"REPLY {reply[2]} bytes, expected {len(list(reply))-5} bytes. RE-{action_code}")
                                # self.accuracy_com["CODE"] += 1
                                continue

                            #Check CRC for telegram
                            crc_low, crc_high = reply.pop(), reply.pop()
                            tel_crc_high, tel_crc_low = self._short_to_2bytes(self._calculate_crc(reply))
                            if crc_high != tel_crc_high or crc_low != tel_crc_low:
                                print(f"CRC FAILURE. RE-{action_code}")
                                self.accuracy_com["CRC"] += 1
                                print(f"\033[34msend telegram {list(telegram)}\033[0m")
                                print(f"\033[36mReply telegram {list(reply)}\033[0m")
                                continue

                            #Get data from telegram response
                            if action_code == "READ":
                                for i in range(0, reply[2], 2):
                                    # data.append(np.int16(int(reply[i+3] * 2**8) + reply[i+4]))
                                    value = (reply[i+3] << 8) | reply[i+4]
                                    data.append(value - 65536 if value > 32767 else value)

                            
                            #Telegram OK
                            self.mutex.release()
                            self.accuracy_com["OK"] += 1
                            self.time_com.insert(0, time.time() - t1)
                            self.time_com = self.time_com[:50]
                            return data
                        
                        if time.time() - t1 > MSL:
                            print(f"MSL FAILURE. RE-{action_code}")
                            self.accuracy_com["MSL"] += 1
                            break  
                self.mutex.release()
                print(f"{action_code} ATTEMPTS EXHAUSTED ")
        except serial.SerialException:
            sys.exit("FAUL IN PORT. Has the SVD48V possibly been disconnected?")
        except EXCEPTION_TELEGRAM:
            raise
        except Exception as e:
            self.mutex.release()
            raise
        return None

    def _read_register(self, id=0xee, add_register=DRIVER_REGISTERS["GET_STATUS"], num_registers=2):
        """
        Read register from the motor driver.

        Args:
            id (int): The id of the motor driver. Defaults to 0xee.
            add_register (str): The register to read from. Defaults to DRIVER_REGISTERS["GET_STATUS"].
            single (bool): Whether to read only a single register. Defaults to False.

        Returns:
            list: A list of the data from the register.
        """
        attempt = 0
        telegram = self._generate_telegram(id, "READ",add_register, num_registers=num_registers)
        try:
            while (data := self._process_telegram(telegram, "READ")) is None and attempt < 2:
                print("Restarting Reading")
                time.sleep(0.005)
                attempt+=1
            return data
        except Exception as e:
            print("######ERROR###### ",list(add_register), "\t", e)
            return None
    
        
        
    def _write_register(self, id, add_register, data_tuple, num_registers=2):
        """
        Write to a register on the motor driver.

        Args:
            id (int): The id of the motor driver. Defaults to 0xee.
            add_register (str): The register to write to. Defaults to DRIVER_REGISTERS["SET_STATUS"].
            data_tuple (list): The data to write to the register. Defaults to [0,0].

        Returns:
            int: -2 if the wrong number of data items are provided, 0 OK and None fault communication .
        """

        attempt = 0
        telegram = self._generate_telegram(id, "WRITE", add_register, data_tuple, num_registers)
        if telegram is None:
            print("Wrong number of data items")
            return -2
        try:
            while (data := self._process_telegram(telegram, "WRITE")) is None and attempt < 2:
                print("Restarting Writing")
                time.sleep(0.005)
                attempt+=1
            return data
        except Exception as e:
            print("######ERROR###### ",list(add_register), "\t", e)
            return None

    def _calculate_crc(self, telegram):
        """
        Calculate the Cyclic Redundancy Check (CRC) of a telegram.

        Args:
            telegram (bytearray): The telegram to calculate the CRC of.

        Returns:
            int: The calculated CRC of the telegram.
        """
        crc_result = 0xffff
        for i in range(len(telegram)):
            crc_result = crc_result ^ int(telegram[i])
            for _ in range(8):
                crc_num = (crc_result & 0x0001)
                xor_flag = 1 if crc_num == 1 else 0
                crc_result >>= 1
                if xor_flag:
                    crc_result = crc_result ^ 0xa001
        return crc_result

    def _short_to_2bytes(self, short):
        """
        Convert a short integer to a list of 2 bytes.

        Args:
            short (int): The short integer to convert.

        Returns:
            list: The short integer represented as a list of 2 bytes.
        """
        low = int(short & 0x00FF)
        high = int((short & 0xFF00) >> 8)
        return high, low
    
    def _2short_to_int(self, short1, short2):
            return (int(short1) << 16) + int(short2)
    

    def _2shorts_to_float(self, short1, short2):
        """
        Convierte dos shorts (de 2 bytes cada uno) en un número flotante de 32 bits.

        Args:
            short1 (int): Primer short, un valor de 2 bytes.
            short2 (int): Segundo short, un valor de 2 bytes.

        Returns:
            float: El número flotante de 32 bits resultante.
        """
        # Combinamos los dos shorts en una lista de 4 bytes
        bytes_values = [
            (short1 >> 8) & 0xFF,  # Byte alto del primer short
            short1 & 0xFF,         # Byte bajo del primer short
            (short2 >> 8) & 0xFF,  # Byte alto del segundo short
            short2 & 0xFF          # Byte bajo del segundo short
        ]
        
        # Empaquetar estos 4 bytes como un valor de 32 bits.
        packed = struct.pack('BBBB', bytes_values[0], bytes_values[1], bytes_values[2], bytes_values[3])

        # Interpretar los 4 bytes como un valor flotante de 32 bits
        return struct.unpack('f', packed)[0]
                

    def show_params(self, advanced=False):
        """
        Display the parameters of the current object.

    Args:
        advanced (bool, optional): If True, display advanced parameters. Defaults to False.
        """

        print("------------------------------")
        print("Driver parameters list:")

        if advanced:
            print("port: ", self.port)
            print("Drivers ID: ", self.ids)
            print("Software Version ", self._get_motor_data(register_key=DRIVER_REGISTERS["SOFTWARE_VERSION"], num_registers=1))
            print("Hardware Version: ", self._get_motor_data(register_key=DRIVER_REGISTERS["HARDWARE_VERSION"], num_registers=1))
            print("Bootloader Version: ", self._get_motor_data(register_key=DRIVER_REGISTERS["BOOTLOADER_VERSION"], num_registers=1))
            print("Product ID: ", self._get_motor_data(register_key=DRIVER_REGISTERS["PRODUCT_ID"], num_registers=1))
            print("rpmMaxSpeed: ", self.rpm_max_speed)
            print("wheelRadius: ", self.wheel_radius)
            # print("KP: ",self.get_pid(param="KP"))
            # print("KI: ", self.get_pid(param="KI"))
            # print("KD: ", self.get_pid(param="KD"))

        print("Driver states(m1...mn): ", np.round(self.get_status(), 0).tolist())   
        print("Errors (m1...mn): ", self.get_error())
        print("Speed rpm (m1...mn): ", np.round(self.get_rpm(), 4).tolist())
        print("Speed mm/s (m1...mn): ", np.round(self.get_speed(), 4).tolist())
        print("Current amp (m1...mn): ", np.round(self.get_current(), 4).tolist())
        print("Angle rad (m1...mn): ", np.round(self.get_angle(), 4).tolist())
        print("Motor temperature Cº(m1...mn): ", np.round(self.get_temperature(), 4).tolist())
        print("COMMUNICATION ERRORS:", self.accuracy_com, "ACCURACY:",self.accuracy_com["OK"]*100/sum(self.accuracy_com.values()) )
        print("AVERAGE COMMUNICATION TIME: ", np.mean(self.time_com))
        print("------------------------------")


    def security_thread(self):
        """
        Check whether the motor driver is still responsive and try to reset it if not.
        """
        while self.turnedOn:
            if self.driver.isOpen():
                error = self.get_error()
                status = self.get_status()
                rpm = self.get_rpm()

                #Check errors
                if not all(err != ERROR_CODES.get(17) for err in error):
                    print(error)
                    self.disable_driver()
                    print("DRIVER STOPED, DETECTED ERROR")

                #Check Speed
                elif np.max(np.abs(rpm))>self.rpm_max_speed:
                    if self.counterSpeed < MAX_COUNT_SPEED:
                        self.counterSpeed += 1
                    else:
                        self.disable_driver()
                        print("DRIVER STOPED, TOO FAST")
                        self.counterSpeed = 0

                #Check status
                elif 0 in status: # Motor OFF 0, ON 1, Alarm 2
                    time.sleep(1)
                    print("THE DRIVER HAS STOPPED. TRYING TO RESTART")
                    self.enable_driver()
                else:
                    self.counterSpeed = 0
            else:
                sys.exit("PORT NOT OPEN. Has the SVD48V possibly been disconnected?")
            time.sleep(0.05)
            

    def _get_motor_data(self, register_key: bytes, divisor: float = 1.0, num_registers=2) -> np.ndarray:
        """
        Retrieve motor data from the specified register.

        Args:
            register_key (bytes): The key identifying the register to read from.
            divisor (float, optional): A value by which to divide the retrieved data. Defaults to 1.0.

        Returns:
            np.ndarray: An array of motor data scaled by the specified divisor. 
                        Returns an array of the same shape as the number of motors, with None 
                        values replaced if the read operation fails.
        """
        data = []
        for id in self.ids:
            data.extend(self._read_register(id, register_key, num_registers) or [-np.inf, -np.inf])
        return np.array(data) / divisor

    def _set_motor_data(self, register_key: bytes, value: np.ndarray) -> None:
        """
        Set motor data to the specified register.

        Args:
            register_key (bytes): The key identifying the register to write to.
            value (list): A list of values to set for each motor. The length must be twice the number of motors.

        Raises:
            AssertionError: If the length of the value list does not match twice the number of motors.

        Returns:
            None
        """
        assert len(value) >= len(self.ids) * 2, "Set motor data incorrectly. Expected length: {}".format(len(self.ids) * 2)
        for i in range(len(self.ids)):
            self._write_register(self.ids[i], register_key, [value[i * 2], value[i * 2 + 1]])

    def _get_error_codes(self, error:int)->List[str]:
        error_codes = []
        #print(error)
        for bit in ERROR_CODES:
            if error & (1 << bit):
                error_codes.append(ERROR_CODES.get(bit))
        return error_codes
    
    def get_status(self):
        """
        Get the status of the motor driver.

        Returns:
            The status of the motor driver.
        """
        return self._get_motor_data(DRIVER_REGISTERS["GET_STATUS"])

    def get_temperature(self):
        """
        Get the temperature of the motors.

        Returns:
            The temperature of the motors.
        """
        return self._get_motor_data(DRIVER_REGISTERS["GET_MOTOR_TEMPERATURE"], divisor=-10)

    def get_rpm(self):
        """
        Get the rpm of the motors.

        Returns:
            The rpm of the motors.
        """
        return self._get_motor_data(DRIVER_REGISTERS["GET_SPEED"], divisor=10)

    def get_speed(self):
        """
        Get the speed of the motors.

        Returns:
            The speed of the motors.
        """
        return self.get_rpm()/self.mms2rpm
    
    def get_current(self):
        """
        Get the current of the motors.

        Returns:
            The current of the motors.
        """
        return self._get_motor_data(DRIVER_REGISTERS["GET_CURRENT"], divisor=10)
    
    def get_angle(self):
        """
        Get the angle of the motors.

        Returns:
            The angle of the motors.
        """
        position = self._get_motor_data(DRIVER_REGISTERS["GET_POSITION"], divisor=1, num_registers=4)
        return np.array([position[1::2]])/4096
    
    def get_position(self):
        """
        Get the position of the motors.

        Returns:
            The position of the motors.
        """
        return self.get_angle()*self.rad2mm
    
    def get_error(self) -> List[str]:
        """
        Get the error of the motors.

        Returns:
            The error of the motors.
        """
        errors = self._get_motor_data(DRIVER_REGISTERS["GET_ERROR"], num_registers=4)
        ret = []
        for i in range(0, len(errors), 2):
            error = self._2short_to_int(int(errors[i]), int(errors[i + 1]))
            ret.append(self._get_error_codes(error)) 
        return ret
    
    # def get_pid(self, param:str):
    #     assert param in ["KP", "KI", "KD"], "PID param must be KP, KI or KD"
    #     pids = self._get_motor_data(DRIVER_REGISTERS[param], num_registers=4)
    #     ret = []

    #     for i in range(0, len(pids), 2):
    #         pid = self._2shorts_to_float(int(pids[i]), int(pids[i + 1]))
    #         ret.append(pid) 

    #     return ret

    
    
    def get_safety(self):
        """
        Returns the current safety status of the motor drivers.

        Returns:
            bool: The safety status. True if it's safe to operate, False it is not safe to operate.
        """
        return self.safety
    
    def get_enable(self):
        """
        Returns the current status of the motor drivers.

        Returns:
            bool: The status. True if it's  enable, False it's disable.
        """
        return self.turnedOn
    
    
    def set_acceleration(self, max_acceleration):
        """
        Set the maximum acceleration.

        Args:
            max_acceleration (int): The maximum acceleration to set in mm/s^2.
        """
        self.rpm_max_acceleration = abs(int(max_acceleration * self.mms2rpm))

    def set_deceleration(self, max_deceleration):
        """
        Set the maximum deceleration.

        Args:
            max_deceleration (int): The maximum deceleration to set in mm/s^2.
        """
        self.rpm_max_deceleration = abs(int(max_deceleration * self.mms2rpm))

    def set_max_speed(self, max_speed:int) ->None:
        """
        Set the maximum speed.

        Args:
            max_speed: The maximum speed to set in mm/s.
        """
        self.rpm_max_speed = abs(int(max_speed * self.mms2rpm))
        
    def set_rpm(self, motor_rpm:np.ndarray) ->None:
        """
        Set the rpm of the motors. If the rpm exceeds the maximum, 
        all rpms are reduced proportionally.

        Args:
            motor_rpm[(int), (int), (int), (int)]: The rpm to set for the four motors.

        Returns:
            -2 if emergency stop is activated, -1 if the rpm had to be adjusted due to exceeding the maximum RPM, 0 otherwise.
        """
        if self.safety:
            ret = 0
            max_rpm = np.max(np.abs(motor_rpm))

            if max_rpm > self.rpm_max_speed:
                print(f"WARNING: WHEEL SPEED LIMIT EXCEEDED {max_rpm} WHEN MAXIMUM IS {self.rpm_max_speed}")
                motor_rpm = (motor_rpm / max_rpm) * self.rpm_max_speed
                ret = -1
            motor_rpm = motor_rpm.astype(np.int16)
            #print(f"RPM {motor_rpm.tolist()}")

            self._set_motor_data(DRIVER_REGISTERS["SET_SPEED"], motor_rpm)
        else:
            ret = -2
            print("Emergency stop is activated, the base will not move")
        return ret
        
    
    def set_speed(self, motor_speed:np.ndarray):
        """
        Set the speed of the motors. If the speed exceeds the maximum, 
        all speeds are reduced proportionally.

        Args:
            motor_speed[(int), (int), (int), (int)]: The speed to set for the four motors.

        Returns:
            -2 if emergency stop is activated, -1 if the speed had to be adjusted due to exceeding the maximum RPM, 0 otherwise.
        """
        return self.set_rpm(np.array(motor_speed, dtype=float) * self.mms2rpm)
    
    def reset_emergency_stop(self):
        """
        Resets the emergency stop for the motor drivers.
        """
        print("RESET EMERGENCY STOP")
        self._set_motor_data(DRIVER_REGISTERS["MAX_DECELATION"], np.array([self.rpm_max_deceleration]*(len(self.ids)*2), dtype=np.int16))        
        self.safety = True

    def emergency_stop(self):
        """
        Triggers an emergency stop for the motor drivers.

        Note:
            The acceleration is increased abruptly to induce a quick stop for each motor driver using their IDs.
            The speed is set to zero and the safety flag is also set to False, indicating an unsafe condition.
        """
        print("EMERGENCY STOP")
        self.safety = False
        acc_stop = self.mms2rpm*2000

        self._set_motor_data(DRIVER_REGISTERS["MAX_DECELATION"], np.array([acc_stop]*(len(self.ids)*2),dtype=np.int16))
        self._set_motor_data(DRIVER_REGISTERS["SET_SPEED"], np.array([0]*(len(self.ids)*2),dtype=np.int16))