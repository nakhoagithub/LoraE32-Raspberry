import RPi.GPIO as GPIO
import serial
import time
import json

class LoraE32:
    ''' class to interface an ESP32 via serial commands to the EBYTE E32 Series LoRa modules '''
    
    # UART ports
    PORT = { 'U1':'/dev/ttyS0', 'U2':'/dev/ttyUSB0', 'U3': '/dev/ttyUSB1' }
    # UART parity strings
    PARSTR = { '8N1':'00', '8O1':'01', '8E1':'10' }
    PARINV = { v:k for k, v in PARSTR.items() }
    # UART parity bits
    PARBIT = { 'N':None, 'E':0, 'O':1 }
    
    # UART baudrate
    BAUDRATE = { 1200:'000', 2400:'001', 4800:'010', 9600:'011',
                 19200:'100', 38400:'101', 57600:'110', 115200:'111' }
    BAUDRINV = { v:k for k, v in BAUDRATE.items() }
    # LoRa datarate
    DATARATE = { '0.3k':'000', '1.2k':'001', '2.4k':'010',
                 '4.8k':'011', '9.6k':'100', '19.2k':'101' }
    DATARINV = { v:k for k, v in DATARATE.items() }
    # Commands
    CMDS = { 'setConfigPwrDwnSave':0xC0,
             'getConfig':0xC1,
             'setConfigPwrDwnNoSave':0xC2,
             'getVersion':0xC3,
             'reset':0xC4 }
    # operation modes (set with M0 & M1)
    OPERMODE = { 'normal':'00', 'wakeup':'10', 'powersave':'01', 'sleep':'11' }
    # model frequency ranges (MHz)
    FREQ = { 170:[160, 170, 173], 400:[410, 470, 525], 433:[410, 433, 441],
             868:[862, 868, 893], 915:[900, 915, 931] }
    # version info frequency
    FREQV = { '0x32':433, '0x38':470, '0x45':868, '0x44':915, '0x46':170 }
    # model maximum transmision power
    # 20dBm = 100mW - 27dBm = 500 mW - 30dBm = 1000 mW (1 W)
    MAXPOW = { 'T20':0, 'T27':1, 'T30':2 }
    # transmission mode
    TRANSMODE = { 0:'transparent', 1:'fixed' }
    # IO drive mode
    IOMODE = { 0:'TXD AUX floating output, RXD floating input',
               1:'TXD AUX push-pull output, RXD pull-up input' }
    # wireless wakeup times from sleep mode
    WUTIME = { 0b000:'250ms', 0b001:'500ms', 0b010:'750ms', 0b011:'1000ms',
               0b100:'1250ms', 0b101:'1500ms', 0b110:'1750ms', 0b111:'2000ms' }
    # Forward Error Correction (FEC) mode
    FEC = { 0:'off', 1:'on' }
    # transmission power T20/T27/T30 (dBm)
    TXPOWER = { 0b00:['20dBm', '27dBm', '30dBm'],
                0b01:['17dBm', '24dBm', '27dBm'],
                0b10:['14dBm', '21dBm', '24dBm'],
                0b11:['10dBm', '18dBm', '21dBm'] }
    

    def __init__(self, PinM0, PinM1, PinAUX, Model='433T20D', Port='U2', Baudrate=9600, Parity='8N1', AirDataRate='2.4k', Address=5, Channel=7, debug=False):
        ''' constructor for ebyte E32 LoRa module '''
        # configuration in dictionary
        self.config = {}
        self.config['model'] = Model               # E32 model (default 868T20D)
        self.config['port'] = Port                 # UART channel on the ESP (default U1)
        self.config['baudrate'] = Baudrate         # UART baudrate (default 9600)
        self.config['parity'] = Parity             # UART Parity (default 8N1)
        self.config['datarate'] = AirDataRate      # wireless baudrate (default 2.4k)
        self.config['address'] = Address           # target address (default 0x0000)
        self.config['channel'] = Channel           # target channel (0-31, default 0x06)
        self.calcFrequency()                       # calculate frequency (min frequency + channel*1 MHz)
        self.config['transmode'] = 0               # transmission mode (default 0 - tranparent)
        self.config['iomode'] = 1                  # IO mode (default 1 = not floating)
        self.config['wutime'] = 0                  # wakeup time from sleep mode (default 0 = 250ms)
        self.config['fec'] = 1                     # forward error correction (default 1 = on)
        self.config['txpower'] = 0                 # transmission power (default 0 = 20dBm/100mW)
        self.PinM0 = PinM0                         # M0 pin number
        self.PinM1 = PinM1                         # M1 pin number
        self.PinAUX = PinAUX                       # AUX pin number
        self.M0 = None                             # instance for M0 Pin (set operation mode)
        self.M1 = None                             # instance for M1 Pin (set operation mode)
        self.AUX = None                            # instance for AUX Pin (device status : 0=busy - 1=idle)
        self.serdev = None                         # instance for UART
        self.debug = debug
        

    def start(self):
        ''' Start the ebyte E32 LoRa module '''
        try:
            # check parameters
            if int(self.config['model'].split('T')[0]) not in LoraE32.FREQ:
                self.config['model'] = '433T20D'
            if self.config['port'] not in LoraE32.PORT:
                self.config['port'] = 'U1'
            if int(self.config['baudrate']) not in LoraE32.BAUDRATE:    
                self.config['baudrate'] = 9600
            if self.config['parity'] not in LoraE32.PARSTR:
                self.config['parity'] = '8N1'
            if self.config['datarate'] not in LoraE32.DATARATE:
                self.config['datarate'] = '2.4k'
            if self.config['channel'] > 31:
                self.config['channel'] = 23
            
            
            # Serial kh?i t?o
            par = LoraE32.PARBIT.get(str(self.config['parity'])[1])
            self.serdev = serial.Serial(LoraE32.PORT.get(self.config['port']), self.config['baudrate'], timeout = 1)
            ser = serial.Serial(port=LoraE32.PORT.get(self.config['port']), baudrate=self.config['baudrate'], parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
            if self.debug:
                print(self.serdev)  
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            self.M0 = GPIO.setup(self.PinM0, GPIO.OUT)
            self.M1 = GPIO.setup(self.PinM1, GPIO.OUT)
            GPIO.output(self.PinM0, GPIO.LOW)
            GPIO.output(self.PinM1, GPIO.HIGH)
            self.AUX = GPIO.setup(self.PinAUX, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            if self.debug:
                print("PinM0:", self.PinM0, "PinM1:", self.PinM1, "PinAUX", self.PinAUX)
            self.setConfig('setConfigPwrDwnSave')
            return "OK"
        
        except Exception as E:
            if self.debug:
                print("error on start UART", E)
            return "NOK"
        
  
    def sendMessage(self, to_address, to_channel, payload, useChecksum=False):
        ''' Send the payload to ebyte E32 LoRa modules in transparent or fixed mode. The payload is a data dictionary to
            accomodate key value pairs commonly used to store sensor data and is converted to a JSON string before sending.
            The payload can be appended with a 2's complement checksum to validate correct transmission.
            - transparent mode : all modules with the same address and channel of the transmitter will receive the payload
            - fixed mode : only the module with this address and channel will receive the payload;
                           if the address is 0xFFFF all modules with the same channel will receive the payload'''
        try:
            # type of transmission
            if (to_address == self.config['address']) and (to_channel == self.config['channel']):
                # transparent transmission mode
                # all modules with the same address and channel will receive the payload
                self.setTransmissionMode(0)
            else:
                # fixed transmission mode
                # only the module with the target address and channel will receive the payload
                self.setTransmissionMode(1)
            # put into wakeup mode (includes preamble signals to wake up device in powersave or sleep mode)
            self.setOperationMode('wakeup')
            # check payload
            if type(payload) != dict:
                print('payload is not a dictionary')
                return 'NOK'
            # encode message
            msg = []
            if self.config['transmode'] == 1:     # only for fixed transmission mode
                msg.append(to_address//256)          # high address byte
                msg.append(to_address%256)           # low address byte
                msg.append(to_channel)               # channel
            js_payload = json.dumps(payload)     # convert payload to JSON string 
            for i in range(len(js_payload)):      # message
                msg.append(ord(js_payload[i]))    # ascii code of character
            if useChecksum:                       # attach 2's complement checksum
                msg.append(int(self.calcChecksum(js_payload), 16))
            # debug
            if self.debug:
                print(msg)
            # wait for idle module
            self.waitForDeviceIdle()
            # send the message
            print("write msg:", msg)
            self.serdev.write(bytes(msg))
            return "OK"
        
        except Exception as E:
            if self.debug:
                print('Error on sendMessage: ',E)
            return "NOK"
        
        
    def receiveMessage(self, from_address, from_channel, useChecksum=False):
        ''' Receive payload messages from ebyte E32 LoRa modules in transparent or fixed mode. The payload is a JSON string
            of a data dictionary to accomodate key value pairs commonly used to store sensor data. If checksumming is used, the
            checksum of the received payload including the checksum byte should result in 0 for a correct transmission.
            - transparent mode : payload will be received if the module has the same address and channel of the transmitter
            - fixed mode : only payloads from transmitters with this address and channel will be received;
                           if the address is 0xFFFF, payloads from all transmitters with this channel will be received'''
        try:
            # type of transmission
            if (from_address == self.config['address']) and (from_channel == self.config['channel']):
                # transparent transmission mode
                # all modules with the same address and channel will receive the message
                self.setTransmissionMode(0)
            else:
                # fixed transmission mode
                # only the module with the target address and channel will receive the message
                self.setTransmissionMode(1)
            # put into normal mode
            self.setOperationMode('normal')
            # receive message
            js_payload = self.serdev.read(self.serdev.inWaiting())
            
            # debug
            if self.debug:
                print(js_payload)
            # did we receive anything ?
            if js_payload == None:
                # nothing
                return { 'msg':None }
            else :
                # decode message
                msg = ''
                for i in range(len(js_payload)):
                    msg += chr(js_payload[i])
                # checksum check
                if useChecksum:
                    cs = int(self.calcChecksum(msg),16)
                    if cs != 0:
                        # corrupt
                        return { 'msg':'corrupt message, checksum ' + str(cs) }
                    else:
                        # message ok, remove checksum
                        msg = msg[:-1]
                # JSON to dictionary
                message = msg #ujson.loads(msg)
                return message
        
        except Exception as E:
            if self.debug:
                print('Error on recvMessage: ',E)
            return "NOK"

    
    def calcChecksum(self, payload):
        ''' Calculates checksum for sending/receiving payloads. Sums the ASCII character values mod256 and returns
            the lower byte of the two's complement of that value in hex notation. '''
        return '%2X' % (-(sum(ord(c) for c in payload) % 256) & 0xFF)


    def reset(self):
        ''' Reset the ebyte E32 Lora module '''
        try:
            # send the command
            res = self.sendCommand('reset')
            # discard result
            return "OK"
          
        except Exception as E:
            if self.debug:
                print("error on reset", E)
            return "NOK"


    def stop(self):
        ''' Stop the ebyte E32 LoRa module '''
        try:
            if self.serdev != None:
                self.serdev.deinit()
                del self.serdev
            return "OK"
            
        except Exception as E:
            if self.debug:
                print("error on stop UART", E)
            return "NOK"
        
    
    def sendCommand(self, command):
        ''' Send a command to the ebyte E32 LoRa module.
            The module has to be in sleep mode '''
        try:
            # put into sleep mode
            self.setOperationMode('sleep')
            # send command
            HexCmd = LoraE32.CMDS.get(command)
            
            if HexCmd in [0xC0, 0xC2]:        # set config to device
                header = HexCmd
                HexCmd = self.encodeConfig()
                HexCmd[0] = header
            else:                             # get config, get version, reset
                HexCmd = [HexCmd]*3
            if self.debug:
                print("Debug HexCmd:", HexCmd)
                
            self.serdev.write(bytes(HexCmd))
            
            # wait for result
            time.sleep(0.5)
            
            # read result
            if command == 'reset':
                result = ''
            else:
                result = self.serdev.read(self.serdev.inWaiting())
                # wait for result
                time.sleep(0.05)
                # debug
                if self.debug:
                    print("Result:", result)
            return result
        
        except Exception as E:
            if self.debug:
                print('Error on sendCommand: ',E)
            return "NOK"
        
        
    
    def getVersion(self):
        ''' Get the version info from the ebyte E32 LoRa module '''
        try:
            # send the command
            result = self.sendCommand('getVersion')
            # check result
            if len(result) != 4:
                return "NOK"
            # decode result
            freq = LoraE32.FREQV.get(hex(result[1]),'unknown')
            # show version
            if result[0] == 0xc3:
                print('================= E32 MODULE ===================')
                print('model       \t%dMhz'%(freq))
                print('version     \t%d'%(result[2]))
                print('features    \t%d'%(result[3]))
                print('================================================')
            return "OK"
        
        except Exception as E:
            if self.debug:
                print('Error on getVersion: ',E)
            return "NOK"
        
    
    def getConfig(self):
        ''' Get config parameters from the ebyte E32 LoRa module '''
        try:
            # send the command
            result = self.sendCommand('getConfig')
            # check result
            if len(result) != 6:
                return "NOK"
            # decode result
            self.decodeConfig(result)
            # show config
            self.showConfig()
            return "OK"

        except Exception as E:
            if self.debug:
                print('Error on getConfig: ',E)
            return "NOK"  
    

    def decodeConfig(self, message):
        ''' decode the config message from the ebyte E32 LoRa module to update the config dictionary '''
        # message byte 0 = header
        header = int(message[0])
        # message byte 1 & 2 = address
        self.config['address'] = int(message[1])*256 + int(message[2])
        # message byte 3 = speed (parity, baudrate, datarate)
        bits = '{0:08b}'.format(message[3])
        self.config['parity'] = LoraE32.PARINV.get(bits[0:2])
        self.config['baudrate'] = LoraE32.BAUDRINV.get(bits[2:5])
        self.config['datarate'] = LoraE32.DATARINV.get(bits[5:])
        # message byte 4 = channel
        self.config['channel'] = int(message[4])
        # message byte 5 = option (transmode, iomode, wutime, fec, txpower)
        bits = '{0:08b}'.format(message[5])
        self.config['transmode'] = int(bits[0:1])
        self.config['iomode'] = int(bits[1:2])
        self.config['wutime'] = int(bits[2:5])
        self.config['fec'] = int(bits[5:6])
        self.config['txpower'] = int(bits[6:])
        
    
    def encodeConfig(self):
        ''' encode the config dictionary to create the config message of the ebyte E32 LoRa module '''
        # Initialize config message
        message = []
        # message byte 0 = header
        message.append(0xC0)
        # message byte 1 = high address
        message.append(self.config['address']//256)
        # message byte 2 = low address
        message.append(self.config['address']%256)
        # message byte 3 = speed (parity, baudrate, datarate)
        bits = '0b'
        bits += LoraE32.PARSTR.get(self.config['parity'])
        bits += LoraE32.BAUDRATE.get(self.config['baudrate'])
        bits += LoraE32.DATARATE.get(self.config['datarate'])
        message.append(int(bits, 0))
        # message byte 4 = channel
        message.append(self.config['channel'])
        # message byte 5 = option (transmode, iomode, wutime, fec, txpower)
        bits = '0b'
        bits += str(self.config['transmode'])
        bits += str(self.config['iomode'])
        bits += '{0:03b}'.format(self.config['wutime'])
        bits += str(self.config['fec'])
        bits += '{0:02b}'.format(self.config['txpower'])
        message.append(int(bits, 0))
        return message
    

    def showConfig(self):
        ''' Show the config parameters of the ebyte E32 LoRa module on the shell '''
        print('=================== CONFIG =====================')
        print('model       \tE32-%s'%(self.config['model']))
        print('frequency   \t%dMhz'%(self.config['frequency']))
        print('address     \t0x%04x'%(self.config['address']))
        print('channel     \t0x%02x'%(self.config['channel']))
        print('datarate    \t%sbps'%(self.config['datarate']))                
        print('port        \t%s'%(self.config['port']))
        print('baudrate    \t%dbps'%(self.config['baudrate']))
        print('parity      \t%s'%(self.config['parity']))
        print('transmission\t%s'%(LoraE32.TRANSMODE.get(self.config['transmode'])))
        print('IO mode     \t%s'%(LoraE32.IOMODE.get(self.config['iomode'])))
        print('wakeup time \t%s'%(LoraE32.WUTIME.get(self.config['wutime'])))
        print('FEC         \t%s'%(LoraE32.FEC.get(self.config['fec'])))
        maxp = LoraE32.MAXPOW.get(self.config['model'][3:6], 0)
        print('TX power    \t%s'%(LoraE32.TXPOWER.get(self.config['txpower'])[maxp]))
        print('================================================')


    def waitForDeviceIdle(self):
        ''' Wait for the E32 LoRa module to become idle (AUX pin high) '''
        count = 0
        # loop for device busy
        while not GPIO.input(self.PinAUX):
            # increment count
            count += 1
            # maximum wait time 100 ms
            if count == 10:
                break
            # sleep for 10 ms
            time.sleep(0.01)
            
        
    
    def calcFrequency(self):
        ''' Calculate the frequency (= minimum frequency + channel * 1MHz)''' 
        # get minimum and maximum frequency
        freqkey = int(self.config['model'].split('T')[0])
        minfreq = LoraE32.FREQ.get(freqkey)[0]
        maxfreq = LoraE32.FREQ.get(freqkey)[2]
        # calculate frequency
        freq = minfreq + self.config['channel']
        if freq > maxfreq:
            self.config['frequency'] = maxfreq
            self.config['channel'] = hex(maxfreq - minfreq)
        else:
            self.config['frequency'] = freq

        
    def setTransmissionMode(self, transmode):
        ''' Set the transmission mode of the E32 LoRa module '''
        if transmode != self.config['transmode']:
            self.config['transmode'] = transmode
            self.setConfig('setConfigPwrDwnSave')
            
            
    def setConfig(self, save_cmd):
        ''' Set config parameters for the ebyte E32 LoRa module '''
        try:
            # send the command
            result = self.sendCommand(save_cmd)
            # check result
            if len(result) != 6:
                print("len != 6")
                return "NOK"
            # debug
            if self.debug:
                # decode result
                self.decodeConfig(result)
                # show config
                self.showConfig()
            return "OK"
        
        except Exception as E:
            if self.debug:
                print('Error on setConfig: ',E)
            return "NOK"  


    def setOperationMode(self, mode):
        ''' Set operation mode of the E32 LoRa module '''
        # get operation mode settings (default normal)
        bits = LoraE32.OPERMODE.get(mode, '00')
        # set operation mode
        GPIO.setup(self.PinM0, int(bits[0]))
        GPIO.setup(self.PinM1, int(bits[1]))
        # wait a moment
        time.sleep(0.05)