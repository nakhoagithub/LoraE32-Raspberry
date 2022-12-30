# LoraE32 Raspberry
```
Source: https://github.com/effevee
Edit: Anh Khoa
```

[Download Datasheet E32](https://www.ebyte.com/en/data-download.html?id=214&pid=211#load)

### Tutorial

- Install Python for Raspberry
- Install these packages into the Raspberry:
```cmd
pip install pyserial
```
```cmd
pip install RPi.GPIO
```
```cmd
git clone https://github.com/nakhoagithub/LoraE32-Raspberry.git
```
```cmd
cd LoraE32-Raspberry
```
- Create main.py
```python
from lora import LoraE32
import time

M0pin = 17
M1pin = 27

# Port
# 'U1': /dev/ttyS0 or /dev/ttyAMA0
# 'U2': /dev/ttyUSB0 
# 'U3': /dev/ttyUSB1

lora = LoraE32(M0pin, M1pin, Port='U2', Address=1, Channel=7, debug=True)
lora.start()
lora.showConfig()

from_address = 5
from_channel = 7

while True:
  message = lora.receiveMessage(from_address, from_channel, useChecksum=True)
  lora.showConfig()
  #message = lora.sendMessage(5, 7, {"a": "5"}, useChecksum=True)
  #message = lora.sendMessage(6, 7, {"a": s}, useChecksum=True)
      
  print("Message:", message) #, message1)
  time.sleep(2)

```

- Run code
```cmd
python main.py
```

### Good luck!