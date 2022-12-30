from lora import LoraE32
import time

M0pin = 17
M1pin = 27

lora = LoraE32(M0pin, M1pin, Port='U2', Address=1, Channel=7, debug=True)
lora.start()
lora.showConfig()

from_address = 5
from_channel = 7

while True:
  message = lora.receiveMessage(from_address, from_channel, useChecksum=True)
  print("Message:", message)
  time.sleep(2)
