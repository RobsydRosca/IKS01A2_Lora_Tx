# IKS01A2_Lora_Tx

Nucleo TSTM32L476RG,  Acquires data from IKS01A2 and send them with SX1276MB1MAS

The project is based on the NUCLEO STM32L476RG, IKS01A2 sensor board and SX1276MB1MAS LoRa shield.
Its purpose is to realize a point to point communication channel. 
The code has been written by merging the Ping-Pong demo project written by Semtech, see here
https://os.mbed.com/teams/Semtech/code/SX1276PingPong/
and demo project HelloWorld_IKS01A2 by ST
https://os.mbed.com/teams/ST/code/HelloWorld_IKS01A2/

Data acquired by sensors are transmetted on the LoRa device
Another project is simply derived by Ping-Pong demo in order to receive the data.
