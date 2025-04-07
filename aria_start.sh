#!bin/bash

aria streaming start --interface usb --use-ephemeral-certs --profile profile22 
#aria streaming start --interface wifi --device-ip 192.168.1.136 --use-ephemeral-certs --profile profile22
python -m streaming_subscribe