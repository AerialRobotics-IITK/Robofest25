#!/usr/bin/env python3
import socket

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    finally:
        s.close()

def update_json5():
    my_ip =get_local_ip() 
    nf = []
    with open("/root/router_config.json5") as f:
        nf = f.readlines()
    nf = [l for l in nf if my_ip not in l]
    with open("/root/router_config.json5",'w') as f:
        f.writelines(nf)

if __name__=="__main__":
    update_json5()






