#!/usr/bin/env python3

import usb.core
import usb.util
import functools
from time import sleep
from random import randrange

def to_hex(l):
    return " ".join([hex(i) for i in l])

def init():
    # find our device
    dev = usb.core.find(idVendor=0x1a86, idProduct=0x5537)

    # was it found?
    if dev is None:
        raise ValueError('Device not found')

    # get an endpoint instance
    cfg = dev.get_active_configuration()
    intf = cfg[(0,0)]

    ep_in = usb.util.find_descriptor(
        intf,
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_IN)
    print(str(ep_in))
    assert ep_in is not None

    ep_out = usb.util.find_descriptor(
        intf,
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_OUT)
    print(str(ep_out))

    assert ep_out is not None

    return (ep_in, ep_out)

ep_in, ep_out = init()

def sequence(ret=False):
    seq = functools.reduce(lambda a,b: a + b, [i.to_bytes(4, 'little') for i in range(1024)])
    ep_out.write(seq)
    if ret: return to_hex(seq)

def bytesequence(ret=False):
    seq = [i & 0xff for i in range(4096)]
    ep_out.write(seq)
    if ret: return to_hex(seq)

def patterns():
    zeroes()
    ones()
    fives()
    aas()

def aas(len=4096):
    ep_out.write([0xaa for i in range(len)])

def fives(len=4096):
    ep_out.write([0x55 for i in range(len)])

def ones(len=4096):
    ep_out.write([0xff for i in range(len)])

def zeroes(len=4096):
    ep_out.write([0x00 for i in range(len)])

def rand(len=4096):
    r = [randrange(0, 256) for _ in range(len)]
    ep_out.write(r)
    return to_hex(r)

def read(len=4096):
    return " ".join([hex(i) for i in ep_in.read(len)]) 

from os import system

def loop(length=4096):
    while True:
        r = bytesequence(length).replace(" ", "\n")
        #sleep(0.0001);
        s = read(length).replace(" ", "\n")
        if r == s:
            print("OK")
        else:
            l = str(len(s.split('\n')))
            msg = "ERR: length " + l
            with open("/tmp/foo", 'w') as f:
                f.write(r)
            with open("/tmp/bar", 'w') as f:
                f.write(s)
            system("meld /tmp/foo /tmp/bar")
            print(msg)
