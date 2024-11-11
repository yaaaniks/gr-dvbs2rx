"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""

import numpy as np
import random
import collections
import struct
import itertools
import enum 
import ctypes

from itertools import cycle, islice
from gnuradio import gr

import pmt

class UPID(enum.Enum):
    TLM = np.byte(0x40)
    DUMMY = np.byte(0x20)
    FIRST_CHANNEL = np.byte(0x30)
    SECOND_CHANNEL = np.byte(0x31)
    

SYM_RATE = 1000000 # mb used for tlm rate check
FRAME_SIZE = 188
SYNC = 0x47
DUMMY_BYTE = np.byte(0xAA)

def consume(iterator, n):
    "Advance the iterator n-steps ahead. If n is none, consume entirely."
    if n is None:
        collections.deque(iterator, maxlen=0)
    else:
        next(islice(iterator, n, n), None)

class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block example - a simple multiply const"""

    def __init__(self, fer_count=1000, debug_level = 0):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='FER',   # will show up in GRC
            in_sig=[np.byte],
            out_sig=[np.float32]
        )
        self.message_port_register_out(pmt.intern('fer_counter'))
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        self.measurements = fer_count
        self.debug_level = debug_level
        
        self.fer: float = 0
        self.byte_err_cnt = 0
        self.frame_count = 0
        self.i_frame_count = 0
        self.fer_cnt = 0
        self.i_fer_cnt = 0
        self.file = "fer.log"
        self.byte_cnt = 0
        self.least_dummy_cnts = {i.name: i.value for i in UPID}
        self.most_dummy_cnts = {i.name: i.value for i in UPID} #dict(map(lambda item: (item.name, item.value), UPID))
        
        self.tlm_cnt = 0
        
        self.prev_frame_bytes = []
        
    
    def find_frames(self, chunk: np.ndarray) -> list:
        frames = []
        if len(self.prev_frame_bytes):
            lost = FRAME_SIZE - self.prev_frame_bytes.size
            prev_frame = np.append(self.prev_frame_bytes, chunk[:lost])
            frames.append(prev_frame)
            self.prev_frame_bytes = []
            chunk = chunk[lost:]
        it = iter(enumerate(chunk))
        temp_err = 0
        for index, byte in it:
            if byte != 0x47: 
                self.byte_err_cnt += 1
                temp_err = self.byte_err_cnt // FRAME_SIZE 
                continue 
            if chunk.size - index < FRAME_SIZE:
                self.prev_frame_bytes = chunk[index:]
                break
            
            frame = chunk[index:index + FRAME_SIZE]
            frames.append(frame)    
            consume(it, FRAME_SIZE - 1)
            self.fer_cnt += temp_err
            temp_err = 0
            self.byte_err_cnt = 0
        return frames
    
    def check_header(self, header: list, upid: UPID) -> bool:
        least_dummy_cnt = int(header[3] & 0b00001111)
        most_dummy_cnt = int.from_bytes(header[4:8].tobytes(), 'big', signed=True)
        uk_field = header[3] & 0b11110000
        b = self.check_least_dummy_counter(least_dummy_cnt, upid) & \
            self.check_most_dummy_counter(most_dummy_cnt, upid) & \
            uk_field.bit_count() == 1
        if uk_field.bit_count() != 1 and self.debug_level != 0:
            print(f"[FER]: Error in header for {upid.name}. Frame count: {self.i_frame_count}\n")
        return b
    
    def check_correctness(self, frame: np.ndarray) -> None:
        correct: bool = False
        
        if self.frame_count >= self.measurements:
            self.fer = self.fer_cnt / self.measurements
            self.fer_cnt = 0
            self.frame_count = 0
        if frame[2] in set(item.value for item in UPID):
            upid = UPID(frame[2])
            correct = self.check_header(frame[:8], upid) & self.check_payload(frame[8:].tolist(), upid)
        if not correct:
            self.i_fer_cnt += 1
            self.fer_cnt += 1
        self.i_frame_count += 1
        self.frame_count += 1
    
    def check_least_dummy_counter(self, cnt: int, pid: UPID) -> bool:
        if pid == UPID.TLM:
            return True
        b = (cnt - self.least_dummy_cnts[pid.name] == -15) or (cnt - self.least_dummy_cnts[pid.name] == 1) 
        if not b and self.debug_level != 0:
            print(f"[FER]: Error in least dummy counter for {pid.name}. Needed {self.least_dummy_cnts[pid.name] + 1}, it is {cnt}. Frame count: {self.i_frame_count}\n")
        self.least_dummy_cnts[pid.name] = cnt
        return b
    
    def check_most_dummy_counter(self, cnt: int, pid: UPID) -> bool:
        if pid == UPID.TLM:
            return True
        b = (cnt - self.most_dummy_cnts[pid.name] == 1) or (cnt - self.most_dummy_cnts[pid.name] == 0)
        if not b and self.debug_level != 0:
             print(f"[FER]: Error in most dummy counter for {pid.name}. Needed {self.most_dummy_cnts[pid.name] + 1}, it is {cnt}. Frame count: {self.i_frame_count}\n")        
        self.most_dummy_cnts[pid.name] = cnt
        return b
    
    def check_payload(self, payload: list, pid: UPID) -> bool:
        if pid == UPID.DUMMY:
            for byte in payload:
                if byte != DUMMY_BYTE and self.debug_level != 0:
                    print(f"[FER]: Error in payload for {pid.name}. Needed {DUMMY_BYTE}, it is {byte}. Frame count: {self.i_frame_count}.\n")
                    return False
            return True
        elif pid == UPID.FIRST_CHANNEL or pid == UPID.FIRST_CHANNEL:
            for byte in payload:
                if byte != 0x00 and self.debug_level != 0:
                    print(f"[FER]: Error in payload for {pid.name}. Needed {DUMMY_BYTE}, it is {byte}. Frame count: {self.i_frame_count}\n")
                    return False
            return True
        else:
            return True
    
    def work(self, input_items, output_items):
        self.byte_cnt = self.nitems_read(0)
        frames = self.find_frames(input_items[0])
        for index, frame in enumerate(frames):
            if frame[2] == UPID.TLM.value:
                continue
            self.check_correctness(frame)
        output_items[0][:] = self.fer * 100
        self.message_port_pub(pmt.intern('fer_counter'), pmt.intern(f'{self.i_fer_cnt}'))
        return len(output_items[0])
        
