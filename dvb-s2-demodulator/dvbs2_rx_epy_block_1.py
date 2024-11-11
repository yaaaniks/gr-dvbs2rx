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

    def __init__(self, ber_count=100, debug_level = 0):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='BER',   # will show up in GRC
            in_sig=[np.byte],
            out_sig=[np.float32]
        )
        self.message_port_register_out(pmt.intern('ber_counter'))
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        self.measurements = ber_count
        self.debug_level = 0
        
        self.ber: float = 0
        self.byte_err_cnt = 0
        self.frame_count = 0
        self.i_frame_count = 0
        self.ber_cnt = 0
        self.i_ber_cnt = 0
        
        self.least_dummy_cnts = {i: i.value for i in UPID}
        self.most_dummy_cnts = {i: i.value for i in UPID} #dict(map(lambda item: (item.name, item.value), UPID))
        
        self.payload_checkers = {
            UPID.TLM: self.check_tlm,
            UPID.DUMMY: self.check_dummy,
            UPID.FIRST_CHANNEL: self.check_first_channel,
            UPID.SECOND_CHANNEL: self.check_second_channel
        }
        
        self.tlm_cnt = 0
        self.byte_counter = 0
        self.prev_frame_bytes = []
    
    def check_tlm(self, payload: list):
        pass
        
    def check_dummy(self, payload: list):
        bit_error = 0
        for byte in payload:
            self.byte_counter += 1
            if byte == DUMMY_BYTE:
                continue
            bit_error = abs((byte & DUMMY_BYTE).bit_count() - DUMMY_BYTE.bit_count()) + (byte & ~DUMMY_BYTE).bit_count() 
            if self.debug_level != 0:
                print(f"[BER]: Bit error {bit_error} in payload for dummy frame. Needed {DUMMY_BYTE}, it is {byte}. Frame count: {self.i_frame_count}")
            self.ber_cnt += bit_error
            self.i_ber_cnt += bit_error
        return bit_error == 0
        
    def check_first_channel(self, payload: list):
        bit_error = 0
        for byte in payload:
            self.byte_counter += 1
            if byte == 0x00:
                continue
            bit_error = (byte & 0xFF).bit_count() 
            if self.debug_level != 0:
                print(f"[BER]: Bit error {bit_error} in payload for first channel frame. Needed 0x00, it is {byte}. Frame count: {self.i_frame_count}")
            self.ber_cnt += bit_error
            self.i_ber_cnt += bit_error
        return bit_error == 0
    
    def check_second_channel(self, payload: list):
        bit_error = 0
        for byte in payload:
            self.byte_counter += 1
            if byte == 0x00:
                continue
            bit_error = (byte & 0xFF).bit_count()
            if self.debug_level != 0:
                print(f"[BER]: Bit error {bit_error} in payload for second channel frame. Needed 0x00, it is {byte}. Frame count: {self.i_frame_count}")
            self.ber_cnt += bit_error
            self.i_ber_cnt += bit_error
        return bit_error == 0
    
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
            if byte != SYNC:
                self.byte_counter += 1
                continue 
            if chunk.size - index < FRAME_SIZE:
                self.prev_frame_bytes = chunk[index:]
                break
            
            frame = chunk[index:index + FRAME_SIZE]
            frames.append(frame)    
            consume(it, FRAME_SIZE - 1)
            
            self.ber_cnt += self.byte_err_cnt
            self.i_ber_cnt += self.byte_err_cnt
            self.byte_err_cnt = 0
        return frames
    
    def check_header(self, header: list, upid: UPID) -> None:
        least_dummy_cnt = int(header[3] & 0b00001111)
        most_dummy_cnt = int.from_bytes(header[4:8].tobytes(), 'big', signed=True)
        
        uk_field = header[3] & 0b11110000
        
        self.check_least_dummy_counter(least_dummy_cnt, upid)
        self.check_most_dummy_counter(most_dummy_cnt, upid)
        self.byte_counter += 8
        if uk_field.bit_count() != 1 and self.debug_level != 0:
             print(f"[BER]: Error in header for {upid.name}. Frame count: {self.i_frame_count}")
    
    def check_correctness(self, frame: np.ndarray) -> None:
        if self.frame_count > self.measurements:
            self.frame_count = 0
            self.ber = self.ber_cnt / (self.measurements * FRAME_SIZE * 8) 
            #print(f"Current BER = {self.ber}")  
            self.ber_cnt = 0
        if frame[2] in set(item.value for item in UPID):
            upid = UPID(frame[2])
            checker = self.payload_checkers[upid]
            self.check_header(frame[:8], upid)
            checker(frame[8:].tolist())
        else:
            self.ber_cnt += FRAME_SIZE * 8
            self.i_ber_cnt += FRAME_SIZE * 8
        self.i_frame_count += 1
        self.frame_count += 1
    
    def check_least_dummy_counter(self, cnt: int, pid: UPID) -> bool:
        if pid == UPID.TLM:
            return True
        expected_cnt = self.least_dummy_cnts[pid] + 1
        if expected_cnt > 15:
            expected_cnt = 0
        bit_error = abs((cnt & expected_cnt).bit_count() - expected_cnt.bit_count()) + (cnt & ~expected_cnt).bit_count() 
        self.least_dummy_cnts[pid] = cnt
        self.ber_cnt += bit_error
        self.i_ber_cnt += bit_error
        if bit_error != 0 and self.debug_level != 0:
            print(f"[BER]: Bit error {bit_error} in least dummy counter for {pid.name}. Needed {expected_cnt}, it is {cnt}. Frame count: {self.i_frame_count}")
        return bit_error == 0
    
    def check_most_dummy_counter(self, cnt: int, pid: UPID) -> bool:
        if pid == UPID.TLM:
            return True
        if self.most_dummy_cnts[pid] == 0 and cnt == 0:
            return True
        expected_cnt = self.most_dummy_cnts[pid] + 1
        bit_error = abs((cnt & expected_cnt).bit_count() - expected_cnt.bit_count()) + (cnt & ~expected_cnt).bit_count() 
        self.most_dummy_cnts[pid] = cnt
        self.ber_cnt += bit_error
        self.i_ber_cnt += bit_error
        if bit_error != 0 and self.debug_level != 0:
            print(f"[BER]: Bit error {bit_error} in most dummy counter for {pid.name}. Needed {expected_cnt}, it is {cnt}.  Frame count: {self.i_frame_count}")
        return bit_error == 0
    
    def work(self, input_items, output_items):
        self.byte_cnt = self.nitems_read(0)
        frames = self.find_frames(input_items[0])
        for index, frame in enumerate(frames):
            if frame[2] == UPID.TLM.value:
                continue
            self.check_correctness(frame)
        output_items[0][:] = self.ber * 100
        self.message_port_pub(pmt.intern('ber_counter'), pmt.intern(f'{self.i_ber_cnt}'))
        return len(output_items[0])
        

