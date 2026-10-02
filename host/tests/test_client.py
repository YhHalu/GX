"""Execute the production C protocol through ctypes, simulate only HAL/UART."""
import ctypes
import os
from pathlib import Path
import threading
import time
import unittest

from orangepi_comm import Client, CommunicationError
from orangepi_comm.protocol import Frame, Framer, decode, encode

CALLBACK=ctypes.CFUNCTYPE(ctypes.c_int,ctypes.c_char_p)
LIBRARY=os.environ.get("GX_PROTOCOL_BRIDGE")

class Peer:
    def __init__(self):
        self.lib=ctypes.CDLL(LIBRARY)
        self.lib.Motor_CommandInit.argtypes=[CALLBACK,CALLBACK]
        self.lib.Motor_CommandReceive.argtypes=[ctypes.c_uint8,ctypes.c_uint32]
        self.lib.Motor_CommandPoll.argtypes=[ctypes.c_uint32]
        self.lib.Motor_CommandSetHealthy.argtypes=[ctypes.c_uint8]
        self.lib.Motor_CommandButtonPoll.argtypes=[ctypes.c_uint8,ctypes.c_uint32]
        self.lib.Bridge_Target.argtypes=[ctypes.c_uint]
        self.lib.Bridge_Target.restype=ctypes.c_float
        self.lock=threading.RLock()
        self.start=time.monotonic()
        self.offset=0
        self.rx=bytearray()
        self.frames=[]
        self.sent=Framer()
        self.closed=False
        self.drop_set_ack=False
        self.drop_all=False
        self.chunk=256
        self.write_chunk=1000
        self.fail_read=False
        self.fail_write=False
        self.corrupt_info=False
        self.reset_after_info=False
        self.status_before_stop_ack=False
        self.status_before_set_ack=False
        self.corrupt_set_ack=False
        self.auto_start=True
        self.button=0
        self.button_at=None
        self.critical=CALLBACK(self.capture)
        self.status=CALLBACK(self.capture)
        self.reset()
    def tick(self):
        return (int((time.monotonic()-self.start)*1000)+self.offset)&0xFFFFFFFF
    def capture(self,raw):
        frame=decode(raw)
        if self.auto_start and frame.kind=="ACK" and frame.args[:2]==("STOP","WAIT_START"):
            self.button_at=time.monotonic()
        if self.corrupt_set_ack and frame.kind=="ACK" and frame.args[0]=="SET":
            raw=encode(Frame("ACK",frame.sid,frame.seq,("SET","IDLE","0","0")))
        if self.status_before_stop_ack and frame.kind=="ACK" and frame.args[0]=="STOP" and frame.seq==1:
            self.rx.extend(encode(Frame("STATUS",frame.sid,0,
                                       ("BOUND","0","0","0","0","3","0","NONE",str(self.tick())))))
        if self.status_before_set_ack and frame.kind=="ACK" and frame.args[0]=="SET":
            state, p1, p2 = ("IDLE","0","0") if frame.seq==2 else frame.args[1:]
            self.rx.extend(encode(Frame("STATUS",frame.sid,frame.seq-1,
                                       (state,p1,p2,"0","0","3","0","NONE",str(self.tick())))))
        if self.status_before_set_ack and frame.kind=="STATUS":
            return 1  # Force only telemetry queued before the latest SET ACK.
        if self.corrupt_info and frame.kind=="INFO":
            raw=encode(Frame("INFO",frame.sid,0,("WRONGFW",*frame.args[1:])))
        if not self.drop_all and not (self.drop_set_ack and frame.kind=="ACK" and frame.args[0]=="SET"):
            self.rx.extend(raw)
        if frame.kind=="INFO" and self.reset_after_info:
            self.reset_after_info=False
            self.lib.Motor_CommandInit(self.critical,self.status)
        return 1
    def reset(self):
        with self.lock:
            self.lib.Motor_CommandInit(self.critical,self.status)
            self.button_at=None
    def factory(self,*args,**kwargs):
        assert kwargs["baudrate"]==115200 and kwargs["exclusive"] and kwargs["write_timeout"]==0.05
        self.closed=False
        return self
    def write(self,raw):
        with self.lock:
            if self.fail_write:
                raise OSError("simulated write failure")
            raw=raw[:self.write_chunk]
            self.lib.Motor_CommandPoll(self.tick())
            for byte in raw:
                self.lib.Motor_CommandReceive(byte,self.tick())
            for frame in self.sent.feed(raw):
                if isinstance(frame,Frame): self.frames.append(frame)
            return len(raw)
    def read(self,size):
        time.sleep(0.003)
        with self.lock:
            if self.fail_read: raise OSError("simulated read failure")
            if self.button_at is not None:
                self.button=int(time.monotonic()-self.button_at >= 0.040)
            self.lib.Motor_CommandButtonPoll(self.button,self.tick())
            self.lib.Motor_CommandPoll(self.tick())
            count=min(size,self.chunk,len(self.rx))
            data=bytes(self.rx[:count]); del self.rx[:count]
            return data
    def reset_input_buffer(self):
        with self.lock: self.rx.clear()
    def reset_output_buffer(self): pass
    def close(self): self.closed=True
    def targets(self):
        with self.lock: return tuple(self.lib.Bridge_Target(i) for i in range(2))
    def inject(self,frame):
        with self.lock: self.rx.extend(encode(frame))
    def advance(self,ms):
        with self.lock:
            self.offset+=ms
            self.lib.Motor_CommandPoll(self.tick())

def wait_for(predicate,timeout=1.5):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate(): return
        time.sleep(0.005)
    raise AssertionError("condition not reached")

@unittest.skipUnless(LIBRARY and Path(LIBRARY).exists(),"run tools/tests/run_native.py for the C bridge")
class ClientTests(unittest.TestCase):
    def setUp(self):
        self.peer=Peer()
        self.client=Client("SIMULATED",serial_factory=self.peer.factory)
    def tearDown(self):
        try: self.client.close()
        except CommunicationError: pass
    def connect(self):
        self.client.connect()
        self.client.wait_for_start(1.5)
        self.assertEqual(self.client.state,"IDLE")
        self.assertEqual(self.peer.targets(),(0,0))
    def test_handshake_fragmentation_and_status(self):
        self.peer.chunk=3; self.peer.write_chunk=2
        self.connect()
        wait_for(lambda:self.client._status is not None)
        status=self.client.get_status()
        self.assertEqual(status["measured_cps"],[0,0]); self.assertEqual(status["valid_bits"],3)
        status["measured_cps"][0]=99
        self.assertEqual(self.client.get_status()["measured_cps"],[0,0])
    def test_motion_stop_and_old_ack(self):
        self.connect(); self.client.set_speed(30.3,60)
        wait_for(lambda:self.peer.targets()[1]==60)
        old=next(f for f in self.peer.frames if f.kind=="SET")
        self.client.stop()
        stopped=len(self.peer.frames)
        self.peer.inject(Frame("ACK",old.sid,old.seq,("SET","MOVING",*old.args)))
        time.sleep(0.15)
        self.assertEqual(self.peer.targets(),(0,0))
        self.assertFalse(any(f.kind=="SET" for f in self.peer.frames[stopped:]))
        self.assertEqual(self.client.state,"IDLE")
    def test_pre_stop_status_is_not_exposed_after_handshake(self):
        self.peer.status_before_stop_ack=True
        self.connect()
        try:
            status=self.client.get_status()
        except CommunicationError:
            pass  # No post-STOP measurement has arrived yet.
        else:
            self.assertGreaterEqual(status["sequence"],1)
            self.assertEqual(status["state"],"IDLE")
        wait_for(lambda:self.client._status is not None)
        self.assertEqual(self.client.get_status()["state"],"IDLE")
    def test_app_expiry_and_explicit_recovery(self):
        self.connect(); old_sid=self.client.sid
        self.client.set_speed(20,0)
        wait_for(lambda:self.peer.targets()[0]==20)
        wait_for(lambda:self.client.state=="FAULT")
        wait_for(lambda:self.peer.targets()==(0,0))
        with self.assertRaises(CommunicationError): self.client.set_speed(20,0)
        self.client.recover(); self.assertNotEqual(old_sid,self.client.sid)
        self.assertEqual(self.client.state,"WAIT_START")
        time.sleep(0.15); self.assertEqual(self.peer.targets(),(0,0))
    def test_inflight_set_telemetry_remains_available(self):
        self.connect(); self.peer.status_before_set_ack=True
        deadline=time.monotonic()+0.55
        while time.monotonic()<deadline:
            self.client.set_speed(20,0); time.sleep(0.04)
        snapshot=self.client.get_status()
        self.assertEqual(snapshot["state"],"MOVING")
        self.assertEqual(snapshot["target_permille"],[200,0])
        self.assertGreaterEqual(snapshot["sequence"],2)
        self.assertLess(snapshot["sequence"],self.client._confirmed)
        self.assertEqual(snapshot["valid_bits"],3)
    def test_ack_loss_even_with_live_status(self):
        self.connect(); self.peer.drop_set_ack=True
        self.client.set_speed(20,0)
        deadline=time.monotonic()+0.4
        while time.monotonic()<deadline and self.client.state!="FAULT":
            self.client.set_speed(20,0); time.sleep(0.05)
        wait_for(lambda:self.client.state=="FAULT")
        wait_for(lambda:self.peer.targets()==(0,0))
        self.assertIn("ACK_TIMEOUT",self.client.fault)
    def test_mismatched_ack_keeps_transport_for_safety_stop(self):
        self.connect(); self.peer.corrupt_set_ack=True
        self.client.set_speed(20,0)
        wait_for(lambda:self.client.state=="FAULT")
        wait_for(lambda:self.peer.targets()==(0,0))
        self.assertTrue(self.client._thread.is_alive())
        self.assertIn("PROTOCOL:",self.client.fault)
        self.assertTrue(any(f.kind=="STOP" and f.seq>1 for f in self.peer.frames))
    def test_reset_rejects_previous_motion(self):
        self.connect(); self.client.set_speed(20,0)
        wait_for(lambda:self.peer.targets()[0]==20)
        old=self.client.sid
        self.peer.reset()
        wait_for(lambda:self.client.state=="FAULT")
        self.assertEqual(self.peer.targets(),(0,0))
        self.client.recover(); self.assertNotEqual(old,self.client.sid)
        time.sleep(0.15); self.assertEqual(self.peer.targets(),(0,0))
    def test_rx_loss_lower_lease(self):
        self.connect(); self.client.set_speed(20,0)
        wait_for(lambda:self.peer.targets()[0]==20)
        self.peer.fail_read=True
        wait_for(lambda:self.client.state=="FAULT")
        self.peer.advance(501)
        self.assertEqual(self.peer.targets(),(0,0))
    def test_invalid_api_and_zero_only(self):
        self.connect()
        for value in [-1,101,float("inf"),float("nan"),True,"20"]:
            with self.assertRaises(ValueError): self.client.set_speed(value,0)
        self.client.zero_only=True
        with self.assertRaises(ValueError): self.client.set_speed(0.1,0)
        self.assertFalse(any(f.kind=="SET" for f in self.peer.frames))
    def test_bad_metadata_cannot_arm(self):
        self.peer.corrupt_info=True
        with self.assertRaises(CommunicationError): self.client.connect()
        self.assertFalse(any(f.kind in {"STOP","SET"} for f in self.peer.frames))
    def test_stop_preempts_unacknowledged_set(self):
        self.connect(); self.peer.drop_set_ack=True
        self.client.set_speed(20,0)
        wait_for(lambda:self.peer.targets()[0]==20)
        self.client.stop(); time.sleep(0.15)
        self.assertEqual(self.peer.targets(),(0,0)); self.assertEqual(self.client.state,"IDLE")
        self.assertEqual(self.peer.frames[-1].kind,"STOP")
    def test_write_failure_does_not_retry_nonzero(self):
        self.connect(); self.peer.write_chunk=2
        self.peer.fail_write=True
        self.client.set_speed(20,0)
        wait_for(lambda:self.client.state=="FAULT")
        self.peer.advance(501)
        self.assertEqual(self.peer.targets(),(0,0))
        self.assertFalse(any(f.kind=="SET" for f in self.peer.frames))
    def test_reset_during_handshake_retries_new_session(self):
        self.peer.reset_after_info=True
        self.connect()
        sessions=[f.sid for f in self.peer.frames if f.kind=="HELLO"]
        self.assertGreaterEqual(len(sessions),2)
        self.assertNotEqual(sessions[0],sessions[1])
    def test_sequence_exhaustion_uses_reserved_stop(self):
        self.connect()
        with self.client._lock: self.client._seq=0xFFFFFFFF-4
        self.client.set_speed(20,0)
        wait_for(lambda:self.client.state=="FAULT")
        wait_for(lambda:any(f.kind=="STOP" and f.seq>1 for f in self.peer.frames))
        self.assertEqual(self.peer.targets(),(0,0))
        self.assertFalse(any(f.kind=="SET" for f in self.peer.frames))

    def test_button_blocks_motion_without_queuing(self):
        self.peer.auto_start=False
        self.client.connect()
        self.assertTrue(self.client.info["start_button"])
        self.assertEqual(self.client.state,"WAIT_START")
        with self.assertRaises(CommunicationError): self.client.set_speed(20,0)
        with self.assertRaises(CommunicationError): self.client.wait_for_start(0.05)
        self.assertFalse(any(f.kind=="SET" for f in self.peer.frames))
        time.sleep(0.05); self.peer.button=1
        self.client.wait_for_start(1)
        self.assertEqual(self.peer.targets(),(0,0))
        self.assertFalse(any(f.kind=="SET" for f in self.peer.frames))
        self.client.set_speed(20,0); wait_for(lambda:self.peer.targets()[0]==20)
        self.client.stop(); self.assertEqual(self.client.state,"IDLE")

    def test_reconnect_requires_release_and_new_press(self):
        self.connect()
        self.peer.auto_start=False; self.peer.button_at=None; self.peer.button=1
        self.client.recover()
        time.sleep(0.15)
        self.assertEqual(self.client.state,"WAIT_START")
        self.peer.button=0; time.sleep(0.05); self.peer.button=1
        self.client.wait_for_start(1)
        self.assertEqual(self.peer.targets(),(0,0))

    def test_button_wait_aborts_on_lower_fault(self):
        self.peer.auto_start=False; self.client.connect()
        self.peer.inject(Frame("ERR",self.client.sid,1,("UART_RX","FAULT")))
        wait_for(lambda:self.client.state=="FAULT")
        with self.assertRaises(CommunicationError): self.client.wait_for_start(1)
        self.assertEqual(self.peer.targets(),(0,0))
