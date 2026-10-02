import unittest
from orangepi_comm.protocol import Frame, Framer, ProtocolError, ZERO_SID, crc, decode, encode

SID="1234567890ABCDEF1234567890ABCDEF"
class ProtocolTests(unittest.TestCase):
    def test_crc_and_roundtrip(self):
        self.assertEqual(crc(b"123456789"),0x29B1)
        for frame in [Frame("HELLO",SID,0), Frame("STOP",SID,1), Frame("SET",SID,2,("303","600")),
                      Frame("ACK",SID,2,("SET","MOVING","303","600")),
                      Frame("ACK",SID,1,("STOP","WAIT_START","0","0")),
                      Frame("STATUS",SID,1,("WAIT_START","0","0","0","0","3","5","NONE","100")),
                      Frame("STATUS",SID,2,("MOVING","303","600","12345","-100","3","5","NONE","100"))]:
            self.assertEqual(decode(encode(frame)),frame)
            self.assertEqual(decode(encode(frame)[:-1]+b"\r\n"),frame)
    def test_stream_and_bounds(self):
        frame=Frame("STOP",SID,1); raw=encode(frame); stream=Framer()
        self.assertEqual(list(stream.feed(raw[:12])),[])
        self.assertEqual(list(stream.feed(raw[12:]+raw)),[frame,frame])
        result=list(stream.feed(b"x"*400+b"\n"+raw))
        self.assertIsInstance(result[0],ProtocolError); self.assertEqual(result[1],frame)
        self.assertLessEqual(len(stream.buffer),192)
    def test_bad_input_and_semantics(self):
        for raw in [b"3030\n",b"@2*0000\n",encode(Frame("STOP",SID,1)).replace(b"STOP",b"SET!"),b"x"*193+b"\n"]:
            with self.assertRaises(ProtocolError): decode(raw)
        for frame in [Frame("SET",SID,2,("1001","0")),Frame("SET",SID,2,("-1","0")),
                      Frame("STOP",ZERO_SID,1),Frame("HELLO",SID,1),Frame("STOP",SID,2,("0",)),
                      Frame("ACK",SID,1,("STOP","MOVING","0","0")),
                      Frame("STATUS",SID,2,("IDLE","0","0","100","0","0","5","NONE","100")),
                      Frame("STATUS",SID,2,("IDLE","0","0","0","0","3","101","NONE","100"))]:
            with self.assertRaises(ProtocolError): encode(frame)
