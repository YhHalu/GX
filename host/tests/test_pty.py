"""Real pySerial + Linux PTY + the production C parser. No physical devices."""
import importlib.util
import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import threading
import time
import tty
import unittest

from orangepi_comm import Client
from test_client import Peer, wait_for, LIBRARY

@unittest.skipUnless(sys.platform=="linux" and importlib.util.find_spec("serial") and LIBRARY and Path(LIBRARY).exists(),
                     "Linux pySerial and C bridge are required")
class PtyTests(unittest.TestCase):
    def test_real_serial_module_and_zero_cli(self):
        peer=Peer()
        master,slave=os.openpty()
        tty.setraw(slave)
        path=os.ttyname(slave)
        end=threading.Event()
        errors=[]
        def serve():
            try:
                while not end.is_set():
                    if select.select([master],[],[],0.003)[0]:
                        data=os.read(master,1024)
                        peer.write(data)
                    output=peer.read(256)
                    while output:
                        n=os.write(master,output)
                        output=output[n:]
            except Exception as error:
                errors.append(error)
        worker=threading.Thread(target=serve,daemon=True)
        worker.start()
        try:
            with tempfile.TemporaryDirectory(prefix="gx-pty-") as tmp:
                with Client(path,log_path=Path(tmp)/"client.jsonl") as client:
                    client.wait_for_start(1.5)
                    client.set_speed(20,30)
                    wait_for(lambda:peer.targets()==(20,30))
                    client.stop()
                    self.assertEqual(peer.targets(),(0,0))
                result=subprocess.run([sys.executable,"-B","-m","orangepi_comm","--port",path,"--zero-only",
                                       "--log",str(Path(tmp)/"zero.jsonl"),"zero-test","--seconds","0.3"],
                                      capture_output=True,text=True,timeout=5)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                self.assertIn('"zero_test_passed": true',result.stdout)
                self.assertGreater((Path(tmp)/"zero.jsonl").stat().st_size,0)
                self.assertEqual(peer.targets(),(0,0))
        finally:
            end.set(); worker.join(1)
            os.close(master); os.close(slave)
        self.assertEqual(errors,[])
