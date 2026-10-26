import subprocess
import tempfile
import unittest
from pathlib import Path

from cubesat_sim import protocol as w

ROOT = Path(__file__).resolve().parents[1]


class BinaryProtocolTests(unittest.TestCase):
    def test_golden_request(self):
        self.assertEqual(w.encode(w.T, [1,1000,10]),
                         bytes.fromhex('03 0d 01 01 01 03 e8 03 01 02 0a 03 a9 17 00'))

    def test_quantization_and_bounds(self):
        self.assertEqual(w.quantize(-1.5e-6, 1e6), -2)
        self.assertEqual(w.quantize(.5e-6, 1e6), 1)
        with self.assertRaises(ValueError):
            w.encode(w.G, [0, 8388608, 0, 0])
        with self.assertRaises(ValueError):
            w.quantize(float('nan'), 1)

    def test_corruption_length_and_version(self):
        packet = w.encode(w.T, [0xffffffff, 0, 10])
        self.assertEqual(w.decode(packet[:-1]), (w.T, [0xffffffff, 0, 10]))
        for i in range(len(packet)-1):
            corrupt = bytearray(packet[:-1]); corrupt[i] ^= 0x80
            with self.assertRaises(ValueError):
                w.decode(corrupt)
        for packet in (b'', b'x'*65, b'\x05\x01', b'\0'):
            with self.assertRaises(ValueError):
                w.decode(packet)

    def test_c_codecs_match_python_and_resynchronize(self):
        headers = [ROOT/'protocol/adcs_wire.h',
                   ROOT.parent/'YUAA_cubesat/Core/Inc/adcs_wire.h',
                   ROOT.parent/'obc/espf/core/services/adcs_io/inc/adcs_wire.h']
        self.assertTrue(all(p.read_bytes() == headers[0].read_bytes() for p in headers if p.exists()))
        source = r'''
#include "adcs_wire.h"
#include <stdio.h>
int main(void) {
    WireDecoder d={0}; WireFrame f; int b;
    while((b=getchar())!=EOF) if(Wire_Feed(&d,(uint8_t)b,&f)==1) {
        uint8_t out[WIRE_MAX]; size_t n=Wire_Encode(out,sizeof out,f.type,f.payload);
        if(!n || fwrite(out,1,n,stdout)!=n) return 2;
    }
    return 0;
}
'''
        fixtures = [
            (w.M, [0xffffffff, 255, -8388608, 8388607, 0]),
            (w.C, [0, 0, -2147483648, 2147483647, 1000000]),
            (w.G, [123, -1, 0, 10000]),
            (w.S, [42, 1, -32768, 32767, 0]),
            (w.B, [42, 1000, -2147483648]),
            (w.T, [0xffffffff, 0xffffffff, 65535]),
            (w.A, [42, -200000, 0, 200000, 3000, 123, 1, 0, -12345, 54321]),
            (w.U, [42, -200000, 0, 200000, 3000, 123, 1, 0, -12345, 54321, 0xffffffff, 9, 100, 0xffffff00, 1])]
        frames = b''.join(w.encode(kind, values) for kind, values in fixtures)
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/'codec.c'; src.write_text(source)
            exe=Path(tmp)/'codec'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                            '-I'+str(headers[0].parent),str(src),'-o',str(exe)],check=True)
            # Oversize, partial/truncated frames, and CRC errors are discarded at delimiters.
            bad = bytearray(w.encode(w.T,[99,0,10])); bad[-2] ^= 1
            result=subprocess.run([str(exe)], input=b'x'*100+b'\0'+bytes(bad)+b'\x05\x01\0'+frames,
                                  capture_output=True,check=True)
            self.assertEqual(result.stdout,frames)
        for frame, (kind, values) in zip(frames.split(b'\0'), fixtures):
            self.assertEqual(w.decode(frame),(kind,values))


    def test_both_firmware_payload_adapters(self):
        source = r"""
#include "sim_protocol.h"
#include <stdio.h>
#include <assert.h>
int main(void) {
    WireDecoder d={0}; WireFrame f; AdcsSensors sensors={0}; AdcsStep step; int byte;
    while((byte=getchar())!=EOF) if(Wire_Feed(&d,(uint8_t)byte,&f)==1) {
        SimParseResult result=Sim_ParseFrame(&f,&sensors,&step);
        if(f.type==WIRE_G && f.length==14 && f.payload[4]==1) assert(result==SIM_IGNORED);
        if(result==SIM_STEP) {
            assert(step.sensors.acceleration[1][0]==-123456789);
            assert(step.sensors.sun[2]==32767 && step.sensors.sun_valid);
            assert(step.sensors.battery.soc_permille==800 && step.sensors.battery.power_mW==-750);
            AdcsCommand c={0}; c.output.tick=step.tick; c.output.boom=3000;
            for(unsigned i=0;i<3;++i) {
                c.output.dipole[i]=step.sensors.magnetic[0][i];
                c.rates.angular_rate[i]=step.sensors.gyro[i]*1e-6f;
            }
            c.rates.valid=step.sensors.have_gyro; c.rates.time_ms=step.sensors.gyro_time;
            char out[WIRE_MAX]; int n=Sim_FormatResponse(out,sizeof out,&c);
            if(!n || fwrite(out,1,n,stdout)!=(size_t)n) return 2;
            WireSync sync={.tick=7,.host_ms=200,.local_ms=500,.valid=true};
            n=Sim_FormatStream(out,sizeof out,&c,510,&sync);
            if(!n || fwrite(out,1,n,stdout)!=(size_t)n) return 3;
        }
    }
    return 0;
}
"""
        obc=ROOT.parent/'obc/espf'
        yuaa=ROOT.parent/'YUAA_cubesat/Core'
        if not yuaa.exists() or not obc.exists():
            self.skipTest('cross-project payload checks require sibling firmware checkouts')
        targets=[(yuaa/'Src/sim_protocol.c', [yuaa/'Inc']),
                 (obc/'core/services/adcs_io/src/sim_protocol.c',
                  [obc/'core/services/adcs_io/inc', obc/'core/lib/adcs_common/inc', obc/'config/adcs'])]
        frames=b''.join(w.encode(kind,values) for kind,values in [
            (w.M,[100,0,-200000,0,200000]), (w.C,[100,1,-123456789,0,0]),
            (w.G,[100,999,999,999]), # legacy still accepted
            (w.G,[101,0,10000,-20000,30000]),
            (w.G,[102,1,777,888,999]), # secondary cannot overwrite primary/time
            (w.G,[103,255,777,888,999]), # invalid ID
            (w.S,[101,1,0,0,32767]),
            (w.B,[101,800,-750]),
            # Semantically invalid but CRC-valid fields must not overwrite state.
            (w.M,[101,255,1,2,3]), (w.B,[101,1001,0]), (w.S,[101,2,0,0,0]),
            (w.T,[99,101,0]), (w.T,[0xffffffff,101,10])])
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/'adapter.c'; src.write_text(source)
            for index,(implementation,includes) in enumerate(targets):
                exe=Path(tmp)/str(index)
                subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                                *('-I'+str(p) for p in includes),str(src),str(implementation),'-lm','-o',str(exe)],check=True)
                result=subprocess.run([str(exe)], input=frames,capture_output=True,check=True)
                a,u,_=result.stdout.split(b'\0')
                expected=[0xffffffff,-200000,0,200000,3000,101,1,10000,-20000,30000]
                self.assertEqual(w.decode(a), (w.A,expected))
                self.assertEqual(w.decode(u), (w.U,expected+[510,7,200,500,1]))

    def test_indexed_gyro_codec_and_rust_vectors(self):
        for sensor_id in (0, 1):
            values = [123, sensor_id, -8388608, 8388607, -1]
            frame = w.encode(w.G, values)
            self.assertEqual(len(frame), 19)
            self.assertEqual(w.decode(frame[:-1]), (w.G, values))
            vectors = ROOT.parent / 'sensor-hitl/tests/wire-vectors.txt'
            if vectors.exists():
                line = next(line for line in vectors.read_text().splitlines()
                            if line.startswith(f'G{sensor_id} '))
                self.assertEqual(bytes.fromhex(line.split(' ', 1)[1]), frame)

    def test_absolute_pacing_and_overrun(self):
        from unittest.mock import patch
        from cubesat_sim.pacing import pace
        with patch('cubesat_sim.pacing.time.monotonic', side_effect=[100.009,100.010]), \
                patch('cubesat_sim.pacing.time.sleep') as sleep:
            pace(100, .010, .25)
            self.assertAlmostEqual(sleep.call_args.args[0], .001)
        with patch('cubesat_sim.pacing.time.monotonic', return_value=101):
            with self.assertRaisesRegex(RuntimeError, 'deadline missed'):
                pace(100, .010, .25)

    def test_wire_budget(self):
        # Typical 100 Hz cycle, without the 1 Hz sun update.
        sizes=[len(w.encode(k,[0]*len(w.FIELDS[k]))) for k in (w.M,w.M,w.C,w.C,w.G,w.B,w.T,w.A)]
        self.assertEqual(sum(sizes),164)
        self.assertLess(sum(sizes)*10/921600,.002)
