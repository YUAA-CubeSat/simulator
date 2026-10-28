"""Real WebSocket transport checks (requires permission to bind localhost)."""
import asyncio
from pathlib import Path
import tempfile
import threading
import unittest

from aiohttp import ClientSession, WSServerHandshakeError
from utilities.live_dashboard import Dashboard, SimulationSession, make_server


class LiveWebSocketTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / 'telemetry.csv'
        self.path.write_text('time_s,boom_length_m\n0,0\n1,1\n')
        self.session = SimulationSession()
        self.server = make_server(Dashboard(self.path), 0, self.session)
        self.worker = threading.Thread(target=self.server.serve_forever)
        self.worker.start()
        self.url = f'http://127.0.0.1:{self.server.server_port}'
        self.client = ClientSession()

    async def asyncTearDown(self):
        await self.client.close()
        self.server.shutdown()
        await asyncio.to_thread(self.worker.join, 5)
        self.server.server_close()
        self.assertFalse(self.worker.is_alive())
        self.directory.cleanup()

    async def receive_until(self, ws, predicate):
        async with asyncio.timeout(5):
            while True:
                payload = await ws.receive_json()
                await ws.send_json({'type': 'ack'})
                if predicate(payload):
                    return payload

    async def test_start_then_numeric_stream_append_pause_and_reconnect(self):
        original = self.path.read_text()
        async with self.client.ws_connect(self.url + '/ws', origin=self.url) as ws:
            initial = await ws.receive_json()
            self.assertEqual(initial['session']['state'], 'waiting')
            await ws.send_json({'type': 'subscribe', 'ids': ['boom:0'], 'interval': .1})
            await ws.send_json({'type': 'ack'})
            waiting = await self.receive_until(ws, lambda p: 'boom:0' in p.get('plots', {}))
            self.assertIn('Start simulation', waiting['plots']['boom:0']['error'])
            self.assertEqual(self.path.read_text(), original)
            await ws.send_json({'type': 'start', 'token': initial['session']['token']})
            await self.receive_until(ws, lambda p: p['session']['state'] == 'starting')
            self.assertTrue(self.session.start_requested.is_set())
            self.session.ready()
            data = await self.receive_until(ws, lambda p: 'data' in p.get('plots', {}).get('boom:0', {}))
            self.assertEqual(data['plots']['boom:0']['data'][0]['y'], [0, 1])
            self.assertNotIn('image', data['plots']['boom:0'])
            unchanged = await ws.receive_json()
            self.assertEqual(unchanged['plots'], {})
            await ws.send_json({'type': 'ack'})
            with self.path.open('a') as stream:
                stream.write('2,0.5\n')
            data = await self.receive_until(ws, lambda p: 'data' in p.get('plots', {}).get('boom:0', {}))
            self.assertEqual(data['plots']['boom:0']['data'][0]['y'], [0, 1, .5])
            await ws.send_json({'type': 'subscribe', 'ids': ['boom:0'], 'paused': True, 'interval': .1})
            await self.receive_until(ws, lambda p: not p['plots'])
            self.session.finish()
            await self.receive_until(ws, lambda p: p['session']['state'] == 'completed')
        async with self.client.ws_connect(self.url + '/ws', origin=self.url) as ws:
            await ws.send_json({'type': 'subscribe', 'ids': ['boom:0'], 'interval': .1})
            data = await self.receive_until(ws, lambda p: 'data' in p.get('plots', {}).get('boom:0', {}))
            self.assertEqual(data['plots']['boom:0']['data'][0]['x'], [0, 1, 2])
            self.assertEqual(data['session']['state'], 'completed')

    async def test_backpressure_and_request_validation(self):
        self.session.ready()
        async with self.client.ws_connect(self.url + '/ws', origin=self.url) as ws:
            await ws.receive_json()  # deliberately withhold acknowledgement
            with self.assertRaises(asyncio.TimeoutError):
                await asyncio.wait_for(ws.receive_json(), .4)
            await ws.send_json({'type': 'subscribe', 'ids': ['not-a-plot']})
            error = await ws.receive_json()
            self.assertEqual(error['type'], 'error')
            await ws.send_json({'type': 'ack'})
            self.assertEqual((await ws.receive_json())['type'], 'update')
        with self.assertRaises(WSServerHandshakeError):
            await self.client.ws_connect(self.url + '/ws', origin='https://unrelated.example')

    async def test_local_assets_and_removed_image_api(self):
        for path in ('/', '/live_dashboard.js', '/plotly.js'):
            async with self.client.get(self.url + path) as response:
                self.assertEqual(response.status, 200)
                self.assertTrue(await response.read())
        async with self.client.get(self.url + '/api/plots?id=boom:0') as response:
            self.assertEqual(response.status, 404)


if __name__ == '__main__':
    unittest.main()
