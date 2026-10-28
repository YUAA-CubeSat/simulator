"""Local, draggable live dashboard backed by the existing plotting utilities."""
from __future__ import annotations


# Allow direct execution from any working directory as well as package execution.
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import asyncio
import socket

import numpy as np

from utilities.live_traces import Traces
from collections import deque
import csv
import json
import secrets
import traceback
from pathlib import Path
import threading
import time
import webbrowser

# Stable graph IDs preserve layouts from the original viewer.
FAMILIES = {
    'body': ('plot_angular_velocity', ['--frame', 'body'], ['X rate', 'Y rate', 'Z rate', 'Rate magnitude']),
    'world': ('plot_angular_velocity', ['--frame', 'world'], ['ECI X rate', 'ECI Y rate', 'ECI Z rate', 'ECI rate magnitude']),
    'boom': ('plot_ggb', [], ['Boom extension']),
    'torquers': ('plot_magnetorquers', [], ['X dipole', 'Y dipole', 'Z dipole']),
    'battery': ('plot_battery', [], ['Battery energy / SoC', 'Interval solar / battery power', 'Smoothed charge / discharge']),
    'sampled': ('plot_battery', ['--power-mode', 'sampled'], ['Battery energy / SoC', 'Sampled solar / battery power', 'Smoothed charge / discharge']),
    'solar': ('plot_solar', [], ['Cumulative solar energy', 'Solar power / eclipse', 'Energy per orbit', 'Energy per sun cycle']),
    'estimator': ('plot_estimator', [], ['X estimate / truth', 'Y estimate / truth', 'Z estimate / truth', 'Rate estimation error', 'Attitude truth']),
    'pointing': ('plot_y_thomson', [], [f'{name} · {window}' for name in ['Nadir error', 'Other axis errors', 'Full attitude error', 'Orbit-relative rate', 'Boom extension'] for window in ['overview', 'final orbit']]),
    'orbit': ('animate_orbit', [], ['Orbit in ECI', 'Spacecraft attitude', 'Recorded power', 'Panel illumination']),
}
CATALOG = [{'id': f'{key}:{i}', 'group': key, 'title': title}
           for key, (_, _, titles) in FAMILIES.items() for i, title in enumerate(titles)]
VALID_IDS = {entry['id'] for entry in CATALOG}


class TelemetryTail:
    """Read only appended bytes, retaining complete numeric rows and bounded history."""
    def __init__(self, path, max_rows):
        self.path = Path(path)
        self.rows = deque(maxlen=max_rows)
        self.header = None
        self.offset = 0
        self.pending = b''
        self.identity = None
        self.version = 0
        self.total = 0
        self.prefix = b''
        self.last_time = None

    def poll(self):
        try:
            source = self.path.open('rb')
        except FileNotFoundError:
            return 'Waiting for telemetry file'
        with source:
            import os
            stat = os.fstat(source.fileno())
            identity = (stat.st_dev, stat.st_ino)
            prefix = source.read(min(self.offset, 512))
            if (identity != self.identity or stat.st_size < self.offset
                    or (self.prefix and not prefix.startswith(self.prefix))):
                self.rows.clear()
                self.header = None
                self.offset = 0
                self.pending = b''
                self.total = 0
                self.last_time = None
                self.version += 1
                self.identity = identity
            source.seek(0)
            self.prefix = source.read(min(stat.st_size, 512))
            source.seek(self.offset)
            block = source.read(8 * 1024 * 1024)
            self.offset += len(block)
        lines = (self.pending + block).split(b'\n')
        self.pending = lines.pop()
        for raw in lines:
            line = raw.decode('utf-8-sig').strip()
            if not line:
                continue
            fields = next(csv.reader([line]))
            if self.header is None or fields[0] == 'time_s':
                if self.header is not None:
                    self.rows.clear()
                    self.total = 0
                    self.last_time = None
                self.header = fields
                self.version += 1
                continue
            if len(fields) != len(self.header):
                raise ValueError('Incomplete or malformed telemetry row')
            timestamp = float(fields[0])
            # A writer may truncate and grow past our offset between polls.
            if self.last_time is not None and timestamp <= self.last_time:
                self.rows.clear()
                self.total = 0
            self.rows.append(fields)
            self.last_time = timestamp
            self.total += 1
            self.version += 1
        if not self.rows:
            return 'Waiting for complete telemetry samples'
        suffix = ' · catching up' if self.offset < stat.st_size else ''
        return f't = {self.last_time:g} s · {len(self.rows):,} retained / {self.total:,} samples{suffix}'

    def snapshot(self, path):
        if not self.header or not self.rows:
            raise ValueError('Waiting for complete telemetry samples')
        with path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(self.header)
            writer.writerows(self.rows)


class Dashboard:
    def __init__(self, path, config=None, max_rows=20000, estimator_enabled=True):
        self.tail = TelemetryTail(path, max_rows)
        self.estimator = TelemetryTail(Path(path).with_name(Path(path).stem + '_estimator.csv'), max_rows)
        self.config = Path(config) if config else None
        self.estimator_enabled = estimator_enabled
        self.lock = threading.Lock()
        self.cache = {}
        self.builders = {}

    def snapshot(self, ids, known=None):
        """Return changed numeric panels; retained full-resolution data feeds all metrics."""
        known = known or {}
        with self.lock:
            status = self.tail.poll()
            estimator_status = (self.estimator.poll() if self.estimator_enabled
                                else 'Unavailable in orbit_solar mode')
            plots = {}
            for item in ids:
                family, index = item.split(':')
                tail = self.estimator if family == 'estimator' else self.tail
                if known.get(item) == tail.version:
                    continue
                if self.cache.get(item, (None,))[0] != tail.version:
                    try:
                        if family == 'estimator' and not self.estimator_enabled:
                            raise ValueError(estimator_status)
                        if not tail.rows:
                            raise ValueError('Waiting for telemetry samples')
                        key = 'estimator' if family == 'estimator' else 'main'
                        if self.builders.get(key, (None,))[0] != tail.version:
                            data = np.asarray(tail.rows, dtype=float)
                            columns = {name: data[:, i] for i, name in enumerate(tail.header)}
                            self.builders[key] = (tail.version, Traces(columns, self.config))
                        value = self.builders[key][1].panel(family, int(index))
                    except (OSError, ValueError, KeyError, TypeError) as exc:
                        value = {'error': str(exc)}
                    self.cache[item] = (tail.version, value)
                plots[item] = dict(self.cache[item][1], version=tail.version)
            return {'status': status, 'estimator_status': estimator_status, 'plots': plots}


class SimulationSession:
    """Single-use start gate shared by the HTTP handler and simulator main thread."""
    def __init__(self):
        self.start_requested = threading.Event()
        self.output_ready = threading.Event()
        self.lock = threading.Lock()
        self.state = 'waiting'
        self.error = None
        self.token = secrets.token_urlsafe(32)

    def snapshot(self):
        with self.lock:
            return {'state': self.state, 'error': self.error, 'token': self.token}

    def request_start(self):
        with self.lock:
            if self.state != 'waiting':
                return False
            self.state = 'starting'
            self.start_requested.set()
            return True

    def ready(self):
        with self.lock:
            self.state = 'running'
            self.output_ready.set()

    def finish(self, error=None):
        with self.lock:
            self.state = 'failed' if error else 'completed'
            self.error = error


def run_simulator_viewer(config, config_path, simulate, *, port=8765,
                         max_rows=20000, open_browser=True):
    """Serve the UI in a thread; wait for Start before running physics on the main thread.

    Keeping physics on the main thread preserves normal Ctrl+C handling and the
    simulator's existing telemetry/controller cleanup on all supported platforms.
    """
    session = SimulationSession()
    dashboard = Dashboard(config['simulation']['output_csv'], config_path, max_rows,
                          estimator_enabled=config['simulation']['mode'] != 'orbit_solar')
    server = make_server(dashboard, port, session=session)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    exit_code = 0
    try:
        url = f'http://127.0.0.1:{server.server_port}'
        print(f'Live viewer: {url}\nArrange your graphs, then press Start simulation. '
              'Ctrl+C closes the viewer.', flush=True)
        if open_browser:
            webbrowser.open(url)
        # Timed waits keep Ctrl+C responsive on Windows as well as Unix.
        while not session.start_requested.wait(0.2):
            pass
        try:
            simulate(config, on_output_ready=session.ready)
        except Exception as exc:
            session.finish(str(exc) or type(exc).__name__)
            traceback.print_exc()
            exit_code = 1
        else:
            session.finish()
        print('Simulation finished. Viewer remains open for inspection; Ctrl+C to exit.', flush=True)
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        if session.snapshot()['state'] in ('waiting', 'starting', 'running'):
            exit_code = 130
    finally:
        server.shutdown()
        worker.join()
        server.server_close()
    return exit_code


class LiveServer:
    """aiohttp server adapter retaining the simulator's main-thread lifecycle."""
    def __init__(self, dashboard, port, session):
        self.dashboard, self.session = dashboard, session
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.socket.bind(('127.0.0.1', port))
            self.socket.listen(128)
            self.socket.setblocking(False)
        except BaseException:
            self.socket.close()
            raise
        self.server_port = self.socket.getsockname()[1]
        self.loop = None
        self.stopping = threading.Event()
        self.connections = set()

    def state(self):
        return self.session.snapshot() if self.session else {'state': 'viewer-only'}

    def application(self):
        from aiohttp import web, WSMsgType
        app = web.Application(client_max_size=16384)
        async def index(request):
            return web.FileResponse(Path(__file__).with_name('live_dashboard.html'))
        async def javascript(request):
            return web.FileResponse(Path(__file__).with_name('live_dashboard.js'))
        async def plotly(request):
            import plotly
            return web.FileResponse(Path(plotly.__file__).parent / 'package_data' / 'plotly.min.js')
        async def catalog(request):
            return web.json_response(CATALOG)
        async def state(request):
            return web.json_response(self.state(), headers={'Cache-Control': 'no-store'})
        async def websocket(request):
            # Reject browser requests initiated by another origin, including DNS rebinding.
            expected = f'http://127.0.0.1:{self.server_port}'
            if request.host != f'127.0.0.1:{self.server_port}' or request.headers.get('Origin', expected) != expected:
                raise web.HTTPForbidden()
            ws = web.WebSocketResponse(heartbeat=20, max_msg_size=16384)
            await ws.prepare(request)
            self.connections.add(ws)
            subscription = {'ids': [], 'paused': False, 'interval': .25, 'revision': 0}
            acknowledged = asyncio.Event()
            acknowledged.set()
            async def stream():
                known = {}
                revision = -1
                try:
                    while not ws.closed:
                        # One in-flight update per client. Slow tabs never accumulate a queue.
                        await acknowledged.wait()
                        acknowledged.clear()
                        if revision != subscription['revision']:
                            known = {}
                            revision = subscription['revision']
                        ids = [] if subscription['paused'] else list(subscription['ids'])
                        state_value = self.state()
                        if self.session and not self.session.output_ready.is_set():
                            message = ('Arrange your graphs, then press Start simulation' if state_value['state'] == 'waiting'
                                       else 'Waiting for simulation telemetry')
                            payload = {'status': message, 'estimator_status': 'Not started',
                                       'plots': {item: {'error': message} for item in ids}}
                        else:
                            try:
                                payload = await asyncio.to_thread(self.dashboard.snapshot, ids, known)
                            except Exception as exc:
                                payload = {'status': f'Telemetry error: {exc}', 'estimator_status': '', 'plots': {}}
                        for item, panel in payload['plots'].items():
                            known[item] = panel.get('version')
                        await ws.send_json(dict(payload, type='update', session=self.state()), dumps=lambda x: json.dumps(x, allow_nan=False))
                        await asyncio.sleep(subscription['interval'])
                except (ConnectionError, asyncio.CancelledError):
                    pass
                except Exception:
                    traceback.print_exc()
                    await ws.close(code=1011, message=b'Stream failed; reconnect')
            sender = asyncio.create_task(stream())
            try:
                async for message in ws:
                    if message.type != WSMsgType.TEXT:
                        continue
                    try:
                        data = json.loads(message.data)
                        if not isinstance(data, dict):
                            raise ValueError('Expected an object')
                        if data.get('type') == 'ack':
                            acknowledged.set()
                        elif data.get('type') == 'subscribe':
                            ids = data.get('ids', [])
                            if not isinstance(ids, list) or len(ids) > len(VALID_IDS) or any(not isinstance(i, str) or i not in VALID_IDS for i in ids):
                                raise ValueError('Unknown graph selection')
                            interval = float(data.get('interval', .25))
                            if not np.isfinite(interval):
                                raise ValueError('Invalid update interval')
                            subscription.update(ids=list(dict.fromkeys(ids)), paused=bool(data.get('paused', False)),
                                                interval=max(.1, min(10, interval)), revision=subscription['revision']+1)
                        elif data.get('type') == 'start':
                            if not self.session or not secrets.compare_digest(str(data.get('token', '')), self.session.token):
                                raise ValueError('Invalid simulation session')
                            self.session.request_start()
                        else:
                            raise ValueError('Unknown message type')
                    except (ValueError, TypeError) as exc:
                        await ws.send_json({'type': 'error', 'error': str(exc)})
            finally:
                sender.cancel()
                await sender
                self.connections.discard(ws)
            return ws
        app.add_routes([web.get('/', index), web.get('/live_dashboard.js', javascript),
                        web.get('/plotly.js', plotly), web.get('/api/catalog', catalog),
                        web.get('/api/session', state), web.get('/ws', websocket)])
        return app

    async def _serve(self):
        from aiohttp import web
        self.loop = asyncio.get_running_loop()
        runner = web.AppRunner(self.application(), shutdown_timeout=2)
        await runner.setup()
        try:
            await web.SockSite(runner, self.socket).start()
            while not self.stopping.is_set():
                await asyncio.sleep(.1)
        finally:
            for connection in list(self.connections):
                await connection.close(code=1001, message=b'Viewer closing')
            await runner.cleanup()

    def serve_forever(self):
        asyncio.run(self._serve())

    def shutdown(self):
        self.stopping.set()

    def server_close(self):
        self.shutdown()
        self.socket.close()


def make_server(dashboard, port=0, session=None):
    # Fail before waiting for Start if live dependencies are unavailable.
    import aiohttp  # noqa: F401
    import plotly  # noqa: F401
    return LiveServer(dashboard, port, session)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('csv', nargs='?', type=Path, default=Path('telemetry.csv'))
    cli.add_argument('--config', type=Path, help='Configuration that produced the telemetry (for orbit views)')
    cli.add_argument('--port', type=int, default=8765)
    cli.add_argument('--max-rows', type=int, default=20000, help='Retained rows per CSV; calculations use this history')
    cli.add_argument('--no-browser', action='store_true')
    args = cli.parse_args()
    if args.max_rows < 3 or not 0 <= args.port <= 65535:
        cli.error('--max-rows must be >= 3; --port must be between 0 and 65535')
    server = make_server(Dashboard(args.csv, args.config, args.max_rows), args.port)
    url = f'http://127.0.0.1:{server.server_port}'
    print(f'Live dashboard: {url}\nWatching {args.csv.resolve()} · Ctrl+C to close', flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
