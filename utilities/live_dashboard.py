"""Local, draggable live dashboard backed by the existing plotting utilities."""
from __future__ import annotations


# Allow direct execution from any working directory as well as package execution.
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import base64
from collections import deque
import csv
import importlib
import io
import json
import secrets
import traceback
from pathlib import Path
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
import webbrowser

# Axis indices refer to the existing figures; secondary axes stay with their panel.
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


def panel_png(fig, family, index):
    axes = list(fig.axes)
    if index >= len(FAMILIES[family][2]) or index >= len(axes):
        raise ValueError('This panel requires full attitude and power telemetry')
    chosen = [axes[index]]
    if family in ('battery', 'sampled') and index == 0:
        chosen.append(axes[3])
    if family == 'solar' and index == 1:
        chosen.append(axes[4])
    fig.set_layout_engine(None)
    fig.set_size_inches(8, 4.8)
    if fig._suptitle:
        fig._suptitle.set_visible(False)
    for ax in axes:
        ax.set_visible(ax in chosen)
    for ax in chosen:
        ax.set_position([0.16, 0.19, 0.67, 0.64])
        ax.tick_params(labelbottom=True)
    # Shared-axis panels inherit the source figure's time unit.
    if not chosen[0].get_xlabel() and family != 'orbit':
        chosen[0].set_xlabel('Simulation time (hours)' if family == 'pointing' else 'Simulation time (s)')
    buffer = io.BytesIO()
    fig.savefig(buffer, format='png', dpi=110)
    return base64.b64encode(buffer.getvalue()).decode('ascii')


class Dashboard:
    def __init__(self, path, config=None, max_rows=20000, estimator_enabled=True):
        self.tail = TelemetryTail(path, max_rows)
        self.estimator = TelemetryTail(Path(path).with_name(Path(path).stem + '_estimator.csv'), max_rows)
        self.config = config
        self.estimator_enabled = estimator_enabled
        self.lock = threading.Lock()
        self.cache = {}

    def render(self, ids):
        with self.lock:
            status = self.tail.poll()
            estimator_status = (self.estimator.poll() if self.estimator_enabled
                                else 'Unavailable in orbit_solar mode')
            result = {}
            with tempfile.TemporaryDirectory(prefix='cubesat-live-') as directory:
                for family in dict.fromkeys(item.split(':')[0] for item in ids):
                    selected = [item for item in ids if item.startswith(family + ':')]
                    tail = self.estimator if family == 'estimator' else self.tail
                    missing = [item for item in selected if self.cache.get(item, (None,))[0] != tail.version]
                    if missing:
                        try:
                            if family == 'estimator' and not self.estimator_enabled:
                                raise ValueError(estimator_status)
                            snapshot = Path(directory) / 'telemetry.csv'
                            tail.snapshot(snapshot)
                            module, options, _ = FAMILIES[family]
                            argv = [str(snapshot), *options]
                            if family == 'orbit':
                                if not self.config:
                                    raise ValueError('Restart dashboard with --config for orbit and attitude views')
                                argv += ['--config', str(self.config), '--frames', '2']
                            def capture(fig):
                                for item in missing:
                                    try:
                                        value = {'image': panel_png(fig, family, int(item.split(':')[1]))}
                                    except ValueError as exc:
                                        value = {'error': str(exc)}
                                    self.cache[item] = (tail.version, value)
                            importlib.import_module('utilities.' + module).main(argv, render=capture)
                        except (OSError, ValueError, KeyError, TypeError) as exc:
                            for item in missing:
                                self.cache[item] = (tail.version, {'error': str(exc)})
                        finally:
                            from matplotlib import pyplot as plt
                            plt.close('all')
                    result.update({item: self.cache[item][1] for item in selected})
            return {'status': status, 'estimator_status': estimator_status, 'plots': result}


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
    import matplotlib
    matplotlib.use('Agg')
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


def make_server(dashboard, port=0, session=None):
    class Handler(BaseHTTPRequestHandler):
        def send_json(self, value, status=200):
            body = json.dumps(value).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path != '/api/start' or session is None:
                self.send_error(404)
                return
            # A different website must not be able to start hardware via localhost.
            if not secrets.compare_digest(self.headers.get('X-Session-Token', ''), session.token):
                self.send_error(403)
                return
            started = session.request_start()
            self.send_json(session.snapshot(), 202 if started else 409)

        def do_GET(self):
            url = urlparse(self.path)
            try:
                if url.path == '/':
                    body = Path(__file__).with_name('live_dashboard.html').read_bytes()
                    content_type = 'text/html; charset=utf-8'
                elif url.path == '/api/session':
                    self.send_json(session.snapshot() if session else {'state': 'viewer-only'})
                    return
                elif url.path == '/api/catalog':
                    body = json.dumps(CATALOG).encode()
                    content_type = 'application/json'
                elif url.path == '/api/plots':
                    ids = list(dict.fromkeys(parse_qs(url.query).get('id', [])))
                    if any(item not in VALID_IDS for item in ids):
                        self.send_error(400, 'Unknown graph')
                        return
                    if session is not None and not session.output_ready.is_set():
                        message = ('Arrange your graphs, then press Start simulation'
                                   if session.snapshot()['state'] == 'waiting' else 'Waiting for simulation telemetry')
                        self.send_json({'status': message, 'estimator_status': 'Not started',
                                        'plots': {item: {'error': message} for item in ids}})
                        return
                    body = json.dumps(dashboard.render(ids)).encode()
                    content_type = 'application/json'
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:
                self.send_error(500, str(exc))

        def log_message(self, *args):
            pass
    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


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
    import matplotlib
    matplotlib.use('Agg')
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
