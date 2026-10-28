"""Numeric Plotly traces for live telemetry; no rasterization or plotting backend."""
from datetime import datetime

import numpy as np
import yaml

from utilities.plot_angular_velocity import body_to_world
from utilities.plot_battery import interval_power, smooth_interval_power
from utilities.plot_solar import analyze
from utilities.plot_y_thomson import pointing_metrics
from utilities.animate_orbit import frame_geometry, panel_parameters
from cubesat_sim.sun import sun_position_eci

COLORS = ['#65b8ff', '#69dfb2', '#ff8f86', '#d5a1ff']


def reduced(x, y, limit=2000):
    """Keep extrema in each time bucket, after calculations on full retained data."""
    x, y = np.asarray(x), np.asarray(y)
    if len(x) <= limit:
        return x.tolist(), y.tolist()
    indices = [0, len(x) - 1]
    for bucket in np.array_split(np.arange(1, len(x) - 1), (limit - 2) // 2):
        indices.extend((bucket[np.argmin(y[bucket])], bucket[np.argmax(y[bucket])]))
    indices = np.unique(indices)
    return x[indices].tolist(), y[indices].tolist()


def line(name, x, y, color=None, step=False, **kwargs):
    x, y = reduced(x, y)
    return dict(type='scatter', mode='lines', name=name, x=x, y=y,
                line=dict(color=color or COLORS[0], width=1.5, shape='hv' if step else 'linear'), **kwargs)


def chart(traces, ylabel, xlabel='Simulation time (s)', **layout):
    return {'data': traces, 'layout': dict(xaxis={'title': {'text': xlabel}},
                                         yaxis={'title': {'text': ylabel}}, **layout)}


class Traces:
    def __init__(self, columns, config=None):
        self.columns = columns
        self.config = config
        self.cache = {}
        self.time = self.get('time_s')[:, 0]

    def get(self, *names):
        missing = set(names) - self.columns.keys()
        if missing:
            raise ValueError('Missing telemetry columns: ' + ', '.join(sorted(missing)))
        data = np.column_stack([self.columns[name] for name in names])
        if not np.isfinite(data).all():
            raise ValueError('Telemetry contains nonfinite values')
        return data

    def cached(self, name, calculate):
        if name not in self.cache:
            self.cache[name] = calculate()
        return self.cache[name]

    def panel(self, family, index):
        t = self.time
        if not len(t):
            raise ValueError('Waiting for telemetry samples')
        if len(t) > 1 and np.any(np.diff(t) <= 0):
            raise ValueError('Telemetry times must increase')
        if family in ('body', 'world'):
            rates = self.get('wx_rad_s', 'wy_rad_s', 'wz_rad_s')
            if family == 'world':
                rates = body_to_world(rates, self.get('sigma_BN_x', 'sigma_BN_y', 'sigma_BN_z'))
            values = rates[:, index] if index < 3 else np.linalg.norm(rates, axis=1)
            return chart([line('XYZ'[index] if index < 3 else 'Magnitude', t, values)], 'Angular rate (rad/s)')
        if family == 'boom':
            return chart([line('Boom extension', t, self.get('boom_length_m')[:, 0])], 'Extension (m)')
        if family == 'torquers':
            return chart([line('XYZ'[index], t, self.get(('mx_Am2', 'my_Am2', 'mz_Am2')[index])[:, 0], step=True)], 'Dipole (A·m²)')
        if family in ('battery', 'sampled'):
            energy = self.get('battery_energy_Wh')[:, 0]
            if index == 0:
                soc = self.get('battery_soc')[:, 0]
                return chart([line('Stored energy', t, energy), line('State of charge', t, 100*soc, COLORS[1], yaxis='y2')],
                             'Energy (Wh)', yaxis2={'title': {'text': 'SoC (%)'}, 'overlaying': 'y', 'side': 'right', 'range': [0, 100]})
            if index == 1 and family == 'sampled':
                return chart([line('Solar bus', t, self.get('solar_power_W')[:, 0], '#f3c76b'),
                              line('Net battery', t, self.get('battery_power_W')[:, 0], COLORS[1])], 'Power (W)')
            if len(t) < 2:
                raise ValueError('At least two energy samples are needed')
            net = interval_power(t, energy)
            if index == 1:
                solar = interval_power(t, self.get('solar_energy_Wh')[:, 0])
                return chart([line('Solar bus interval mean', t, np.r_[solar, solar[-1]], '#f3c76b', step=True),
                              line('Net battery interval mean', t, np.r_[net, net[-1]], COLORS[1], step=True)], 'Power (W)')
            centers, charge = smooth_interval_power(t, np.maximum(net, 0), 300)
            _, discharge = smooth_interval_power(t, np.minimum(net, 0), 300)
            return chart([line('Battery charging', centers, charge, '#f3c76b'),
                          line('Battery discharging', centers, discharge, COLORS[2]),
                          line('Net battery', centers, charge+discharge, COLORS[1])], 'Power (W) · 300 s centered mean')
        if family == 'solar':
            data = self.get('solar_power_W', 'sunlight_fraction', 'solar_energy_Wh', 'orbit_period_s')
            orbits, cycles = self.cached('solar', lambda: analyze(t, data))
            if index == 0:
                return chart([line('Produced energy', t, data[:, 2]-data[0, 2])], 'Energy (Wh)')
            if index == 1:
                power = interval_power(t, data[:, 2])
                return chart([line('Solar bus interval mean', t, np.r_[power, power[-1]], '#f3c76b', step=True),
                              line('Visible Sun fraction', t, data[:, 1], '#9bacbf', yaxis='y2')], 'Power (W)',
                             yaxis2={'title': {'text': 'Visible Sun'}, 'overlaying': 'y', 'side': 'right', 'range': [0, 1.05]})
            edges, values, complete = (orbits, cycles)[index-2]
            return chart([dict(type='bar', name=name, x=(np.flatnonzero(mask)+1).tolist(), y=values[mask].tolist(),
                               marker={'color': color}) for mask, name, color in
                          [(complete, 'Complete', COLORS[0]), (~complete, 'Partial / unbounded', '#f3c76b')]],
                         'Produced energy (Wh)', 'Interval number')
        if family == 'estimator':
            state = self.get('wx_rad_s', 'wy_rad_s', 'wz_rad_s', 'sigma_BN_x', 'sigma_BN_y', 'sigma_BN_z')
            def comparison():
                d = self.get('estimate_present', 'estimate_valid', 'estimate_time_s', 'estimate_wx_rad_s', 'estimate_wy_rad_s', 'estimate_wz_rad_s')
                d = d[(d[:, 0] == 1) & (d[:, 1] == 1) & (d[:, 2] >= t[0]) & (d[:, 2] <= t[-1])]
                epochs, first = np.unique(d[:, 2], return_index=True)
                estimate = d[first, 3:6]
                truth = np.column_stack([np.interp(epochs, t, state[:, a]) for a in range(3)])
                return epochs, estimate, estimate-truth
            epochs, estimate, error = self.cached('estimator', comparison)
            if index < 3:
                marker = line('Firmware (sensor epoch)', epochs, estimate[:, index], COLORS[1]); marker['mode'] = 'markers'
                return chart([line('Truth', t, state[:, index]), marker], 'Angular rate (rad/s)')
            if index == 3:
                if not len(epochs):
                    raise ValueError('No valid rate estimates in retained truth interval')
                rmse = np.sqrt(np.mean(error**2, axis=0))
                return chart([line(f'{axis} · RMSE {rmse[i]:.4g}', epochs, error[:, i], COLORS[i]) for i, axis in enumerate('XYZ')], 'Estimate − truth (rad/s)')
            return chart([line(axis, t, state[:, i+3], COLORS[i]) for i, axis in enumerate('XYZ')], 'Truth MRP σ_BN')
        if family == 'pointing':
            angles, total, relative, orbital = self.cached('pointing', lambda: pointing_metrics(t,
                self.get('x_m', 'y_m', 'z_m'), self.get('sigma_BN_x', 'sigma_BN_y', 'sigma_BN_z'),
                self.get('wx_rad_s', 'wy_rad_s', 'wz_rad_s')))
            mask = t >= t[-1]-2*np.pi/np.median(orbital) if index % 2 else np.ones(len(t), dtype=bool)
            x = t[mask]
            kind = index // 2
            if kind == 0:
                return chart([line('+Z nadir error', x, angles[mask, 2])], 'Error (deg)')
            if kind == 1:
                return chart([line(name, x, angles[mask, i], COLORS[i]) for i, name in enumerate(('+X to along-track', '+Y to negative orbit normal'))], 'Error (deg)')
            if kind == 2:
                return chart([line('Full attitude error', x, total[mask])], 'Error (deg)')
            if kind == 3:
                rates = np.degrees(relative[mask])
                return chart([line(a, x, rates[:, i], COLORS[i]) for i, a in enumerate('XYZ')]+
                             [line('Magnitude', x, np.linalg.norm(rates, axis=1), COLORS[3])], 'Orbit-relative rate (deg/s)')
            return chart([line('Boom extension', x, self.get('boom_length_m')[mask, 0])], 'Extension (m)')
        if family == 'orbit':
            return self.orbit(index)
        raise ValueError('Unknown graph family')

    def orbit(self, index):
        if not self.config:
            raise ValueError('Orbit views require the simulation configuration (--config for standalone viewer)')
        config = self.cached('config', lambda: yaml.safe_load(self.config.read_text()))
        t = self.time
        position = self.get('x_m', 'y_m', 'z_m') / 1000
        radius = float(config['environment']['earth_radius_m']) / 1000
        start = config['simulation']['start_utc']
        start = start.isoformat() if isinstance(start, datetime) else start
        p = position[-1]
        direction = sun_position_eci(start, t[-1])-p*1000
        direction /= np.linalg.norm(direction)
        def vector(name, origin, direction, length, color):
            xyz = np.array([origin, origin+direction*length])
            return dict(type='scatter3d', mode='lines', name=name, x=xyz[:, 0].tolist(), y=xyz[:, 1].tolist(),
                        z=xyz[:, 2].tolist(), line={'color': color, 'width': 5})
        def scene(data, units):
            return {'data': data, 'layout': {'scene': {'aspectmode': 'data',
                **{a+'axis': {'title': {'text': f'ECI {a.upper()} {units}'}} for a in 'xyz'}}}}
        if index == 0:
            selected = np.unique(np.linspace(0, len(t)-1, min(len(t), 2000)).astype(int))
            lon, lat = np.meshgrid(np.linspace(0, 2*np.pi, 40), np.linspace(0, np.pi, 20))
            earth = dict(type='surface', x=(radius*np.cos(lon)*np.sin(lat)).tolist(),
                         y=(radius*np.sin(lon)*np.sin(lat)).tolist(), z=(radius*np.cos(lat)).tolist(),
                         colorscale=[[0, '#244a82'], [1, '#479cc7']], showscale=False, opacity=.5, hoverinfo='skip')
            path = dict(type='scatter3d', mode='lines', name='Recorded orbit',
                        **{axis: position[selected, i].tolist() for i, axis in enumerate('xyz')}, line={'color': COLORS[0]})
            satellite = dict(type='scatter3d', mode='markers', name=f'Spacecraft · {t[-1]:g} s',
                             **{axis: [float(p[i])] for i, axis in enumerate('xyz')}, marker={'size': 4, 'color': COLORS[2]})
            traces = [earth, path, satellite, vector('Sun direction (length reduced)', p, direction, radius*.5, '#f3c76b')]
            if all('sigma_BN_'+a in self.columns for a in 'xyz'):
                axes = body_to_world(np.eye(3), self.get('sigma_BN_x', 'sigma_BN_y', 'sigma_BN_z')[-1])
                traces += [vector('Body '+axis, p, axes[i], radius*.12, COLORS[i]) for i, axis in enumerate('XYZ')]
            return scene(traces, '(km)')
        if index == 2:
            return chart([line('Solar bus', t, self.get('solar_power_W')[:, 0], '#f3c76b'),
                          line('Net battery', t, self.get('battery_power_W')[:, 0], COLORS[1])], 'Recorded power (W)')
        data = self.get('x_m', 'y_m', 'z_m', 'sigma_BN_x', 'sigma_BN_y', 'sigma_BN_z', 'solar_power_W', 'battery_power_W', 'sunlight_fraction')
        solar = config['spacecraft']['solar_charging']
        normals, area = panel_parameters(config)
        sun, axes, panel_normals, power = frame_geometry(start, t[-1], data[-1], normals, area, solar)
        if index == 3:
            return chart([dict(type='bar', name='Estimated panel bus power', x=[f'P{i+1}' for i in range(len(power))],
                               y=power.tolist(), marker={'color': '#f3c76b'})], 'Estimated power (W)', 'Panel')
        traces = [vector('Body '+a, np.zeros(3), axes[i], 1, COLORS[i]) for i, a in enumerate('XYZ')]
        traces += [vector('Sun direction', np.zeros(3), sun, 1.6, '#f3c76b')]
        traces += [vector(f'P{i+1} · {power[i]:.3g} W', normal*.55, normal, .75,
                          '#f3c76b' if power[i] > 0 else '#777') for i, normal in enumerate(panel_normals)]
        vertices = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)])
        rotated = body_to_world(vertices, data[-1, 3:6])
        for i in range(8):
            for j in range(i+1, 8):
                if np.count_nonzero(vertices[i] != vertices[j]) == 1:
                    traces.append(vector('Body', rotated[i], rotated[j]-rotated[i], 1, '#9bacbf'))
                    traces[-1]['showlegend'] = False
        result = scene(traces, '(schematic)')
        for a in 'xyz':
            result['layout']['scene'][a+'axis']['range'] = [-1.8, 1.8]
        return result
