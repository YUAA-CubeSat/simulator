"""Plot Y-Thomson nadir pointing errors and rotation relative to the orbit frame."""

import numpy as np

from utilities.common import load_telemetry, parser, pyplot, run, save_plot
from utilities.plot_angular_velocity import body_to_world


def pointing_metrics(time, position, attitude, rates):
    """Return directed axis errors [deg], full attitude error [deg], and
    orbit-relative body rates [rad/s]. Target: X=along-track, Y=-h, Z=nadir.

    Position derivatives estimate velocity on the original (possibly uneven)
    time grid. The orbit-rate formula assumes the simulator's fixed orbit plane.
    """
    time = np.asarray(time, dtype=float)
    position, attitude, rates = (np.asarray(x, dtype=float) for x in (position, attitude, rates))
    if time.ndim != 1 or len(time) < 3 or np.any(np.diff(time) <= 0):
        raise ValueError("At least three strictly increasing telemetry times are required")
    if any(x.shape != (len(time), 3) for x in (position, attitude, rates)):
        raise ValueError("Position, attitude and rates must each have shape (samples, 3)")
    if not all(np.isfinite(x).all() for x in (time, position, attitude, rates)):
        raise ValueError("Pointing telemetry must be finite")
    radius = np.linalg.norm(position, axis=1)
    if np.any(radius == 0):
        raise ValueError("Position must be nonzero to define nadir")
    velocity = np.gradient(position, time, axis=0, edge_order=2)
    momentum = np.cross(position, velocity)
    momentum_norm = np.linalg.norm(momentum, axis=1)
    if np.any(momentum_norm <= 1e-12 * radius * np.linalg.norm(velocity, axis=1)):
        raise ValueError("Orbit normal is undefined for stationary or radial motion")
    z = -position / radius[:, None]
    y = -momentum / momentum_norm[:, None]
    x = np.cross(y, z)
    # Columns are body axes expressed in inertial coordinates.
    body = np.stack([body_to_world(np.broadcast_to(axis, position.shape), attitude)
                     for axis in np.eye(3)], axis=-1)
    target = np.stack((x, y, z), axis=-1)
    cosines = np.sum(body * target, axis=1)
    axis_errors = np.degrees(np.arccos(np.clip(cosines, -1, 1)))
    attitude_error = np.degrees(np.arccos(np.clip((cosines.sum(axis=1) - 1) / 2, -1, 1)))
    orbit_rate_n = momentum / radius[:, None] ** 2
    orbit_rate_b = np.einsum('nji,nj->ni', body, orbit_rate_n)
    return axis_errors, attitude_error, rates - orbit_rate_b, np.linalg.norm(orbit_rate_n, axis=1)


def main():
    cli = parser(__doc__, "plots/y_thomson_pointing.png")
    cli.add_argument("--start-s", type=float, help="First simulation second to display")
    cli.add_argument("--end-s", type=float, help="Last simulation second to display")
    args = cli.parse_args()
    columns = ('x_m', 'y_m', 'z_m', 'sigma_BN_x', 'sigma_BN_y', 'sigma_BN_z',
               'wx_rad_s', 'wy_rad_s', 'wz_rad_s', 'boom_length_m')
    time, data = load_telemetry(args.csv, columns)
    angles, total, relative, orbital = pointing_metrics(time, data[:, :3], data[:, 3:6], data[:, 6:9])
    mask = np.ones(len(time), dtype=bool)
    for bound in (args.start_s, args.end_s):
        if bound is not None and not np.isfinite(bound):
            raise ValueError("Time bounds must be finite")
    if args.start_s is not None:
        mask &= time >= args.start_s
    if args.end_s is not None:
        mask &= time <= args.end_s
    if not mask.any():
        raise ValueError("No telemetry samples in the requested time window")
    # Keep the overview, but resolve the residual motion over one final orbit.
    period = 2 * np.pi / np.median(orbital[mask])
    detail = mask & (time >= time[mask][-1] - period)
    plt = pyplot(args.show)
    fig, axes = plt.subplots(5, 2, figsize=(15, 13), layout='constrained',
                             sharex='col')
    for col, window in enumerate((mask, detail)):
        hours = time[window] / 3600
        z_error = angles[window, 2]
        axes[0, col].plot(hours, z_error, color='C2', linewidth=0.8)
        if col == 0:
            axes[0, col].set_ylim(-3, 183)
            axes[0, col].set_yticks([0, 90, 180],
                                   ['0°: toward Earth', '90°: sideways', '180°: away'])
        else:
            # Tight limits expose small wobble without other axes obscuring Z.
            lo, hi = z_error.min(), z_error.max()
            pad = max(0.5, (hi - lo) * 0.15)
            axes[0, col].set_ylim(max(-1, lo - pad), min(181, hi + pad))
        axes[0, col].set_ylabel('+Z nadir error (deg)')
        for i, label in enumerate(('+X to along-track', '+Y to negative orbit normal')):
            axes[1, col].plot(hours, angles[window, i], label=label, linewidth=0.8)
        axes[1, col].set_ylabel('Other axis errors (deg)')
        axes[1, col].set_ylim(0, 180)
        axes[1, col].legend(loc='upper right', fontsize=8)
        axes[2, col].plot(hours, total[window], color='C3', linewidth=0.8)
        axes[2, col].set_ylabel('Full attitude error (deg)')
        axes[2, col].set_ylim(0, 180)
        for i, label in enumerate(('X', 'Y', 'Z')):
            axes[3, col].plot(hours, np.degrees(relative[window, i]), label=label, linewidth=0.8)
        axes[3, col].plot(hours, np.degrees(np.linalg.norm(relative[window], axis=1)),
                         color='black', label='Magnitude', linewidth=0.8)
        axes[3, col].axhline(0, color='gray', linestyle='--', linewidth=0.7)
        axes[3, col].set_ylabel('Orbit-relative rate (deg/s)')
        axes[3, col].legend(loc='upper right', ncol=4, fontsize=8)
        axes[4, col].plot(hours, data[window, 9], color='C4')
        axes[4, col].set_ylabel('Boom extension (m)')
        axes[4, col].set_xlabel('Simulation time (hours)')
        for ax in axes[:, col]:
            ax.grid(alpha=0.3)
    axes[0, 0].set_title('Selected time window')
    axes[0, 1].set_title('Final orbit within selected window — Z scale enlarged')
    z_detail = angles[detail, 2]
    rate_detail = np.degrees(np.linalg.norm(relative[detail], axis=1))
    fig.suptitle('Y-Thomson pointing: success = all errors and orbit-relative rate near zero\n'
                 f'Final-orbit +Z nadir error: {z_detail.min():.1f}–{z_detail.max():.1f}°; '
                 f'median residual rotation: {np.median(rate_detail):.3f}°/s')
    save_plot(fig, plt, args)
    print(f"Displayed window: {time[mask][0]:g}–{time[mask][-1]:g} s; "
          f"median orbital rate {np.degrees(np.median(orbital[mask])):.5f} deg/s")
    print(f"Final sample: nadir error {angles[mask][-1, 2]:.3f} deg; "
          f"full attitude error {total[mask][-1]:.3f} deg; "
          f"orbit-relative rate {np.degrees(np.linalg.norm(relative[mask][-1])):.6f} deg/s")


if __name__ == '__main__':
    run(main)
