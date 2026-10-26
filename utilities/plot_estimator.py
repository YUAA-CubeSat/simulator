"""Compare firmware body angular-rate estimates with truth at the sensor epoch."""

# Allow direct execution from any working directory as well as package execution.
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pathlib import Path

import numpy as np
from utilities.common import load_telemetry, parser, pyplot, run, save_plot


def load_comparison(path):
    columns = ('wx_rad_s', 'wy_rad_s', 'wz_rad_s', 'sigma_BN_x', 'sigma_BN_y',
               'sigma_BN_z', 'estimate_present', 'estimate_time_s', 'estimate_valid',
               'estimate_wx_rad_s', 'estimate_wy_rad_s', 'estimate_wz_rad_s')
    time, data = load_telemetry(path, columns)
    mask = (data[:, 6] == 1) & (data[:, 8] == 1)
    epoch = data[mask, 7]
    estimates = data[mask, 9:12]
    # Exclude epochs outside recorded truth; never silently extrapolate.
    inside = (epoch >= time[0]) & (epoch <= time[-1])
    epoch, estimates = epoch[inside], estimates[inside]
    # A held estimate can be transmitted on many control ticks. Score it once.
    epoch, indices = np.unique(epoch, return_index=True)
    estimates = estimates[indices]
    truth = np.column_stack([np.interp(epoch, time, data[:, a]) for a in range(3)])
    return time, data[:, :6], epoch, estimates, truth


def main(argv=None, render=None):
    cli = parser(__doc__, 'plots/estimator.png')
    cli.set_defaults(csv=Path('telemetry_estimator.csv'))
    args = cli.parse_args(argv)
    time, state, epoch, estimates, truth = load_comparison(args.csv)
    plt = pyplot(args.show)
    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(11, 12), layout='constrained')
    for a, label in enumerate('xyz'):
        axes[a].plot(time, state[:, a], label='Truth')
        axes[a].plot(epoch, estimates[:, a], '.', ms=2, label='Firmware (sensor epoch)')
        axes[a].set_ylabel(f'ω{label} (rad/s)')
        axes[a].legend()
    if len(epoch):
        error = estimates - truth
        for a, label in enumerate('xyz'):
            axes[3].plot(epoch, error[:, a], '.', ms=2, label=label)
        rmse = np.sqrt(np.mean(error**2, axis=0))
        print(f'{len(epoch)} unique valid samples; RMSE x/y/z (rad/s): {rmse}')
        axes[3].legend()
    else:
        print('No valid rate estimates in recorded truth interval. Rates are available only in pointing mode.')
        axes[3].text(0.5, 0.5, 'No valid rate estimates', transform=axes[3].transAxes, ha='center')
    axes[3].set_ylabel('Estimate − truth\n(rad/s)')
    for a, label in enumerate('xyz'):
        axes[4].plot(time, state[:, a + 3], label=label)
    axes[4].set_ylabel('Truth MRP σ_BN')
    axes[4].set_title('Attitude truth only — firmware has no attitude estimator')
    axes[4].legend()
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel('Simulation time (s)')
    fig.suptitle('Firmware estimator verification (body-frame rates)')
    if render is None:
        save_plot(fig, plt, args)
    else:
        save_plot(fig, plt, args, render=render)


if __name__ == '__main__':
    run(main)
