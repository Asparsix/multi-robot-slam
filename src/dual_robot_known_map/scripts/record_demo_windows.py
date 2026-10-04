#!/usr/bin/env python3
"""Record Gazebo + RViz (via X window IDs) into a side-by-side MP4."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys


def _window_ids() -> tuple[str, str]:
    out = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
    rviz = None
    gz = None
    for line in out.splitlines():
        m = re.match(r'\s*(0x[0-9a-fA-F]+)\s+"([^"]*)"', line)
        if not m:
            continue
        xid, title = m.group(1), m.group(2)
        if 'RViz' in title and 'mutter-x11-frames' in line:
            rviz = xid
        if title == 'Gazebo Sim' and 'mutter-x11-frames' in line:
            gz = xid
    if not rviz or not gz:
        raise RuntimeError(f'Could not find windows (rviz={rviz}, gz={gz})')
    return gz, rviz


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=10.0)
    args = ap.parse_args()

    gz, rviz = _window_ids()
    print(f'Recording gz={gz} rviz={rviz} for {args.seconds}s', flush=True)

    # Side-by-side compositor; timeout sends INT so -e finalizes the mp4.
    cmd = [
        'timeout', '-s', 'INT', str(args.seconds),
        'gst-launch-1.0', '-e',
        'ximagesrc', f'xid={gz}', 'use-damage=false', 'show-pointer=false',
        '!', 'videoconvert',
        '!', 'videoscale',
        '!', 'video/x-raw,width=960,height=540,framerate=10/1',
        '!', 'comp.sink_0',
        'ximagesrc', f'xid={rviz}', 'use-damage=false', 'show-pointer=false',
        '!', 'videoconvert',
        '!', 'videoscale',
        '!', 'video/x-raw,width=960,height=540,framerate=10/1',
        '!', 'comp.sink_1',
        'compositor', 'name=comp',
        'sink_0::xpos=0', 'sink_0::ypos=0',
        'sink_1::xpos=960', 'sink_1::ypos=0',
        '!', 'videoconvert',
        '!', 'x264enc', 'tune=zerolatency', 'speed-preset=ultrafast', 'bitrate=5000', 'key-int-max=20',
        '!', 'mp4mux',
        '!', 'filesink', f'location={args.out}',
    ]
    rc = subprocess.call(cmd)
    # timeout returns 124 on SIGINT completion; that is success for us
    print(f'gst exit={rc}', flush=True)
    return 0 if rc in (0, 124) else rc


if __name__ == '__main__':
    sys.exit(main())
