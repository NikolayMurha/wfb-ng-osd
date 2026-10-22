"""Loopback DVR integration test; requires PyGObject, GStreamer GL and FFmpeg.

Send black video, disconnect for five seconds, then reconnect with white video
and a reset sender clock. Verify frozen video, changing OSD and resumed video.
Use --srt to exercise separate SRT connections instead of RTP/UDP.
"""
import argparse
import json
import os
import pathlib
import signal
import socket
import struct
import subprocess
import tempfile
import threading
import time

import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)
parser = argparse.ArgumentParser(description='Record through a sender outage and timestamp reset, with and without OSD.')
parser.add_argument('--player', default=str(pathlib.Path(__file__).resolve().parents[1] / 'osd.gst'))
parser.add_argument('--srt', action='store_true', help='Use SRT connections instead of RTP/UDP')
args = parser.parse_args()
work = tempfile.TemporaryDirectory(prefix='wfb-continuous-')
root = pathlib.Path(work.name)

def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]

def heartbeat(port, stop):
    seq = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        while not stop.is_set():
            payload = struct.pack('<IBBBBB', 0, 1, 3, 128, 4, 3)
            header = bytes([9, seq % 256, 1, 1, 0])
            crc = 65535
            for byte in header + payload + bytes([50]):
                tmp = byte ^ crc & 255
                tmp ^= tmp << 4 & 255
                crc = (crc >> 8 ^ tmp << 8 ^ tmp << 3 ^ tmp >> 4) & 65535
            sock.sendto(b'\xfe' + header + payload + struct.pack('<H', crc), ('127.0.0.1', port))
            seq += 1
            stop.wait(0.2)

def send_video(port, pattern):
    p = Gst.parse_launch(f'videotestsrc is-live=true num-buffers=60 pattern={pattern} ! video/x-raw,format=I420,width=320,height=180,framerate=30/1 ! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 ! h264parse ! ' + (f'mpegtsmux ! srtsink uri="srt://127.0.0.1:{port}?mode=caller" sync=false' if args.srt else f'rtph264pay config-interval=1 pt=96 ! udpsink host=127.0.0.1 port={port} sync=false'))
    p.set_state(Gst.State.PLAYING)
    m = p.get_bus().timed_pop_filtered(10 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
    assert m is not None and m.type == Gst.MessageType.EOS
    p.set_state(Gst.State.NULL)

def frame(path, seconds):
    return subprocess.check_output(['ffmpeg', '-v', 'error', '-ss', str(seconds), '-i', str(path), '-frames:v', '1', '-f', 'rawvideo', '-pix_fmt', 'rgb24', 'pipe:1'])
for osd in (False, True):
    name = 'osd' if osd else 'plain'
    port, telemetry = (free_port(), free_port())
    logfile = (root / f'{name}.log').open('w')
    command = [args.player, '-4', '-j', '0', '-s', 'fakesink', '-w', '320', '-P', str(port), '-p', str(telemetry), '-O', str(root / f'{name}_%05d.ts')]
    if args.srt:
        index = command.index('-P')
        command[index:index + 2] = ['-R', f'srt://127.0.0.1:{port}?mode=listener']
    if not osd:
        command.append('-n')
    player = subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT, env=os.environ)
    stop = threading.Event()
    thread = threading.Thread(target=heartbeat, args=(telemetry, stop))
    thread.start()
    try:
        time.sleep(2)
        assert player.poll() is None, f'{name}: player exited early'
        send_video(port, 'black')
        time.sleep(5)
        assert player.poll() is None, f'{name}: player exited during outage'
        send_video(port, 'white')
        time.sleep(1)
        player.send_signal(signal.SIGTERM)
        assert player.wait(timeout=6) == 0
    finally:
        stop.set()
        thread.join()
        logfile.close()
        if player.poll() is None:
            player.kill()
            player.wait()
    path = root / f'{name}_00000.ts'
    info = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0', '-show_entries', 'stream=nb_read_frames,r_frame_rate', '-of', 'json', str(path)]))['streams'][0]
    frames = int(info['nb_read_frames'])
    print(name, info, flush=True)
    assert frames >= 240, f'{name}: recorded only {frames} frames, outage was lost'
    frozen_a, frozen_b, resumed = (frame(path, 3), frame(path, 5), frame(path, frames / 30 - 0.2))
    assert len(frozen_a) == len(frozen_b) > 0
    assert sum(frozen_a) / len(frozen_a) < 50
    assert sum(resumed) / len(resumed) > 180, f'{name}: video did not resume after sender restarted'
    if osd:
        changed = sum((a != b for a, b in zip(frozen_a, frozen_b)))
        print('OSD pixel changes during frozen video:', changed, flush=True)
        assert changed > 20, 'OSD did not update during outage'
