#!/usr/bin/env python

import yaml
import math

length = 10.0
start_time = 0
dt = 0.1

def get_frame(t):
    return {'x': t**2, 'y': t**3, 'z': math.exp(t), 
            'roll': math.sin(t), 'pitch': math.cos(t), 'yaw': math.erf(t)}

right_frames = ['right_wrist', 'right_elbow', 'right_shoulder']
left_frames = ['left_wrist', 'left_elbow', 'left_shoulder']
syncpoints = {0.0: 'start', 1.0: 'strokeStart', 8.0: 'strokeEnd'}
parent_frame = '/world'

def main():
    sequence = []
    for n in range(start_time, int(length/dt)):
        t = n*dt
        rframes = [{'parent_frame': parent_frame, 'child_frame': f, 'transform': get_frame(t)} for f in right_frames]
        lframes = [{'parent_frame': parent_frame, 'child_frame': f, 'transform': get_frame(t)} for f in left_frames]

        f = {'time': t, 'frames': {'right': rframes, 'left': lframes}}
        if t in syncpoints:
            f['syncpoint'] = syncpoints[t]
        sequence.append(f)
    obj = {'type': 'gesture', 'gesture': {'type': 'sequence', 'name': 'test_sequence_gesture', 'sequence': sequence } }
    stream = file('test_sequence_gesture.yaml', 'w')
    yaml.dump(obj, stream)
    

if __name__ == "__main__":
    main()
