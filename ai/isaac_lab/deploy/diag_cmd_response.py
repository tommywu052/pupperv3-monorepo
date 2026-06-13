"""Offline diagnostic: does the exported policy respond to the velocity command?

Builds a nominal "standing" observation (zeros => robot at default pose, level,
zero joint vel, zero last action) and varies ONLY the velocity_commands slice,
then prints how much the action changes. If the action barely moves between
cmd=0 and cmd=[0.5,0,0] / [0,0,1.5], the actor learned to ignore the command.
"""
import numpy as np
import onnxruntime as ort

M = "/home/pi/pupper_policy/policy.onnx"
s = ort.InferenceSession(M, providers=["CPUExecutionProvider"])
inn = s.get_inputs()[0].name
out = s.get_outputs()[0].name


def act(cmd):
    obs = np.zeros((1, 45), dtype=np.float32)
    obs[0, 6:9] = cmd  # velocity_commands slice
    return s.run([out], {inn: obs})[0].reshape(-1)


base = act([0, 0, 0])
print("action @cmd=0      :", np.round(base, 3))
for c in ([0.2, 0, 0], [0.5, 0, 0], [0.75, 0, 0], [0, 0, 1.5], [0, 0, -1.5]):
    a = act(c)
    print(f"action @cmd={str(c):14s}: dmax={np.abs(a-base).max():.3f} "
          f"dmean={np.abs(a-base).mean():.3f}")
