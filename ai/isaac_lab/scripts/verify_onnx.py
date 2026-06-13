"""Verify the exported ONNX policy matches the Isaac-side (TorchScript) policy.

``play.py`` exports two artifacts from the *same* running policy module
(actor + observation normalizer):

  * ``policy.pt``  — TorchScript, i.e. exactly what runs inside Isaac Lab,
  * ``policy.onnx`` — what we will build into a TensorRT engine for Jetson NX.

This script feeds identical observation batches through both and reports the
numerical difference, so we know the ONNX export is faithful before touching
the robot. It needs only numpy / torch / onnxruntime (no Isaac Sim app).

Run (Windows)::

    cd C:\\Nvidia\\IsaacLab\\IsaacLab
    .\\isaaclab.bat -p C:\\Nvidia\\pupperv3\\pupperv3-monorepo\\ai\\isaac_lab\\scripts\\verify_onnx.py
"""

from __future__ import annotations

import argparse
import glob
import os

import numpy as np

DEFAULT_LOG_GLOB = r"C:\Nvidia\IsaacLab\IsaacLab\logs\rsl_rl\pupper_flat\*\exported"


def _latest_exported_dir() -> str:
    candidates = sorted(glob.glob(DEFAULT_LOG_GLOB), key=os.path.getmtime)
    if not candidates:
        raise FileNotFoundError(f"No exported/ dir found under {DEFAULT_LOG_GLOB}")
    return candidates[-1]


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=None, help="exported/ dir (defaults to latest pupper_flat run)")
    ap.add_argument("--onnx", default=None)
    ap.add_argument("--jit", default=None)
    ap.add_argument("--n", type=int, default=512, help="number of random observation samples")
    ap.add_argument("--seed", type=int, default=0)
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    exp_dir = args.dir or _latest_exported_dir()
    onnx_path = args.onnx or os.path.join(exp_dir, "policy.onnx")
    jit_path = args.jit or os.path.join(exp_dir, "policy.pt")
    print(f"[verify] exported dir : {exp_dir}")
    print(f"[verify] onnx         : {onnx_path}")
    print(f"[verify] jit (torch)  : {jit_path}")

    import onnxruntime as ort  # noqa: E402
    import torch  # noqa: E402

    # --- ONNX session + IO spec ---
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    in_info = sess.get_inputs()[0]
    out_info = sess.get_outputs()[0]
    in_name, out_name = in_info.name, out_info.name
    # resolve observation width (handle symbolic batch dim)
    in_shape = in_info.shape
    obs_dim = int([d for d in in_shape if isinstance(d, int) and d > 1][-1]) if any(
        isinstance(d, int) and d > 1 for d in in_shape
    ) else int(in_shape[-1])
    print(f"[verify] onnx input  '{in_name}' shape={in_shape} -> obs_dim={obs_dim}")
    print(f"[verify] onnx output '{out_name}' shape={out_info.shape}")

    # --- TorchScript reference (the module Isaac runs) ---
    jit_model = torch.jit.load(jit_path, map_location="cpu").eval()

    rng = np.random.default_rng(args.seed)

    # Isaac Lab exports the ONNX with a fixed batch dim of 1, so feed row-by-row.
    fixed_batch = isinstance(in_shape[0], int) and in_shape[0] == 1

    def run_case(name: str, obs: np.ndarray):
        obs = obs.astype(np.float32)
        if fixed_batch:
            onnx_out = np.concatenate(
                [sess.run([out_name], {in_name: obs[i : i + 1]})[0] for i in range(obs.shape[0])], axis=0
            )
        else:
            onnx_out = sess.run([out_name], {in_name: obs})[0]
        with torch.inference_mode():
            jit_out = jit_model(torch.from_numpy(obs)).cpu().numpy()
        onnx_out = np.asarray(onnx_out).reshape(obs.shape[0], -1)
        jit_out = np.asarray(jit_out).reshape(obs.shape[0], -1)
        abs_diff = np.abs(onnx_out - jit_out)
        denom = np.maximum(np.abs(jit_out), 1e-6)
        rel = abs_diff / denom
        print(
            f"  {name:<22} max|Δ|={abs_diff.max():.3e}  mean|Δ|={abs_diff.mean():.3e}  "
            f"max rel={rel.max():.3e}  allclose(1e-4)={np.allclose(onnx_out, jit_out, atol=1e-4, rtol=1e-3)}"
        )
        return abs_diff.max()

    print("[verify] comparing onnxruntime vs TorchScript on identical observations:")
    worst = 0.0
    # standing (zeros), small, normal, large-magnitude, and a big random batch
    worst = max(worst, run_case("zeros (standing)", np.zeros((1, obs_dim))))
    worst = max(worst, run_case("small uniform", rng.uniform(-0.1, 0.1, (16, obs_dim))))
    worst = max(worst, run_case("normal(0,1)", rng.standard_normal((args.n, obs_dim))))
    worst = max(worst, run_case("large normal(0,5)", 5.0 * rng.standard_normal((64, obs_dim))))

    print("-" * 70)
    ok = worst < 1e-3
    print(f"[verify] worst max|Δ| across all cases = {worst:.3e}  ->  {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
