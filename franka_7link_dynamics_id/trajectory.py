from __future__ import annotations

import json
from pathlib import Path

import numpy as np


class FFSTrajectory:
    def __init__(self, source: Path | dict):
        if isinstance(source, dict):
            values = source
        else:
            with Path(source).open("r", encoding="utf-8") as stream:
                values = json.load(stream)
        self.initial_q = np.asarray(values["initial_q"], dtype=float).reshape(7)
        self.ak = np.asarray(values["ak"], dtype=float).reshape(7, -1)
        self.bk = np.asarray(values["bk"], dtype=float).reshape(7, -1)
        if self.ak.shape != self.bk.shape:
            raise ValueError("FFS ak and bk arrays must have matching shapes.")
        self.omega_f = float(values["omega_f"])
        self.duration = float(values["duration"])
        self.num_harmonics = int(values["num_harmonics"])
        if self.ak.shape[1] != self.num_harmonics or self.omega_f <= 0.0:
            raise ValueError("FFS coefficients or base frequency are inconsistent.")
        self.k_omega = np.arange(1, self.num_harmonics + 1) * self.omega_f

    def evaluate(self, time_s):
        time_s = np.atleast_1d(np.asarray(time_s, dtype=float))
        phase = time_s[:, None] * self.k_omega[None, :]
        sine = np.sin(phase)
        cosine = np.cos(phase)
        q = self.initial_q + (
            sine[:, :, None] * (self.ak.T / self.k_omega[:, None])[None, :, :]
        ).sum(axis=1) - (
            (cosine - 1.0)[:, :, None]
            * (self.bk.T / self.k_omega[:, None])[None, :, :]
        ).sum(axis=1)
        qd = (
            cosine[:, :, None] * self.ak.T[None, :, :]
            + sine[:, :, None] * self.bk.T[None, :, :]
        ).sum(axis=1)
        qdd = (
            -sine[:, :, None] * (self.ak.T * self.k_omega[:, None])[None, :, :]
            + cosine[:, :, None] * (self.bk.T * self.k_omega[:, None])[None, :, :]
        ).sum(axis=1)
        return q, qd, qdd
