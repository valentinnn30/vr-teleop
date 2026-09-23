"""MANO-21 hand landmarks -> normalized BrainCo motor commands.

Extracted verbatim (FingerRetargeting class + helpers) from
gear_sonic/scripts/quest_manager_thread_server.py in the GR00T-WholeBodyControl
fork. Depends only on numpy + the brainco_retargeting package (third_party/
brainco-retargeting) -- no robot-body dependency, so a future non-BrainCo hand
is a new module here, not a rewrite of the teleop manager.
"""

import numpy as np

try:
    from brainco_retargeting.retargeter import BrainCoRetargeter
except ImportError:
    BrainCoRetargeter = None

try:
    from brainco_retargeting import np_retargeting
except ImportError:
    np_retargeting = None


_XR25_FROM_MANO = [
    (1, 1), (2, 2), (3, 3), (4, 4),  # thumb CMC/MCP/IP/TIP
    (6, 5), (7, 6), (8, 7), (9, 8),  # index MCP/PIP/DIP/TIP
    (11, 9), (12, 10), (13, 11), (14, 12),  # middle
    (16, 13), (17, 14), (18, 15), (19, 16),  # ring
    (21, 17), (22, 18), (23, 19), (24, 20),  # pinky
]
_XR25_METACARPALS = [(5, 5), (10, 9), (15, 13), (20, 17)]  # (xr_idx, mano_mcp_idx)


def mano21_to_xr25(landmarks21: np.ndarray) -> np.ndarray:
    lm = np.asarray(landmarks21, dtype=np.float64)
    xr = np.zeros((25, 3), dtype=np.float64)
    xr[0] = lm[0]
    for xr_i, mano_i in _XR25_FROM_MANO:
        xr[xr_i] = lm[mano_i]
    for xr_i, mano_mcp in _XR25_METACARPALS:
        xr[xr_i] = 0.5 * (lm[0] + lm[mano_mcp])
    return xr


class FingerRetargeting:
    """MANO-21 landmarks -> 7-element wire vector (6 normalized motors + pad).

    Uses the optimization-based BrainCoRetargeter when available; otherwise
    falls back to the pure-numpy angle-based retargeter.
    """

    # Wire order matches the BrainCo firmware / mock streamer convention.
    _NP_JOINT_KEYS = [
        "thumb_metacarpal",
        "thumb_proximal",
        "index_proximal",
        "middle_proximal",
        "ring_proximal",
        "pinky_proximal",
    ]

    def __init__(self, force_np: bool = False):
        self._opt = None
        if not force_np and BrainCoRetargeter is not None:
            try:
                self._opt = BrainCoRetargeter()
                print("[QuestManager] Finger retargeting: optimization-based BrainCoRetargeter")
            except Exception as e:
                print(f"[QuestManager] BrainCoRetargeter init failed ({e}), using numpy fallback")
        if self._opt is None:
            if np_retargeting is None:
                raise ImportError(
                    "Neither BrainCoRetargeter nor np_retargeting is available. "
                    "Install third_party/brainco-retargeting."
                )
            print("[QuestManager] Finger retargeting: pure-numpy fallback")

    def __call__(self, landmarks21: np.ndarray, side: str) -> list[float]:
        if self._opt is not None:
            xr = mano21_to_xr25(landmarks21)
            canon = self._opt.canonicalize(xr, side)
            if side == "left":
                motors = self._opt.retarget_left(canon)
            else:
                motors = self._opt.retarget_right(canon)
        else:
            angles = np_retargeting.retarget(np.asarray(landmarks21, dtype=np.float64), side)
            motors = [
                angles[f"{side}_{k}_joint"] / np_retargeting._JOINT_LIMITS[k][1]
                for k in self._NP_JOINT_KEYS
            ]
        return [float(np.clip(m, 0.0, 1.0)) for m in motors] + [0.0]
