"""Grey Wolf Optimizer, in an ask/tell form so hardware can do the evaluating.

The textbook GWO calls a fitness function directly. Here every evaluation means
pushing gains to the MCU, letting the loop settle and measuring the live stream
for a few hundred milliseconds -- none of which can happen inside a synchronous
callback without freezing the GUI. So the optimizer is turned inside out:

    pos = opt.ask()      # next candidate, or None when the run is over
    ...                  # apply it, wait, measure (across many Qt timer ticks)
    opt.tell(cost)       # hand back what it scored

The algorithm itself is unchanged from Mirjalili, Mirjalili & Lewis (2014):
each iteration scores the whole pack, keeps the best three (alpha, beta, delta)
seen so far, and moves every wolf toward the average of the three positions
those leaders imply. The exploration coefficient `a` falls linearly 2 -> 0
across the run, which is what turns a wide search into a local one.
"""

from __future__ import annotations

import numpy as np


class GreyWolfOptimizer:
    """Ask/tell GWO over a box-bounded real search space.

    Args:
        lower, upper: per-dimension bounds, same length.
        n_wolves: pack size. Each iteration costs this many evaluations.
        n_iterations: how many times the pack moves.
        seed_position: optional starting point placed as wolf 0, so a run
            always has at least one known-good candidate in it -- on real
            hardware that matters, it means the first iteration cannot be
            uniformly worse than what was already running.
        rng_seed: fixes the run for reproducibility.
    """

    def __init__(self, lower, upper, n_wolves=10, n_iterations=25,
                 seed_position=None, rng_seed=None):
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        if self.lower.shape != self.upper.shape:
            raise ValueError("lower and upper must have the same length")
        if np.any(self.upper < self.lower):
            raise ValueError("upper bound below lower bound")

        self.dim = self.lower.size
        self.n_wolves = int(n_wolves)
        self.n_iterations = int(n_iterations)
        self._rng = np.random.default_rng(rng_seed)

        self.positions = self._rng.uniform(
            self.lower, self.upper, size=(self.n_wolves, self.dim)
        )
        if seed_position is not None:
            self.positions[0] = np.clip(
                np.asarray(seed_position, dtype=float), self.lower, self.upper
            )

        # Leaders, best-first. inf until the first tell().
        self.alpha_position = self.positions[0].copy()
        self.beta_position = self.positions[0].copy()
        self.delta_position = self.positions[0].copy()
        self.alpha_score = np.inf
        self.beta_score = np.inf
        self.delta_score = np.inf

        self.iteration = 0          # iterations fully scored so far
        self._wolf = 0              # next wolf to hand out this iteration
        self._asked = None          # index the caller is currently evaluating
        self.history = []           # alpha score after each iteration
        self.finished = False

    # ------------------------------------------------------------------ API
    @property
    def total_evaluations(self):
        return self.n_wolves * self.n_iterations

    @property
    def evaluations_done(self):
        return self.iteration * self.n_wolves + self._wolf

    def ask(self):
        """The next candidate position, or None once the run is complete."""
        if self.finished:
            return None
        self._asked = self._wolf
        return self.positions[self._wolf].copy()

    def tell(self, cost: float):
        """Score for the position handed out by the last ask()."""
        if self._asked is None:
            raise RuntimeError("tell() without a matching ask()")

        position = self.positions[self._asked]
        self._asked = None

        # A cost that is NaN (a diverged run, a measurement that never
        # arrived) must never win; +inf sorts it out of the leadership.
        if not np.isfinite(cost):
            cost = np.inf
        self._rank(position, float(cost))

        self._wolf += 1
        if self._wolf < self.n_wolves:
            return

        # Whole pack scored: record, move, and start the next iteration.
        self._wolf = 0
        self.iteration += 1
        self.history.append(self.alpha_score)
        if self.iteration >= self.n_iterations:
            self.finished = True
            return
        self._move_pack()

    # ------------------------------------------------------------- internals
    def _rank(self, position, cost):
        """Slot a scored position into the alpha/beta/delta hierarchy."""
        if cost < self.alpha_score:
            self.delta_score, self.delta_position = self.beta_score, self.beta_position.copy()
            self.beta_score, self.beta_position = self.alpha_score, self.alpha_position.copy()
            self.alpha_score, self.alpha_position = cost, position.copy()
        elif cost < self.beta_score:
            self.delta_score, self.delta_position = self.beta_score, self.beta_position.copy()
            self.beta_score, self.beta_position = cost, position.copy()
        elif cost < self.delta_score:
            self.delta_score, self.delta_position = cost, position.copy()

    def _move_pack(self):
        """One GWO position update for every wolf.

        `a` shrinks linearly with the iteration count, which shrinks |A| with
        it: |A| > 1 pushes a wolf away from the leaders (explore), |A| < 1
        pulls it in (exploit). That schedule is the whole of GWO's
        exploration/exploitation balance.
        """
        a = 2.0 - 2.0 * (self.iteration / float(self.n_iterations))

        shape = (self.n_wolves, self.dim)
        new = np.empty(shape)
        for leader in (self.alpha_position, self.beta_position, self.delta_position):
            A = 2.0 * a * self._rng.random(shape) - a
            C = 2.0 * self._rng.random(shape)
            D = np.abs(C * leader - self.positions)
            candidate = leader - A * D
            new = candidate if leader is self.alpha_position else new + candidate

        self.positions = np.clip(new / 3.0, self.lower, self.upper)
