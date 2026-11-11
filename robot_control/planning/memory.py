"""PickMemory: what a sorting run remembers between looks at the plate."""

from dataclasses import dataclass, field

MISS_MM = 4.0           # a body this close to where the last pick aimed is the one it was meant to take
MAX_TRIES = 2           # misses at one spot before the run leaves that resistor on the plate


@dataclass
class Attempt:
    """One pick: where the jaws aimed, the value that was there, and the bin it was taken to."""
    spot_mm: tuple[float, float]
    value_ohm: float
    bin_id: str


@dataclass
class GivenUp:
    spot_mm: tuple[float, float]
    why: str


def _near(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 <= MISS_MM ** 2


@dataclass
class PickMemory:
    """The gripper can't feel whether it holds anything, so a pick is judged by the next look: if a body
    is still where the jaws aimed, the pick missed. After MAX_TRIES misses at one spot, or a pick the arm
    refused, that resistor is given up and left on the plate for the rest of the run.

        memory.attempted(attempt)       # just before the pick
        memory.judge(view)              # on the next look: "sorted", "missed", "gave up" or None
        memory.is_given_up(resistor)    # the reason to leave it alone, or None
    """

    last: Attempt | None = None
    sorted: list[Attempt] = field(default_factory=list)         # picks whose resistor left its spot
    misses: dict = field(default_factory=dict)                  # spot mm: misses there
    given_up: list[GivenUp] = field(default_factory=list)

    def attempted(self, attempt):
        self.last = attempt

    def judge(self, view):
        """How the last pick went, from a look taken after it: "sorted" if its spot is now empty,
        "missed" if a body is still there, "gave up" if that was its last try; None if there was no pick
        to judge."""
        attempt, self.last = self.last, None
        if attempt is None:
            return None
        if not any(_near(r.centre_mm, attempt.spot_mm) for r in view.resistors):
            self.sorted.append(attempt)
            return "sorted"
        spot = next((s for s in self.misses if _near(s, attempt.spot_mm)), tuple(attempt.spot_mm))
        self.misses[spot] = self.misses.get(spot, 0) + 1
        if self.misses[spot] >= MAX_TRIES:
            self.give_up(spot, f"missed {self.misses[spot]} times")
            return "gave up"
        return "missed"

    def give_up(self, spot_mm, why):
        """Leave whatever lies at spot_mm on the plate for the rest of the run."""
        self.given_up.append(GivenUp(tuple(spot_mm), why))

    def is_given_up(self, resistor):
        """The reason a resistor is being left on the plate, or None."""
        return next((g.why for g in self.given_up if _near(resistor.centre_mm, g.spot_mm)), None)
