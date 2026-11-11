"""Planner: given a look at the plate, which resistor to pick next, and what becomes of every other one."""

from dataclasses import dataclass

from robot_offline.post.bands.learned import ohms_text


@dataclass(frozen=True)
class Pick:
    """The next pick: this resistor, into this bin."""
    resistor: object        # world.Resistor
    bin_id: str


@dataclass
class Plan:
    """What one look leads to: the next pick (None when nothing is left that the run can sort), and a
    line per resistor saying what becomes of it, for the window and the log."""
    next: Pick | None
    fates: dict             # resistor number: text
    waiting: int            # resistors that will be picked, the next one included


class Planner:
    """Decides; never moves anything. A resistor is picked if all of these hold:
    - its value was read with confidence (an unsure one stays on the plate),
    - the run hasn't given up on it (PickMemory),
    - its value is in the sorter's range, and has a bin or a bin is still free for it (BinPlan).
    Of those, the most isolated goes first: its neighbours are the least likely to be knocked, and taking
    it clears space around the rest.
    """

    def __init__(self, bins, memory):
        self.bins = bins            # planning.BinPlan
        self.memory = memory        # planning.PickMemory

    def plan(self, view):
        fates, candidates = {}, []
        for r in view.resistors:
            stays = self.why_it_stays(r)
            if stays:
                fates[r.number] = f"stays: {stays}"
                continue
            pick = Pick(r, self.bins.bin_for(r.value_ohm))
            candidates.append(pick)
            fates[r.number] = f"{ohms_text(r.value_ohm)} -> {pick.bin_id}"
        candidates.sort(key=lambda p: -gap_mm(p.resistor, view.resistors))
        if candidates:
            first = candidates[0].resistor.number
            fates[first] = "NEXT " + fates[first]
        return Plan(candidates[0] if candidates else None, fates, len(candidates))

    def why_it_stays(self, r):
        """Why a resistor is left on the plate, or None if it can be picked."""
        if not r.is_read:
            return f"unsure (best {ohms_text(r.best_ohm)} at {r.confidence:.0%})"
        given_up = self.memory.is_given_up(r)
        if given_up:
            return given_up
        if not self.bins.in_range(r.value_ohm):
            low, high = self.bins.sorter_ohm
            return f"{ohms_text(r.value_ohm)} is outside {ohms_text(low)} to {ohms_text(high)}"
        if self.bins.bin_for(r.value_ohm) is None:
            return f"no bin free for {ohms_text(r.value_ohm)}"
        return None


def gap_mm(resistor, others):
    """How far a resistor is from its nearest neighbour, mm (infinite when it is alone)."""
    return min((resistor.distance_to(o.centre_mm) for o in others if o is not resistor), default=float("inf"))
