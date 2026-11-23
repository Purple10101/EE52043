"""Planner: given a look at the plate, which resistor to pick next, and what becomes of every other one."""

from dataclasses import dataclass

from robot_offline.post.bands.learned import ohms_text


@dataclass(frozen=True)
class Pick:
    """The next pick: this resistor, into this bin. measure: the ohmmeter reads it once it is in the jaws,
    and its reading decides the bin; bin_id is then where its band value would go (None with no value)."""
    resistor: object        # world.Resistor
    bin_id: str | None
    measure: bool = False


@dataclass
class Plan:
    """What one look leads to: the next pick (None when nothing is left that the run can sort), and a
    line per resistor saying what becomes of it, for the window and the log."""
    next: Pick | None
    fates: dict             # resistor number: text
    waiting: int            # resistors that will be picked, the next one included


class Planner:
    """Decides; never moves anything. A resistor is picked if all of these hold:
    - it has a trusted value: read with confidence at the start of the run, and still where it was read.
      One without is to be measured: with the ohmmeter on (measure set) it is picked and measured in the
      jaws; without it, it stays on the plate unless the BinPlan has an unclassified bin, which then takes
      it, as it takes a value outside the range,
    - with the ohmmeter on, a value read outside the sorter's range is measured too, never binned on the bands
      alone: unclassified and out-of-range resistors always go to the meter (the user's rule, 2026-11-23), and
      its reading decides the bin (an out-of-range misread like 22k for a 2k2 goes where it belongs),
    - the run hasn't given up on it (PickMemory),
    - its value is in the sorter's range, and has a bin or a bin is still free for it (BinPlan).
    Of those, the most isolated goes first: its neighbours are the least likely to be knocked, and taking
    it clears space around the rest.
    """

    MEASURE = (None, "unsure", "all")

    def __init__(self, bins, memory, measure=None):
        """measure: which resistors the ohmmeter reads (config/ohmmeter.toml): None (no ohmmeter), "unsure"
        (those with no trusted value, and those read outside the sorter's range) or "all" (every pick). With the
        ohmmeter, the BinPlan needs an
        unclassified bin: it is where a resistor the meter can't place goes, as the arm is holding it."""
        if measure not in self.MEASURE:
            raise ValueError(f"measure must be one of {self.MEASURE}, not {measure!r}")
        if measure and bins.unclassified is None:
            raise ValueError("the ohmmeter needs a reject bin (the BinPlan's unclassified bin) for resistors it "
                             "can't place: set reject_bin in ohmmeter.toml or pass --unclassified-bin")
        self.bins = bins            # planning.BinPlan
        self.memory = memory        # planning.PickMemory
        self.measure = measure

    def plan(self, view):
        fates, candidates = {}, []
        for r in view.resistors:
            stays = self.why_it_stays(r)
            if stays:
                fates[r.number] = f"stays: {stays}"
                continue
            if self.measure and (not r.is_read or not self.bins.in_range(r.value_ohm)):
                pick = Pick(r, None, measure=True)
                candidates.append(pick)
                why = self.unsure(r) if not r.is_read else f"{ohms_text(r.value_ohm)}, outside the range"
                fates[r.number] = f"measure: {why} -> ohmmeter"
                continue
            pick = Pick(r, self.bins.bin_for(r.value_ohm), measure=self.measure == "all")
            candidates.append(pick)
            if not r.is_read:
                what = f"unclassified (best {ohms_text(r.best_ohm)} at {r.confidence:.0%})"
            elif not self.bins.in_range(r.value_ohm):
                what = f"unclassified ({ohms_text(r.value_ohm)}, outside the range)"
            else:
                what = ohms_text(r.value_ohm)
            fates[r.number] = f"{what} -> {pick.bin_id}" + (", measured first" if pick.measure else "")
        candidates.sort(key=lambda p: -gap_mm(p.resistor, view.resistors))
        if candidates:
            first = candidates[0].resistor.number
            fates[first] = "NEXT " + fates[first]
        return Plan(candidates[0] if candidates else None, fates, len(candidates))

    def why_it_stays(self, r):
        """Why a resistor is left on the plate, or None if it can be picked. One with no value read goes to
        the unclassified bin when the BinPlan has one."""
        given_up = self.memory.is_given_up(r)
        if not r.is_read:
            if (self.measure or self.bins.unclassified is not None) and not given_up:
                return None
            return given_up or f"measure: {self.unsure(r)}"
        if given_up:
            return given_up
        if not self.bins.in_range(r.value_ohm) and self.bins.unclassified is None:
            low, high = self.bins.sorter_ohm
            return f"{ohms_text(r.value_ohm)} is outside {ohms_text(low)} to {ohms_text(high)}"
        if self.bins.bin_for(r.value_ohm) is None:
            return f"no bin free for {ohms_text(r.value_ohm)}"
        return None

    @staticmethod
    def unsure(r):
        """Why a resistor has no trusted value, in a few words."""
        why = r.note or "unsure"
        return f"unsure (best {ohms_text(r.best_ohm)} at {r.confidence:.0%})" if why == "unsure" else why


def gap_mm(resistor, others):
    """How far a resistor is from its nearest neighbour, mm (infinite when it is alone)."""
    return min((resistor.distance_to(o.centre_mm) for o in others if o is not resistor), default=float("inf"))
