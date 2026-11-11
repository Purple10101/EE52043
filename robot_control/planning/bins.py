"""BinPlan: which bin each resistor value goes in, for one sorting run, and what each bin holds."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Tray:
    """What one bin holds so far, as the run's window shows it."""
    bin_id: str
    value_ohm: float | None         # the value this bin was given, None while it has none
    count: int                      # resistors sorted into it (each confirmed by the look after its drop)


class BinPlan:
    """Which bin each resistor value goes in, for one run.

    The plan starts empty. The first time a value is sorted it takes the next free bin, in the order the
    bins are listed, and keeps it until the run ends, so every resistor of that value lands together.
    A value outside the sorter's range isn't sorted: it stays on the plate. Once every bin is taken, a
    new value has nowhere to go and stays on the plate too.

        plan = BinPlan(["A1", "A2", "A3"], sorter_ohm=(10, 10_000))
        plan.bin_for(1500)      # "A1": where it would go (nothing is decided yet)
        plan.assign(1500)       # "A1": decided, for the rest of the run
        plan.bin_for(1500)      # "A1" again; plan.bin_for(2200) is now "A2"
    """

    def __init__(self, bin_ids, sorter_ohm):
        """bin_ids: the bins, in the order they are handed out. sorter_ohm: the range of values sorted,
        (low, high) ohms."""
        self.bin_ids = list(bin_ids)
        self.free = list(bin_ids)
        self.sorter_ohm = tuple(sorter_ohm)
        self.assigned = {}          # ohms: bin id, in the order values arrived

    @classmethod
    def from_slots(cls, slots, sorter_ohm):
        """From the trays' slots ({id: entry}, as resistor_nav.load_bins gives), handed out in the file's order."""
        return cls(list(slots), sorter_ohm)

    def in_range(self, ohms):
        return self.sorter_ohm[0] <= ohms <= self.sorter_ohm[1]

    def bin_for(self, ohms):
        """The bin a resistor of this value would go in, or None if it has none (outside the sorter's
        range, or every bin taken). Decides nothing."""
        if not self.in_range(ohms):
            return None
        if ohms in self.assigned:
            return self.assigned[ohms]
        return self.free[0] if self.free else None

    def assign(self, ohms):
        """The bin this value goes in from now on: bin_for's answer, made final. Raises ValueError if it
        has none."""
        b = self.bin_for(ohms)
        if b is None:
            raise ValueError(f"no bin for {ohms:g} ohm")
        if ohms not in self.assigned:
            self.assigned[ohms] = self.free.pop(0)
        return b

    def trays(self, sorted_attempts):
        """A Tray per bin, in order: the value each was given and how many resistors have gone in.
        sorted_attempts: PickMemory.sorted, the picks confirmed by a later look."""
        by_bin = {b: v for v, b in self.assigned.items()}
        return [Tray(b, by_bin.get(b), sum(a.bin_id == b for a in sorted_attempts)) for b in self.bin_ids]
