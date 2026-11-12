"""BinPlan: which bin each resistor value goes in, for one sorting run, and what each bin holds."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Tray:
    """What one bin holds so far, as the run's window shows it."""
    bin_id: str
    value_ohm: float | None         # the value this bin was given, None while it has none
    count: int                      # resistors sorted into it (each confirmed by the look after its drop)
    label: str = ""                 # what the bin is for when it isn't a value ("unclassified"), else ""


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

    With an unclassified bin, resistors whose value couldn't be read (ohms None) or is outside the sorter's
    range go there instead of staying on the plate, and that bin is never given a value.
    """

    def __init__(self, bin_ids, sorter_ohm, unclassified=None):
        """bin_ids: the bins, in the order they are handed out. sorter_ohm: the range of values sorted,
        (low, high) ohms. unclassified: a bin id from bin_ids for resistors with no value read or one outside
        sorter_ohm, or None to leave them on the plate."""
        if unclassified is not None and unclassified not in bin_ids:
            raise ValueError(f"no bin {unclassified} for the unclassified: the bins are {', '.join(bin_ids)}")
        self.bin_ids = list(bin_ids)
        self.unclassified = unclassified
        self.free = [b for b in bin_ids if b != unclassified]
        self.sorter_ohm = tuple(sorter_ohm)
        self.assigned = {}          # ohms: bin id, in the order values arrived

    @classmethod
    def from_slots(cls, slots, sorter_ohm, unclassified=None):
        """From the trays' slots ({id: entry}, as resistor_nav.load_bins gives), handed out in the file's order."""
        return cls(list(slots), sorter_ohm, unclassified)

    def in_range(self, ohms):
        return self.sorter_ohm[0] <= ohms <= self.sorter_ohm[1]

    def bin_for(self, ohms):
        """The bin a resistor of this value would go in, or None if it has none (outside the sorter's
        range with no unclassified bin, or every bin taken). ohms None (a value not read) or outside the
        sorter's range: the unclassified bin, if there is one. Decides nothing."""
        if ohms is None or not self.in_range(ohms):
            return self.unclassified
        if ohms in self.assigned:
            return self.assigned[ohms]
        return self.free[0] if self.free else None

    def assign(self, ohms):
        """The bin this value goes in from now on: bin_for's answer, made final. Raises ValueError if it
        has none."""
        b = self.bin_for(ohms)
        if b is None:
            raise ValueError("no bin for a resistor with no value read" if ohms is None else f"no bin for {ohms:g} ohm")
        if ohms is not None and self.in_range(ohms) and ohms not in self.assigned:
            self.assigned[ohms] = self.free.pop(0)
        return b

    def trays(self, sorted_attempts):
        """A Tray per bin, in order: the value each was given and how many resistors have gone in.
        sorted_attempts: PickMemory.sorted, the picks confirmed by a later look."""
        by_bin = {b: v for v, b in self.assigned.items()}
        return [Tray(b, by_bin.get(b), sum(a.bin_id == b for a in sorted_attempts),
                     "unclassified" if b == self.unclassified else "") for b in self.bin_ids]
