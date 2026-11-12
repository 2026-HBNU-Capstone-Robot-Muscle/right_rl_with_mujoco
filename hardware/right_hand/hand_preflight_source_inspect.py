"""Print the imported preflight implementation; does not construct a bus."""
import inspect
from sequence_flip_checked import Bus
print(inspect.getsource(Bus.preflight))
