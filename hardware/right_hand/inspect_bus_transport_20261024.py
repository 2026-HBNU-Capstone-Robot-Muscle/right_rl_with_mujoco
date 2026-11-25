"""Inspect imported class definitions only; instantiate no hardware."""
import inspect
import sequence_flip_checked as checked
print('MRO',[(k.__name__,inspect.getfile(k)) for k in checked.Bus.__mro__ if k is not object])
for name in ('block','snapshot','read'):
    method=getattr(checked.Bus,name)
    print('METHOD',name,inspect.getfile(method),inspect.getsourcelines(method)[1])
    print(inspect.getsource(method))
