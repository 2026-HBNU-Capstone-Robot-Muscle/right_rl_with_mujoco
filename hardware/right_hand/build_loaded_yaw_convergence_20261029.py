"""Numerical refinement only; preserve every physics acceptance criterion."""
from pathlib import Path
root=Path(__file__).resolve().parent
source=(root/'screen_loaded_yaw_path_20261027.py').read_text(encoding='utf-8')
def once(a,b):
    global source
    if source.count(a)!=1:raise RuntimeError('Unexpected source shape: '+a)
    source=source.replace(a,b)
once("parser.add_argument('--front',type=float);args=parser.parse_args()", "parser.add_argument('--front',type=float);parser.add_argument('--dt',type=float,default=.001,choices=(.002,.001,.0005));parser.add_argument('--gain',type=float,choices=(.5,1.,1.5));parser.add_argument('--friction',type=float,choices=(.2,.3,.5));args=parser.parse_args()")
once('m.opt.timestep=.002','m.opt.timestep=args.dt')
once('for _ in range(1000):','for _ in range(round(2./args.dt)):')
once('for scale in ((1.,) if args.nominal_only else (1.5,1.,.5)):', 'for scale in ((args.gain,) if args.gain is not None else ((1.,) if args.nominal_only else (1.5,1.,.5))):')
once('for friction in ((.3,) if args.nominal_only else (.2,.3,.5)):', 'for friction in ((args.friction,) if args.friction is not None else ((.3,) if args.nominal_only else (.2,.3,.5))):')
once('target=round(sample*.075/.002)','target=round(sample*.075/args.dt)')
once('nohand+.002','nohand+args.dt')
once('row=dict(book_front_mm=front,','row=dict(simulation_timestep_s=args.dt,book_front_mm=front,')
once("result=dict(status='full_loaded_yaw_release_model_diagnostic',", "result=dict(simulation_timestep_s=args.dt,settling_seconds=2.,physics_acceptance_bounds_unchanged=True,status='full_loaded_yaw_release_model_diagnostic',")
destination=root/'screen_loaded_yaw_convergence_20261029.py'
if destination.exists():raise RuntimeError('Refuse to overwrite model')
destination.write_text(source,encoding='utf-8')
print('Numerical refinement model created; original model unchanged.')
