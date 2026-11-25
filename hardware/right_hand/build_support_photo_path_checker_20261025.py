"""Generate a scoped checker without modifying the original loaded checker."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
s=(ROOT/'screen_loaded_quarter_trial_20261025.py').read_text(encoding='utf-8')
def change(a,b,count=1):
    global s
    if s.count(a)!=count:raise ValueError('Ambiguous checker replacement: '+a[:100])
    s=s.replace(a,b)
change('from loaded_quarter_model_policy_20261025 import policy,correct_model',
       'from loaded_quarter_model_policy_20261025 import policy,correct_model\nfrom support_photo_path_plan_20261025 import SupportPhotoPathPlan as BookGraspPlan\nfrom support_photo_contact_resolution_20261025 import classify as classify_photo_initial,metadata as photo_resolution_metadata')
change('raw_ring_observations=[]','raw_ring_observations=[];photo_resolution_observations=[]')
change('        raw_ring_max=[0.]','        raw_ring_max=[0.];photo_discarded={}\n        photo_resolution_observations.append(photo_discarded)')
change('        def inspect(d):','        def inspect(d,initial_release=False):')
change('            return support_pairs,classify_self_contacts(self_pairs)',
       "            classified=classify_self_contacts(self_pairs)\n            resolved=classify_photo_initial(classified,initial_release=initial_release)\n            for pair,depth in classified.items():\n                if pair not in resolved:photo_discarded[pair]=max(photo_discarded.get(pair,0.),depth)\n            return support_pairs,resolved")
change('            pairs,self_pairs=inspect(d)\n            if self_pairs.get(CASING_PAIR',
       '            pairs,self_pairs=inspect(d,initial_release=True)\n            if self_pairs.get(CASING_PAIR',2)
change('pose(d,goal);pairs,self_pairs=inspect(d);escape.observe(pairs,index)',
       'pose(d,goal);pairs,self_pairs=inspect(d,initial_release=phase==RING_RELEASE);escape.observe(pairs,index)')
change('pairs,self_pairs=inspect(d);escape.observe(pairs,elapsed)',
       "pairs,self_pairs=inspect(d,initial_release=phase in ('initial_hold',RING_RELEASE));escape.observe(pairs,elapsed)")
change('native_ranges_unchanged=not policy(raw)',
       'photo_initial_thumb_contact_resolution=photo_resolution_metadata(),initial_resolution_discarded_maximums_by_front=photo_resolution_observations,initial_resolution_forces_preserved=True,native_ranges_unchanged=not policy(raw)')
(ROOT/'screen_support_photo_path_20261025.py').write_text(s,encoding='utf-8')
print('Generated photo-only contact-resolution checker; original checker unchanged')
