from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'propose_checkpoint_counterturn_20261030.py').read_text(encoding='utf-8')
replacements={"O=BASE/'opposed_contact_ik'":"O=BASE/'opposed_contact_free_center_ik'","basis=np.column_stack([e0,e1])":"basis=np.column_stack([e0,e1,up])", "[0.,0.]]":"[0.,0.,0.]]", "[-.02,-.02]]":"[-.02,-.02,-.003]]", "[.02,.02]]":"[.02,.02,.003]]"}
for a,b in replacements.items():assert a in s,a;s=s.replace(a,b)
(R/'propose_checkpoint_counterturn_free_center_20261030.py').write_text(s,encoding='utf-8');print('Generated free-center geometry variant; preceding rejected geometry preserved.')
