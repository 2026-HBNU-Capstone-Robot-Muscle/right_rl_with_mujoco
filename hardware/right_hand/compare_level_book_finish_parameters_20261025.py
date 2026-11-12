"""OFFLINE numerical/contact sensitivity; no motor or camera APIs."""
import sys
import search_level_book_finish_20261025 as experiment
for args in (
    ['--extras','0','128','--seats','36','44','--timestep','0.001'],
    ['--extras','0','128','--seats','36','44','--timestep','0.001','--contact-time','0.004'],
    ['--extras','0','128','--seats','36','44','--timestep','0.0005','--contact-time','0.004'],
):
    print('VARIANT',args,flush=True);sys.argv=[sys.argv[0],*args];experiment.main()
