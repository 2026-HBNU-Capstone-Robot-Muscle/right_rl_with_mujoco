import sys
import search_level_book_support_221703_20261025 as experiment
for friction in (.2,.3,.5):
    args=['--s','42','44','46','--y','12','15','18','--finish-extra','128','--timestep','0.001','--contact-time','0.004','--friction',str(friction)]
    print('VARIANT',args,flush=True);sys.argv=[sys.argv[0],*args];experiment.main()
