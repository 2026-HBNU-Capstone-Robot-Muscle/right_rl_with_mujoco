"""Final-grip transport variant: ordinary READ, unchanged guarded motor writes."""
from leap_forward_hold_observe_20261104 import ForwardHoldBus
from final_grip_standard_read_20261116 import block_once,POLICY

class StandardGripBus(ForwardHoldBus):
    def _block_once(self,address,length,fields):
        if address>=64 and address+length<=147:
            self.report['final_grip_read_policy']=dict(POLICY)
            return block_once(self,64,83,fields)
        return super()._block_once(address,length,fields)
    def _receive(self,operation,label,request):
        if request.get('kind')=='sync_read' and request.get('address',0)>=64 and request.get('address',0)+request.get('length',0)<=147:
            request=dict(request,original_read_address=request['address'],original_read_length=request['length'],address=64,length=83)
        return super()._receive(operation,label,request)
