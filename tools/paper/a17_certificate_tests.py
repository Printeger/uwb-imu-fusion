#!/usr/bin/env python3
"""Meaningful exact/interval adversarial cases, fixed 333-bit precision."""
import json,math,sys,struct
def nextafter(x,towards):
 bits=struct.unpack('Q',struct.pack('d',x))[0];return struct.unpack('d',struct.pack('Q',bits+(1 if towards>x else -1)))[0]
from fractions import Fraction as F
from a17_certificate_worker import choose,PairEvaluator,I,M,mp
from a16_reference import exactD
results=[]
def expect(name,p,d,want):
 r=choose(p,d);assert r['status']==want,(name,r,want)
 if want=='ACCEPT':
  q=float.fromhex(r['fidelity_hex']);assert F.from_float(q)<=d.lo.fraction()/p.hi.fraction();assert F.from_float(q)>F.from_float(.001)
 results.append({'name':name,'expected':want,'actual':r,'pass':True})
expect('positive',I(1),I(.5),'ACCEPT')
expect('negative_D',I(1),I(-.1),'REJECT')
expect('zero_D',I(1),I(0),'REJECT')
expect('negative_P',I(-1),I(.1),'REJECT')
expect('zero_P',I(0),I(.1),'REJECT')
expect('uncertain_D_sign',I(1),I(-1e-18,1e-18),'NUMERIC_REDUCTION_UNRESOLVED')
expect('uncertain_P_sign',I(-1e-18,1e-18),I(.1),'NUMERIC_REDUCTION_UNRESOLVED')
# The offsets are exact binary64 constants; native addition loses the small offset.
a=I(2.**40)+I(2.**-20);b=I(2.**40);D=exactD([a],[b]);assert float(2.**40+2.**-20)==2.**40
expect('large_objectives_cancel_but_paired_D_positive',I(2.**20),D,'ACCEPT')
# Opposite small endpoint errors reverse a truly negative fixed-formula decrease.
ref0=I(1);ref1=I(1)+I(2.**-52);native0=nextafter(1.,math.inf);native1=nextafter(1.,0.)
assert .5*(native0-native1)*(native0+native1)>0
expect('native_false_descent_rejected',I(1),exactD([ref0],[ref1]),'REJECT')
expect('fidelity_low',I(1),I(.0005),'REJECT')
expect('fidelity_equal',I(1),I(.001),'REJECT')
expect('fidelity_high',I(1),I(.002),'ACCEPT')
t=.001;expect('fidelity_straddles_threshold',I(1),I(nextafter(t,0),nextafter(t,math.inf)),'NUMERIC_REDUCTION_UNRESOLVED')
expect('RNDD_conversion_cannot_cross_threshold_upward',I(1),I(M(t)+M(2.**-100)),'NUMERIC_REDUCTION_UNRESOLVED')
expect('wide_D',I(1),I(.1,.2),'NUMERIC_REDUCTION_UNRESOLVED')
expect('wide_P',I(.1,.2),I(.1),'NUMERIC_REDUCTION_UNRESOLVED')
expect('nonfinite_P',I(float('inf')),I(1),'NUMERIC_REFERENCE_NONFINITE')
expect('nonfinite_D',I(1),I(float('-inf')),'NUMERIC_REFERENCE_NONFINITE')
raw={('CONST',0,'log_near_pi_threshold'):[[1e-3]],('CONST',0,'log_near_zero_threshold'):[[-1e-6]]};ev=PairEvaluator(raw,I)
def fails(name,fn,kind):
 try:fn();raise AssertionError('failed to reject '+name)
 except kind as e:results.append({'name':name,'failure':type(e).__name__+':'+str(e),'pass':True})
fails('unknown_factor',lambda:ev.residual({'factor':'0','type':'UNKNOWN','keys':'v0:0'},'BASE'),ValueError)
fails('unsupported_near_pi',lambda:ev.log([[I(1),I(0),I(0)],[I(0),I(-1),I(0)],[I(0),I(0),I(-1)]],'BASE',0,'imu_log'),ArithmeticError)
fails('branch_ambiguous',lambda: I(M(-1e-6)-M(2.**-80),M(-1e-6)+M(2.**-80))<I(-1e-6),ArithmeticError)
# Division is enclosed independently using exact rational values.
r=I(M(.2),M(.3))/I(M(.7),M(.8));assert r.lo.fraction()<=F.from_float(.2)/F.from_float(.8);assert r.hi.fraction()>=F.from_float(.3)/F.from_float(.7)
results.append({'name':'directed_ratio_rational_enclosure','pass':True})
from a17_certificate_worker import certified_norm
z=certified_norm([I(-2.**-100,2.**-100),I(0),I(0)]);assert z.lo==0 and M(2.**-100)<=z.hi
assert certified_norm([I(0),I(0),I(0)]).bounds()==(F(0),F(0))
results.append({'name':'zero_angle_norm_square_enclosure','pass':True})
print(json.dumps({'bits':333,'cases':results,'pass':all(x['pass'] for x in results)},indent=2))
