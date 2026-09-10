"""A16 scalar MPFR + outward interval arithmetic; no optimizer, no numpy.
Only the existing libmpfr ABI is used. Every imported binary64 is exact.
"""
import ctypes as C
from fractions import Fraction
from decimal import Decimal,localcontext,ROUND_FLOOR,ROUND_CEILING
L=C.CDLL('libmpfr.so.6'); PREC=167
class Raw(C.Structure):
 _fields_=[('prec',C.c_long),('sign',C.c_int),('exp',C.c_long),('limbs',C.POINTER(C.c_ulong))]
P=C.POINTER(Raw)
def decl(name,args,result=None):
 f=getattr(L,'mpfr_'+name);f.argtypes=args;f.restype=result;return f
init=decl('init2',[P,C.c_long]);clear=decl('clear',[P]);setd=decl('set_d',[P,C.c_double,C.c_int],C.c_int);setsi=decl('set_si',[P,C.c_long,C.c_int],C.c_int);copy=decl('set',[P,P,C.c_int],C.c_int)
ops={k:decl(k,[P,P,P,C.c_int],C.c_int) for k in ['add','sub','mul','div']}
unary={k:decl(k,[P,P,C.c_int],C.c_int) for k in ['neg','sqrt','sin','cos','acos','tan']}
cmp=decl('cmp',[P,P],C.c_int);getd=decl('get_d',[P,C.c_int],C.c_double);number=decl('number_p',[P],C.c_int)
getstr=decl('get_str',[C.c_void_p,C.POINTER(C.c_long),C.c_int,C.c_size_t,P,C.c_int],C.c_void_p);freestr=decl('free_str',[C.c_void_p]);version=decl('get_version',[],C.c_char_p)
N,U,D=0,2,3
class M:
 def __init__(self,x=0):
  self.raw=Raw();init(self.raw,PREC)
  if isinstance(x,M):copy(self.raw,x.raw,N)
  elif isinstance(x,int):setsi(self.raw,x,N)
  else:setd(self.raw,float(x),N)
 def __del__(self):clear(self.raw)
 def op(self,kind,b,r=N):
  b=b if isinstance(b,M) else M(b);z=M();ops[kind](z.raw,self.raw,b.raw,r);return z
 def fun(self,kind,r=N):
  z=M();unary[kind](z.raw,self.raw,r)
  if not number(z.raw):raise ArithmeticError('MPFR nonfinite '+kind)
  return z
 def __add__(self,b):return self.op('add',b)
 __radd__=__add__
 def __sub__(self,b):return self.op('sub',b)
 def __rsub__(self,b):return M(b).op('sub',self)
 def __mul__(self,b):return self.op('mul',b)
 __rmul__=__mul__
 def __truediv__(self,b):return self.op('div',b)
 def __rtruediv__(self,b):return M(b).op('div',self)
 def __neg__(self):return self.fun('neg')
 def __lt__(self,b):return cmp(self.raw,(b if isinstance(b,M) else M(b)).raw)<0
 def __le__(self,b):return cmp(self.raw,(b if isinstance(b,M) else M(b)).raw)<=0
 def __gt__(self,b):return not self.__le__(b)
 def __ge__(self,b):return not self.__lt__(b)
 def __eq__(self,b):return isinstance(b,(M,int,float)) and cmp(self.raw,(b if isinstance(b,M) else M(b)).raw)==0
 def __float__(self):return getd(self.raw,N)
 def fraction(self):
  e=C.c_long();s=getstr(None,C.byref(e),2,0,self.raw,N)
  try:d=C.string_at(s).decode()
  finally:freestr(s)
  n=int(d,2);q=e.value-len(d.lstrip('-'));return Fraction(n*(2**max(0,q)),2**max(0,-q)) if n else Fraction(0)
 def __repr__(self):return dec(self.fraction(),35)
 def bounds(self):return self.fraction(),self.fraction()
class I:
 def __init__(self,x=0,hi=None):
  if isinstance(x,I):self.lo,self.hi=x.lo,x.hi
  else:self.lo=x if isinstance(x,M) else M(x);self.hi=self.lo if hi is None else (hi if isinstance(hi,M) else M(hi))
  if self.hi<self.lo:raise ArithmeticError('reversed interval')
 def __add__(self,b):
  b=I(b);return I(self.lo.op('add',b.lo,D),self.hi.op('add',b.hi,U))
 __radd__=__add__
 def __neg__(self):return I(self.hi.fun('neg'),self.lo.fun('neg'))
 def __sub__(self,b):return self+-I(b)
 def __rsub__(self,b):return I(b)+-self
 def __mul__(self,b):
  b=I(b);pairs=[(a,c) for a in (self.lo,self.hi) for c in (b.lo,b.hi)];return I(min(a.op('mul',c,D) for a,c in pairs),max(a.op('mul',c,U) for a,c in pairs))
 __rmul__=__mul__
 def __truediv__(self,b):
  b=I(b)
  if b.lo<=0 and 0<=b.hi:raise ArithmeticError('division contains zero')
  pairs=[(a,c) for a in (self.lo,self.hi) for c in (b.lo,b.hi)];return I(min(a.op('div',c,D) for a,c in pairs),max(a.op('div',c,U) for a,c in pairs))
 def __rtruediv__(self,b):return I(b)/self
 def __lt__(self,b):
  b=I(b)
  if self.hi<b.lo:return True
  if b.hi<=self.lo:return False
  raise ArithmeticError('uncertified < branch')
 def __le__(self,b):
  b=I(b)
  if self.hi<=b.lo:return True
  if b.hi<self.lo:return False
  raise ArithmeticError('uncertified <= branch')
 def __gt__(self,b):return I(b).__lt__(self)
 def __ge__(self,b):return I(b).__le__(self)
 def fun(self,kind):
  if kind=='sqrt':
   if self.lo<0:raise ArithmeticError('negative sqrt bound')
   return I(self.lo.fun(kind,D),self.hi.fun(kind,U))
  if kind=='acos':
   if self.lo< -1 or 1<self.hi:raise ArithmeticError('acos domain')
   return I(self.hi.fun(kind,D),self.lo.fun(kind,U))
  if kind in ('sin','cos'):
   mid=(self.lo+self.hi)/2
   radius=max(mid.op('sub',self.lo,U),self.hi.op('sub',mid,U))
   return I(max(M(-1),mid.fun(kind,D).op('sub',radius,D)),min(M(1),mid.fun(kind,U).op('add',radius,U)))
  if kind=='tan':
   if self.lo< -1.5 or 1.5<self.hi:raise ArithmeticError('tan monotonic domain uncertified')
   return I(self.lo.fun(kind,D),self.hi.fun(kind,U))
  raise ValueError(kind)
 def bounds(self):return self.lo.fraction(),self.hi.fraction()
 def __repr__(self):return '['+repr(self.lo)+','+repr(self.hi)+']'
def dec(q,digits=110,up=False):
 with localcontext() as c:
  c.prec=digits;c.rounding=ROUND_CEILING if up else ROUND_FLOOR
  return str(Decimal(q.numerator)/Decimal(q.denominator))
def describe(x,digits=110):
 lo,hi=x.bounds();return {'lo':dec(lo,digits),'hi':dec(hi,digits,True),'halfwidth_upper':dec((hi-lo)/2,digits,True)}
def selftest():
 import struct
 cases=[]
 for bits in (167,333):
  global PREC;PREC=bits
  # Exact IEEE-754 conversion, including a non-decimal rational and subnormal.
  for x in [0.,-.1,1.23456789,2.**-1074,2.**1023]:
   assert M(x).fraction()==Fraction.from_float(x)
  q=I(1)/3;assert q.lo.fraction()<=Fraction(1,3)<=q.hi.fraction()
  s=I(2).fun('sqrt');assert s.lo.fraction()**2<=2<=s.hi.fraction()**2
  t=I(M(.1),M(.2)).fun('sin');assert t.lo< M(.1).fun('sin') and M(.2).fun('sin')<t.hi
  for a,b in [(-3,2),(3,-2),(-3,-2)]:
   assert (I(a)*I(b)).bounds()==(Fraction(a*b),Fraction(a*b))
  try:I(-1,1)/I(-1,1);raise AssertionError('division should fail')
  except ArithmeticError:pass
  cases.append({'bits':bits,'exact_import':True,'rational_bounds':True,'sqrt_bounds':True,'sin_lipschitz':True,'zero_division_rejected':True})
 return {'mpfr':version().decode(),'ctypes_raw_size':C.sizeof(Raw),'checks':cases}
if __name__=='__main__':
 import json;print(json.dumps(selftest(),indent=2))
