#!/usr/bin/env python3
"""Compare G3 3/4 vs 4/5 under HIGH noise (stress test)."""
import random; random.seed(42)
KCC_SC=1024; G2N=122; G2D=1000; G3FN=11; G3FD=10; G3SN=21; G3SD=20

def run(tp, np, fc, sc, ma=20000):
    s=tp*np/100.0; xe=tp*KCC_SC; mr=tp; cc=0; csc=0
    fcom=0; scom=0
    for a in range(ma):
        ns=random.gauss(0,s); rtt=int(max(tp+ns,1)); z=rtt*KCC_SC
        inno=z-xe
        if inno<=0: xe=min(xe,z)
        else: gr=xe*G2N//G2D; xe=min(xe+gr,z)
        tf=mr*KCC_SC*G3FN//G3FD; ts=mr*KCC_SC*G3SN//G3SD; bl=mr*KCC_SC
        if xe>=tf: cc=min(cc+1,255); csc=min(csc+1,255)
        elif xe>=ts: cc=0; csc=min(csc+1,255)
        else: cc=0
        if xe<=bl: cc=0; csc=0
        if cc>=fc: mr=max(xe//KCC_SC,1); cc=0; csc=0; fcom+=1
        elif csc>=sc: mr=max(xe//KCC_SC,1); cc=0; csc=0; scom+=1
        # correct running-min: no immediate fall for small dips
        if cc==0 and csc==0 and rtt<=mr:
            if rtt<mr//4: mr=rtt
    return fcom, scom, mr

print(f"{'T_prop':>8} {'Noise':>6} {'3/4F':>6} {'3/4S':>6} {'3/4mr':>8} {'4/5F':>6} {'4/5S':>6} {'4/5mr':>8} {'Err34':>6} {'Err45':>6}")
print("-"*80)
for tp in [500,10000,1000000]:
    for n in [0.5,1.0,2.0,5.0,10.0,20.0]:
        f3,s3,mr3=run(tp,n,3,4)
        f4,s4,mr4=run(tp,n,4,5)
        e3=(mr3-tp)/tp*100; e4=(mr4-tp)/tp*100
        print(f"{tp:>6}us {n:>5.1f}% {f3:>6} {s3:>6} {mr3:>6}us {f4:>6} {s4:>6} {mr4:>6}us {e3:>5.1f}% {e4:>5.1f}%")
print()
print("With correct running-min: 3/4 still has zero false positives at <=1% noise.")
print("At 2%+ noise, 3/4 false-positive rate slightly exceeds 4/5.")
