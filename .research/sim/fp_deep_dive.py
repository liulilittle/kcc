#!/usr/bin/env python3
"""G3 false-positive deep dive: when does G3 commit, does it self-correct?"""
import random, math
random.seed(42)

KCC_SCALE=1024
G2_N,G2_D=122,1000
G3_FN,G3_FD=11,10
G3_SN,G3_SD=21,20
G3_FC,G3_SC=4,5
RTT_MIN=1
STICKY_N,STICKY_D=75,100
FF_DIV=4
FF_CNT=5
BIT3=7
PD_N,PD_D=95,100
MS=5
STALE=128

def sim(tp, np, ma=20000):
    s=tp*np/100.0
    xe=tp*KCC_SCALE; mr=tp; sc=MS; cc=0; csc=0; mruc=0; rc=0; mrffc=0
    commits=[]; last_ra=-1; g3c=0
    for a in range(ma):
        ns=random.gauss(0,s); rtt=int(max(tp+ns,RTT_MIN)); z=rtt*KCC_SCALE
        inno=z-xe
        if inno<=0: xe=min(xe,z)
        else: gr=xe*G2_N//G2_D; xe=min(xe+gr,z)
        if rc-mruc>=STALE:
            ms_=mr*KCC_SCALE
            if xe<=ms_*G3_FN//G3_FD: xe=ms_*PD_N//PD_D; mruc=rc
        sc+=1
        rd=(a>0 and a%20==0)
        if rd: rc+=1
        tf=mr*KCC_SCALE*G3_FN//G3_FD; ts=mr*KCC_SCALE*G3_SN//G3_SD; bl=mr*KCC_SCALE
        if xe>=tf: cc=min(cc+1,255); csc=min(csc+1,255)
        elif xe>=ts: cc=0; csc=min(csc+1,255)
        else: cc=0
        if xe<=bl: cc=0; csc=0
        cm=False
        if cc>=G3_FC: mr=max(xe//KCC_SCALE,RTT_MIN); cc=0; csc=0; cm=True; g3c+=1; last_ra=a
        elif csc>=G3_SC: mr=max(xe//KCC_SCALE,RTT_MIN); cc=0; csc=0; cm=True; g3c+=1; last_ra=a
        if cm: commits.append((a,mr))
        if cc>0 or csc>0: pass
        else:
            if rtt<=mr:
                rtc=max(rtt,RTT_MIN)
                if rtc<mr*STICKY_N//STICKY_D:
                    if rtc<mr//FF_DIV: mr=rtc; mrffc=0
                    else: mrffc=min(mrffc+1,BIT3)
                    if mrffc>=FF_CNT: mr=rtc; mrffc=0
                    elif rd: mr=max(RTT_MIN,mr*STICKY_N//STICKY_D)
                else: mr=rtc; mrffc=0
            if sc>=MS and xe:
                k=xe//KCC_SCALE
                if k<mr and k<mr*PD_N//PD_D: mr=k; mruc=rc
    err=(mr-tp)/tp*100
    return g3c,last_ra,mr,tp,err

print(f"{'T_prop':>8} {'Noise':>6} {'Commits':>8} {'Last@ACK':>10} {'Final_mr':>10} {'True':>8} {'Err%':>7} {'SC?':>5}")
print("-"*70)
for tp in [500,1000,5000,10000,50000,100000,500000,1000000]:
    for n in [0.5,1.0,2.0,5.0,10.0]:
        g3c,la,mr,tp_,err=sim(tp,n)
        sc="Y" if mr<=tp_*1.01 else "N"
        print(f"{tp:>8} {n:>5.1f}% {g3c:>8} {la:>10} {mr:>8}us {tp_:>8}us {err:>6.1f}% {sc:>5}")
