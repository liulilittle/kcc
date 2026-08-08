#!/usr/bin/env python3
"""Debug: exactly when/how does G3 commit with 4/5 thresholds?"""
import random
random.seed(42)

KCC_SCALE=1024
G2_N,G2_D=122,1000
G3_FN,G3_FD=11,10; G3_SN,G3_SD=21,20
G3_FC,G3_SC=4,5
RTT_MIN=1
STICKY_N,STICKY_D=75,100
FF_DIV=4; FF_CNT=5; BIT3=7
PD_N,PD_D=95,100; MS=5; STALE=128

tp=500; np=5.0; ma=5000
s=tp*np/100.0
xe=tp*KCC_SCALE; mr=tp; sc=MS; cc=0; csc=0; mruc=0; rc=0; mrffc=0
fast_com=0; slow_com=0

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
    old_cc, old_csc = cc, csc

    if xe>=tf: cc=min(cc+1,255); csc=min(csc+1,255)
    elif xe>=ts: cc=0; csc=min(csc+1,255)
    else: cc=0
    if xe<=bl: cc=0; csc=0

    if cc>=G3_FC:
        fast_com+=1; cc=0; csc=0
        mr_old=mr; mr=max(xe//KCC_SCALE,RTT_MIN)
        if a>2000:  # only log later ones
            pass
    elif csc>=G3_SC:
        slow_com+=1; cc=0; csc=0
        mr_old=mr; mr=max(xe//KCC_SCALE,RTT_MIN)

    # running min + pull-down
    if cc==0 and csc==0 and rtt<=mr:
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

x_us=xe//KCC_SCALE
print(f"Results: {ma} ACKs, tp={tp}us, noise={np}%")
print(f"  Fast commits (cc>=4): {fast_com}")
print(f"  Slow commits (csc>=5): {slow_com}")
print(f"  Final mr={mr}us, true={tp}us, x_est={x_us}us")
print(f"  Final cc={cc}, csc={csc}")
