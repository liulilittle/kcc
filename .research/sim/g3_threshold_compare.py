#!/usr/bin/env python3
"""Compare G3 thresholds: old (3/4) vs current (4/5) for false-positive + detection latency."""
import random, math
random.seed(42)

KCC_SCALE=1024
G2_N,G2_D=122,1000
G3_FN,G3_FD=11,10   # 1.10x
G3_SN,G3_SD=21,20   # 1.05x
RTT_MIN=1
STICKY_N,STICKY_D=75,100
FF_DIV=4; FF_CNT=5; BIT3=7
PD_N,PD_D=95,100; MS=5; STALE=128

T_PROPS=[500,1000,5000,10000,50000,100000,500000,1000000]
NOISES=[0.5,1.0,2.0,5.0,10.0]

def sim(tp,np,fast_cnt,slow_cnt,path_change=None,path_factor=1.0,ma=20000):
    s=tp*np/100.0
    xe=tp*KCC_SCALE; mr=tp; sc=MS; cc=0; csc=0; mruc=0; rc=0; mrffc=0
    g3c=0; det_ack=-1; real_tp=tp; path_done=False
    for a in range(ma):
        if path_change and a>=path_change and not path_done:
            real_tp=int(tp*path_factor); s=real_tp*np/100.0; path_done=True
        ns=random.gauss(0,s); rtt=int(max(real_tp+ns,RTT_MIN)); z=rtt*KCC_SCALE
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
        if cc>=fast_cnt:
            mr=max(xe//KCC_SCALE,RTT_MIN); cc=0; csc=0; g3c+=1
            if path_done and det_ack<0 and mr>=real_tp*95//100: det_ack=a
        elif csc>=slow_cnt:
            mr=max(xe//KCC_SCALE,RTT_MIN); cc=0; csc=0; g3c+=1
            if path_done and det_ack<0 and mr>=real_tp*95//100: det_ack=a
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
    err=(mr-real_tp)/real_tp*100
    return g3c,mr,real_tp,err,det_ack

print("="*90)
print("G3 threshold comparison: old (fast=3, slow=4) vs current (fast=4, slow=5)")
print("="*90)

# Part 1: false-positive rate (pure noise, 20000 ACKs)
print("\n--- PART 1: FALSE-POSITIVE RATE UNDER PURE NOISE ---")
print(f"{'T_prop':>8} {'Noise':>6} {'G3c(3/4)':>10} {'G3c(4/5)':>10} {'Err%(3/4)':>10} {'Err%(4/5)':>10} {'SC(3/4)':>8} {'SC(4/5)':>8}")
print("-"*80)
fp34,fp45=0,0
for tp in T_PROPS:
    for n in NOISES:
        c34,m34,_,e34,_=sim(tp,n,3,4)
        c45,m45,_,e45,_=sim(tp,n,4,5)
        sc34="Y" if m34<=tp*1.01 else "N"
        sc45="Y" if m45<=tp*1.01 else "N"
        if c34>0: fp34+=1
        if c45>0: fp45+=1
        if n in [1.0,5.0,10.0] or c34>0 or c45>0:
            print(f"{tp:>8} {n:>5.1f}% {c34:>8} {c45:>10} {e34:>8.1f}% {e45:>10.1f}% {sc34:>8} {sc45:>8}")
print(f"\nTotal scenarios with G3 commits: current(3/4)={fp34}, proposed(4/5)={fp45}")

# Part 2: detection latency (path change at ACK 500)
print("\n--- PART 2: DETECTION LATENCY (path change at ACK 500) ---")
PATH_FACTORS=[1.05,1.10,1.25,1.50,2.0,5.0,10.0,20.0]
print(f"{'T_prop':>8} {'Noise':>6} ", end="")
for pf in PATH_FACTORS: print(f"{pf:>6.2f}x", end="")
print()
for tp in [10000, 100000]:
    for n in [1.0]:
        print(f"{tp:>8} {n:>5.1f}% curr ", end="")
        for pf in PATH_FACTORS:
            _,_,_,_,da=sim(tp,n,3,4,path_change=500,path_factor=pf)
            d=da-500 if da>=0 else -1
            print(f"{d:>6}", end="")
        print()
        print(f"{'':>8} {'':>6} prop ", end="")
        for pf in PATH_FACTORS:
            _,_,_,_,da=sim(tp,n,4,5,path_change=500,path_factor=pf)
            d=da-500 if da>=0 else -1
            print(f"{d:>6}", end="")
        print()
