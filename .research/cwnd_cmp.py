# cwnd_gain 1x vs 2x comparison — 8-phase BBR cycle
import random, math
BBR_UNIT=256; BW=1260.0; BWbps=BW*1e6; BD=BWbps/8; TP=35000; BDB=BD*TP*1e-6; BDPp=BDB/1448
CYCLE=[int(BBR_UNIT*1.25),int(BBR_UNIT*0.75)]+[BBR_UNIT]*6
for cw_gain,label in [(BBR_UNIT,'1x'),(BBR_UNIT*2,'2x=BBR')]:
    for nf in [1,2,4,8]:
        rng=random.Random(42)
        brtts=[max(3000,TP+rng.randint(-500,500)) for _ in range(nf)]
        at=sum(brtts)//nf; pg=[BBR_UNIT]*nf; cw=[cw_gain]*nf
        qs=[]; rates=[]
        for rd in range(400):
            tw=sum(pg); rs=max(1e-9,at*1e-6)
            for _ in range(8):
                tr=0.0; ki=0.0
                for i in range(nf):
                    pa=BWbps*pg[i]/BBR_UNIT; cr=(BDPp*cw[i]/BBR_UNIT)*1448*8/rs
                    tr+=min(pa,cr); ki+=min(pa,cr)*rs/8/1448
                tr=min(tr,BWbps); qb=max(0.0,ki*1448-BDB); rs=at*1e-6+qb/BD
            qu=qb/BD*1e6
            for i in range(nf): pg[i]=CYCLE[rd&7]
            if rd>=200: qs.append(qu); rates.append(tr/1e6)
        qs.sort(); nq=len(qs); avg_r=sum(rates)/len(rates)
        print(f"{label} N={nf}: Rate={avg_r:.0f}Mbps Q50={qs[nq//2]:.0f}us Q95={qs[min(nq-1,int(nq*0.95))]:.0f}us")
