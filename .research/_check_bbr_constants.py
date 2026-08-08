import re
kcc = open(r'D:\dd\ucp\tcp_kcc.c', encoding='utf-8-sig').read()
bbr = open(r'D:\dd\ucp\google\patch\tcp_bbr1.c', encoding='utf-8-sig').read()
pairs = [
    ('KCC_HIGH_GAIN', 'bbr_high_gain', r'BBR_UNIT\s*\*\s*2885\s*/\s*1000'),
    ('KCC_DRAIN_GAIN', 'bbr_drain_gain', r'BBR_UNIT\s*\*\s*1000\s*/\s*2885'),
    ('KCC_CWND_GAIN', 'bbr_cwnd_gain', r'BBR_UNIT\s*\*\s*2'),
]
for kname, bname, expr in pairs:
    kfound = re.search(expr, kcc)
    bfound = re.search(expr, bbr)
    kexpr = kfound.group(0) if kfound else 'NOT FOUND'
    bexpr = bfound.group(0) if bfound else 'NOT FOUND'
    status = 'MATCH' if kfound and bfound else 'MISMATCH'
    print(f'{kname}/{bname}: {status} ({kexpr} == {bexpr})')
