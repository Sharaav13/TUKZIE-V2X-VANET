"""Censored fit with a soft demodulation threshold.
Pr(d) ~ N(P0 - 10 a log10 d, sigma); P(decode | Pr) = Phi((Pr - S)/tau)
received (exact SNR): pdf(Pr) * Phi((Pr-S)/tau);  received, SNR >= SAT: P(Pr >= Pr_obs) (approx, decode ~ 1)
lost: Phi((S - mu)/sqrt(sigma^2 + tau^2))
"""
import csv, math, sys
import numpy as np
from scipy.optimize import minimize
from scipy.stats import norm
N_FLOOR=-117.3; LINK=13.72; SAT=10.0
P=[r for r in csv.DictReader(open(sys.argv[1])) if r['dir']=='2->1' and r['overlap']=='False']
d=np.maximum(np.array([float(r['dist']) for r in P]),1.0); rx=np.array([r['delivered']=='True' for r in P])
snr=np.array([float(r['snr']) for r in P]); pr=N_FLOOR+snr; x=10*np.log10(d)
ex=rx&(snr<SAT); sat=rx&(snr>=SAT)
def nll(th, fixed={}):
    p=dict(zip(['P0','a','ls','S','lt'],th)); p.update(fixed)
    s=math.exp(p['ls']); t=math.exp(p['lt']); mu=p['P0']-p['a']*x
    ll=norm.logpdf(pr[ex],mu[ex],s).sum()+norm.logcdf((pr[ex]-p['S'])/t).sum()
    ll+=norm.logsf(pr[sat],mu[sat],s).sum()
    ll+=norm.logcdf((p['S']-mu[~rx])/math.hypot(s,t)).sum()
    return -ll
def fit(fixed={}):
    names=[n for n in ['P0','a','ls','S','lt'] if n not in fixed]
    x0={'P0':-75,'a':2.5,'ls':math.log(5),'S':-136,'lt':math.log(1.5)}
    f=lambda th: nll(th if not fixed else None, {}) if False else nll_wrap(th,names,fixed)
    r=minimize(f,[x0[n] for n in names],method='Nelder-Mead',options=dict(maxiter=40000,maxfev=40000,xatol=1e-5,fatol=1e-7))
    p=dict(zip(names,r.x)); p.update(fixed); return p,r.fun
def nll_wrap(th,names,fixed):
    p=dict(zip(names,th)); p.update(fixed); return nll([p[k] for k in ['P0','a','ls','S','lt']])
def show(tag,p,f):
    print(f"{tag}: P0={p['P0']:.2f} dBm  alpha={p['a']:.3f}  sigma={math.exp(p['ls']):.2f} dB  S={p['S']:.2f} dBm (SNR {p['S']-N_FLOOR:.2f})  tau={math.exp(p['lt']):.2f}  -LL={f:.2f}  PL1m={LINK-p['P0']:.2f}  simRef(0dBi)={10-p['P0']:.2f}")
p,f=fit(); show('free',p,f)
for a in (2.0,2.5,2.7,3.0,3.5): show(f'a={a}',*fit({'a':a}))
show('S=-137',*fit({'S':-137.0}))
# bootstrap-free CI via profile on alpha already; PDR table
def pdr(p,dd):
    mu=p['P0']-p['a']*10*np.log10(np.maximum(dd,1)); return norm.sf((p['S']-mu)/math.hypot(math.exp(p['ls']),math.exp(p['lt'])))
print('bin      n  meas  model   meanSNR(meas)')
for lo in range(0,375,25):
    m=(d>=lo)&(d<lo+25)
    if m.sum(): print(f'{lo:3d}-{lo+25:3d} {m.sum():3d} {100*rx[m].mean():5.1f} {100*pdr(p,d[m]).mean():5.1f}  {np.nanmean(snr[m&rx]) if (m&rx).any() else float("nan"):6.2f}')
import json; json.dump(p,open('fit2.json','w'))
