"""Position-only TCP IK using the existing SO-101 URDF chain; MoveIt validates collisions."""
import time
import numpy as np
from scipy.optimize import least_squares


def tcp_candidates(chain, positions, offset, target, limits, tolerance, budget_s):
    names = chain.names
    initial = np.array([positions[n] for n in names])
    lower, upper = np.array([limits[n] for n in names]).T
    margin = 1e-6
    rng = np.random.default_rng(101)
    seeds = [initial, (lower+upper)/2]
    seeds += [np.clip(initial+rng.normal(0,.5,len(names)),lower+margin,upper-margin) for _ in range(6)]
    seeds += [rng.uniform(lower+margin,upper-margin) for _ in range(8)]
    deadline = time.monotonic() + budget_s
    seen=[]
    for seed in seeds:
        if time.monotonic() > deadline:
            break
        seed=np.clip(seed,lower+margin,upper-margin)
        def residual(q):
            point,_=chain.evaluate(dict(zip(names,q)),offset)
            return np.r_[point-target, (q-seed)*1e-4]
        def jacobian(q):
            _,jac=chain.evaluate(dict(zip(names,q)),offset)
            return np.vstack((jac,np.eye(len(names))*1e-4))
        result=least_squares(residual,seed,jac=jacobian,bounds=(lower+margin,upper-margin),max_nfev=200,
                             ftol=1e-9,xtol=1e-9,gtol=1e-9)
        point,_=chain.evaluate(dict(zip(names,result.x)),offset)
        if np.linalg.norm(point-target)>tolerance*.5:
            continue
        if any(np.linalg.norm(result.x-q)<.02 for q in seen):
            continue
        seen.append(result.x.copy())
        yield dict(zip(names,map(float,result.x)))
