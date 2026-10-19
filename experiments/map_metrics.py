"""Calculate coverage and map quality for the TurtleBot3 world."""
from functools import lru_cache
from pathlib import Path
import numpy as np
# Inner wall vertices in metres: mesh scale 0.25, yaw -pi/2.
INNER=np.array([[2.932938,0],[1.466469,2.54],[-1.466469,2.54],[-2.932938,0],[-1.466469,-2.54],[1.466469,-2.54]])
ROI_AREA=float(abs(np.dot(INNER[:,0],np.roll(INNER[:,1],1))-np.dot(INNER[:,1],np.roll(INNER[:,0],1)))/2)
def inside(x,y):
    # Include polygon boundary cells.
    cross=[]
    for a,b in zip(INNER,np.roll(INNER,-1,axis=0)):
        cross.append((b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0]))
    return np.logical_and.reduce([c>=-1e-6 for c in cross]) | np.logical_and.reduce([c<=1e-6 for c in cross])
@lru_cache(maxsize=1)
def reference_points():
    points=[]
    for a,b in zip(INNER,np.roll(INNER,-1,axis=0)):
        n=max(2,int(np.linalg.norm(b-a)/.05)+1)
        points.extend(a[None,:]+np.linspace(0,1,n)[:,None]*(b-a)[None,:])
    for cx in [-1.1,0,1.1]:
        for cy in [-1.1,0,1.1]:
            theta=np.linspace(0,2*np.pi,40,endpoint=False)
            points.extend(np.column_stack([cx+.15*np.cos(theta),cy+.15*np.sin(theta)]))
    return np.array(points)
def evaluate(a,resolution,origin):
    yy,xx=np.indices(a.shape);x=origin[0]+(xx+.5)*resolution;y=origin[1]+(yy+.5)*resolution
    roi=inside(x,y);known=(a>=0)&roi;occupied=(a>=65)&roi
    coverage=float(known.sum()*resolution**2/ROI_AREA*100)
    pts=np.column_stack([x[occupied],y[occupied]]);ref=reference_points()
    if len(pts):
        # Score cylinder regions; exclude wall/tower intersections.
        near=np.zeros(len(pts),dtype=bool);agree=np.zeros(len(pts),dtype=bool)
        for cx in [-1.1,0,1.1]:
            for cy in [-1.1,0,1.1]:
                dist=np.hypot(pts[:,0]-cx,pts[:,1]-cy)
                near |= dist<.45;agree |= abs(dist-.15)<=.15
        precision=float(agree[near].mean()*100) if near.any() else 0.
        supported=np.concatenate([((chunk[:,None,:]-pts[None,:,:])**2).sum(axis=2).min(axis=1)<=.15**2 for chunk in np.array_split(ref,20) if len(chunk)])
        completeness=float(supported.mean()*100)
    else:precision=completeness=0.
    mask=a>=65;padded=np.pad(mask,1);neighbors=np.zeros(a.shape,dtype=int)
    for dy in [-1,0,1]:
        for dx in [-1,0,1]:
            if dx or dy:neighbors+=padded[1+dy:1+dy+a.shape[0],1+dx:1+dx+a.shape[1]]
    continuity=float(((neighbors>=2)&occupied).sum()/max(1,occupied.sum())*100)
    # Apply the fixed quality thresholds.
    if coverage>=80 and completeness>=75 and precision>=90:rating='++++'
    elif coverage>=60 and completeness>=55 and precision>=85:rating='+++-'
    elif coverage>=40 and completeness>=30 and precision>=75:rating='++--'
    elif coverage>=20 and precision>=60:rating='+---'
    else:rating='----'
    return dict(coverage_percent=coverage,rating=rating,known_cells_in_roi=int(known.sum()),occupied_cells_in_roi=int(occupied.sum()),roi_area_m2=ROI_AREA,wall_surface_completeness_percent=completeness,occupied_wall_agreement_percent=precision,wall_neighbor_continuity_percent=continuity,wall_tolerance_m=.15,rating_basis='Fixed inner-hexagon coverage, inner-wall/cylinder reference completeness and cylinder radial agreement. Green towers excluded from agreement; no fitted registration.')
