#!/usr/bin/env python3
"""Generate trajectory comparison and error analysis plots for LaTeX slides."""
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os, json

OUT = "/home/mint/ws_fusion_uwb/src/uwb-imu-fusion/logs/plots"
os.makedirs(OUT, exist_ok=True)

def load_tum(path):
    data = np.loadtxt(path)
    return data[:,0], data[:,1:4]

def umeyama_align(est_t, est_p, gt_t, gt_p):
    """SE(3) Umeyama alignment: returns R(3x3), t(3,), aligned_est_p(Nx3)."""
    # Match timestamps
    ep, gp = [], []
    for i in range(len(est_t)):
        j = np.searchsorted(gt_t, est_t[i])
        if j >= len(gt_t): continue
        if j > 0 and j < len(gt_t):
            d1 = gt_t[j] - est_t[i]; d0 = est_t[i] - gt_t[j-1]
            j = j-1 if d0 < d1 else j
        ep.append(est_p[i]); gp.append(gt_p[j])
    if len(ep) < 3:
        print(f"  WARNING: only {len(ep)} matched points for alignment")
        return np.eye(3), np.zeros(3), est_p
    ep = np.array(ep); gp = np.array(gp)
    # Centroids
    ce = ep.mean(0); cg = gp.mean(0)
    # Cross-covariance
    H = (ep - ce).T @ (gp - cg)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1; R = Vt.T @ U.T
    t = cg - R @ ce
    # Apply to all points
    aligned = (R @ est_p.T).T + t
    return R, t, aligned

datasets = [
    # --- VIUNet ---
    {"name":"VIUNet_00","label":"VIUNet (4 ceiling anchors)",
     "log":"/home/mint/ws_fusion_uwb/src/uwb-imu-fusion/logs/2026-08-07_17-34-42_VIUNet_00","color":"#2196F3",
     "anchors":np.array([[-0.76,2.68,2.64],[-0.76,-2.32,2.64],[1.24,-2.32,2.64],[1.24,2.68,2.64]]),
     "anchor_ids":["A1","A2","A3","A4"]},
    # --- MILUV ---
    {"name":"MILUV_circular","label":"MILUV Circular",
     "log":"/home/mint/ws_fusion_uwb/src/uwb-imu-fusion/logs/2026-08-07_22-44-53_default_1_circular3D_0","color":"#4CAF50",
     "anchors":np.array([[3.27,3.46,1.81],[3.19,0.27,1.59],[2.85,-2.92,1.90],[-2.50,-3.50,1.77],[-2.96,0.61,1.66],[-2.73,3.66,1.89]]),
     "anchor_ids":["A0","A1","A2","A3","A4","A5"]},
    {"name":"MILUV_random","label":"MILUV Random",
     "log":"/home/mint/ws_fusion_uwb/src/uwb-imu-fusion/logs/2026-08-07_23-30-39_default_1_random3_0","color":"#FF9800",
     "anchors":np.array([[3.27,3.46,1.81],[3.19,0.27,1.59],[2.85,-2.92,1.90],[-2.50,-3.50,1.77],[-2.96,0.61,1.66],[-2.73,3.66,1.89]]),
     "anchor_ids":["A0","A1","A2","A3","A4","A5"]},
    # --- SFUISE ISAS-Walk (corrected IMU noise) ---
    {"name":"SFUISE_Walk1","label":"SFUISE Walk1 (5 anchors)",
     "log":"/home/mint/ws_fusion_uwb/src/uwb-imu-fusion/logs/sfuise/walk1_v2","color":"#E91E63",
     "anchors":np.array([[0,0,0],[2.61,2.67,0],[5.52,0.05,1.86],[3.12,-2.59,1.85],[5.5,0,0]]),
     "anchor_ids":["A0","A1","A2","A3","A4"]},
    {"name":"SFUISE_Walk2","label":"SFUISE Walk2 (5 anchors)",
     "log":"/home/mint/ws_fusion_uwb/src/uwb-imu-fusion/logs/sfuise/walk2_v2","color":"#9C27B0",
     "anchors":np.array([[0,0,0],[2.61,2.67,0],[5.52,0.05,1.86],[3.12,-2.59,1.85],[5.5,0,0]]),
     "anchor_ids":["A0","A1","A2","A3","A4"]},
    {"name":"SFUISE_Walk3","label":"SFUISE Walk3 (5 anchors)",
     "log":"/home/mint/ws_fusion_uwb/src/uwb-imu-fusion/logs/sfuise/walk3_v2","color":"#00BCD4",
     "anchors":np.array([[0,0,0],[2.61,2.67,0],[5.52,0.05,1.86],[3.12,-2.59,1.85],[5.5,0,0]]),
     "anchor_ids":["A0","A1","A2","A3","A4"]},
]
nds = len(datasets)

# ====== 3D Trajectory ======
fig = plt.figure(figsize=(16,9))
for i, ds in enumerate(datasets):
    ax = fig.add_subplot(2,3,i+1, projection='3d')
    te,pe=load_tum(os.path.join(ds["log"],"trajectory.txt"))
    tg,pg=load_tum(os.path.join(ds["log"],"groundtruth.txt"))
    R_align, t_align, pe_aligned = umeyama_align(te, pe, tg, pg)
    # Also align anchors
    anc_aligned = None
    if "anchors" in ds:
        anc_aligned = (R_align @ ds["anchors"].T).T + t_align
    s=max(1,len(te)//600); sg=max(1,len(tg)//600)
    ax.plot(pe_aligned[::s,0],pe_aligned[::s,1],pe_aligned[::s,2],color=ds["color"],lw=1,label='Estimate',alpha=0.85)
    ax.plot(pg[::sg,0],pg[::sg,1],pg[::sg,2],color='black',lw=0.4,ls='--',label='GT',alpha=0.5)
    # Plot anchors
    if anc_aligned is not None:
        ax.scatter(anc_aligned[:,0],anc_aligned[:,1],anc_aligned[:,2],c='red',s=60,marker='^',zorder=10,label='Anchors',edgecolors='darkred',linewidth=0.5)
        for j,(x,y,z) in enumerate(anc_aligned):
            ax.text(x,y,z,ds["anchor_ids"][j],fontsize=6,color='darkred',ha='center',va='bottom')
    ax.set_title(ds["label"],fontsize=10); ax.legend(fontsize=7,loc='upper left')
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.view_init(elev=25, azim=-60)
plt.tight_layout(); plt.savefig(os.path.join(OUT,"traj_3d.png"),dpi=200,bbox_inches='tight'); plt.close()
print("traj_3d.png saved")

# ====== XY Top-down ======
fig, axes = plt.subplots(2,3,figsize=(14,9))
axes=axes.flatten()
for i, ds in enumerate(datasets):
    ax=axes[i]
    te,pe=load_tum(os.path.join(ds["log"],"trajectory.txt"))
    tg,pg=load_tum(os.path.join(ds["log"],"groundtruth.txt"))
    R_align, t_align, pe_aligned = umeyama_align(te, pe, tg, pg)
    anc_aligned = None
    if "anchors" in ds:
        anc_aligned = (R_align @ ds["anchors"].T).T + t_align
    ax.plot(pe_aligned[:,0],pe_aligned[:,1],color=ds["color"],lw=0.8,label='Estimate')
    ax.plot(pg[:,0],pg[:,1],color='black',lw=0.4,ls='--',label='GT')
    # Plot anchors
    if anc_aligned is not None:
        ax.scatter(anc_aligned[:,0],anc_aligned[:,1],c='red',s=80,marker='^',zorder=10,label='Anchors',edgecolors='darkred',linewidth=0.5)
        for j,(x,y,z) in enumerate(anc_aligned):
            ax.annotate(ds["anchor_ids"][j],(x,y),fontsize=7,color='darkred',ha='center',va='bottom',xytext=(0,-8),textcoords='offset points')
    ax.set_title(ds["label"],fontsize=10); ax.axis('equal')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.legend(fontsize=7); ax.grid(alpha=0.3)
plt.tight_layout(); plt.savefig(os.path.join(OUT,"traj_xy.png"),dpi=200,bbox_inches='tight'); plt.close()
print("traj_xy.png saved")

# ====== ATE Bar Chart ======
fig,ax=plt.subplots(figsize=(12,5))
names=[d["label"] for d in datasets]
ate=[0.59,0.88,1.18,0.170,0.233,0.167]
p95=[0.96,1.57,2.46,0.290,0.471,0.281]
x=np.arange(nds); w=0.32
b1=ax.bar(x-w/2,ate,w,label='ATE RMSE',color=[d["color"] for d in datasets],edgecolor='white')
b2=ax.bar(x+w/2,p95,w,label='ATE P95',color=['#90CAF9','#A5D6A7','#FFCC80','#F48FB1','#CE93D8','#80DEEA'],edgecolor='white')
ax.set_xticks(x); ax.set_xticklabels(names,fontsize=10)
ax.set_ylabel('Error (m)',fontsize=11); ax.legend(fontsize=10); ax.grid(axis='y',alpha=0.3)
for b in b1: ax.text(b.get_x()+b.get_width()/2,b.get_height()+0.03,f'{b.get_height():.2f}',ha='center',fontsize=10,fontweight='bold')
for b in b2: ax.text(b.get_x()+b.get_width()/2,b.get_height()+0.03,f'{b.get_height():.2f}',ha='center',fontsize=10)
ax.set_ylim(0,max(p95)*1.15)
plt.tight_layout(); plt.savefig(os.path.join(OUT,"ate_bar.png"),dpi=200,bbox_inches='tight'); plt.close()
print("ate_bar.png saved")

# ====== ATE vs Height Spread ======
fig,ax=plt.subplots(figsize=(9,5.5))
sv=[4.0,0.0,0.31,0.31,1.86,1.86,1.86]; av=[0.095,0.59,0.88,1.18,0.170,0.233,0.167]
lbl=['Sim\n(8 anchors)','VIUNet_00\n(4 anchors)','MILUV circ\n(6 anchors)','MILUV rand\n(6 anchors)',
      'SFUISE W1\n(5 anchors)','SFUISE W2\n(5 anchors)','SFUISE W3\n(5 anchors)']
clr=['#9C27B0','#2196F3','#4CAF50','#FF9800','#E91E63','#9C27B0','#00BCD4']
ax.scatter(sv,av,c=clr,s=250,zorder=5,edgecolors='black',linewidth=0.8)
for i in range(len(sv)): ax.annotate(lbl[i],(sv[i],av[i]),fontsize=8,ha='center',va='bottom',xytext=(0,10),textcoords='offset points')
ax.set_xlabel('Anchor Height Spread Δz (m)',fontsize=12)
ax.set_ylabel('ATE RMSE (m)',fontsize=12)
ax.set_title('ATE vs Anchor Height Diversity',fontsize=13,fontweight='bold')
ax.grid(alpha=0.3)
z=np.polyfit(sv,av,1); xf=np.linspace(-0.3,4.5,100)
ax.plot(xf,np.polyval(z,xf),'--',color='gray',alpha=0.5,lw=2,
        label=f'Linear fit: ATE ≈ {z[0]:.2f}·Δz + {z[1]:.2f}')
ax.legend(fontsize=9); ax.set_xlim(-0.3,4.5); ax.set_ylim(0,1.4)
plt.tight_layout(); plt.savefig(os.path.join(OUT,"ate_vs_spread.png"),dpi=200,bbox_inches='tight'); plt.close()
print("ate_vs_spread.png saved")

# ====== Error over time ======
fig, axes = plt.subplots(2,3,figsize=(14,9))
axes=axes.flatten()
for i, ds in enumerate(datasets):
    ax=axes[i]
    te,pe=load_tum(os.path.join(ds["log"],"trajectory.txt"))
    tg,pg=load_tum(os.path.join(ds["log"],"groundtruth.txt"))
    R_align, t_align, pe_aligned = umeyama_align(te, pe, tg, pg)
    errs=[]; ts=[]
    for j in range(len(te)):
        idx=np.searchsorted(tg,te[j])
        if idx>=len(tg): continue
        g=pg[min(idx,len(pg)-1)]
        errs.append(np.linalg.norm(pe_aligned[j]-g))
        ts.append(te[j]-te[0])
    ax.plot(ts,errs,color=ds["color"],lw=0.8)
    ax.set_title(ds["label"],fontsize=10)
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Pos Error (m)'); ax.grid(alpha=0.3)
plt.tight_layout(); plt.savefig(os.path.join(OUT,"error_timeline.png"),dpi=200,bbox_inches='tight'); plt.close()
print("error_timeline.png saved")

print("\nAll plots saved to", OUT)
