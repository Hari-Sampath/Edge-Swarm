"""Summarize measured trials and save one matplotlib figure."""
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path
import matplotlib.pyplot as plt

METRICS=('makespan','total_distance','collisions','wait_steps')

def summarize(path):
    """Print grouped measured statistics and save summary.png beside the CSV."""
    source=Path(path); groups=defaultdict(list)
    with source.open(newline='') as stream:
        for row in csv.DictReader(stream):groups[float(row['packet_loss'])].append(row)
    print('loss | metric: mean, standard deviation, min, max | timed_out')
    for loss,rows in sorted(groups.items()):
        timeouts=sum(row['timed_out'].lower()=='true' for row in rows)
        for name in METRICS:
            values=[float(row[name]) for row in rows]
            mean=statistics.mean(values); spread=statistics.stdev(values) if len(values)>1 else 0.0
            print(f'{loss:.1f} | {name}: {mean:.2f}, {spread:.2f}, {min(values):.0f}, {max(values):.0f} | {timeouts}')
    fig,axes=plt.subplots(2,2,figsize=(10,7))
    for ax,name in zip(axes.flat,METRICS):
        means=[]; errors=[]; losses=sorted(groups)
        for loss in losses:
            vals=[float(row[name]) for row in groups[loss]]
            means.append(statistics.mean(vals)); errors.append(statistics.stdev(vals) if len(vals)>1 else 0.0)
        ax.errorbar(losses,means,yerr=errors,marker='o',capsize=4)
        ax.set(title=name.replace('_',' '),xlabel='Packet loss',ylabel=f'Mean measured {name}')
        ax.grid(True,alpha=.3)
    fig.tight_layout(); output=source.parent/'summary.png'; fig.savefig(output,dpi=160)
    print(f'plot: {output}')

if __name__=='__main__':summarize(sys.argv[1] if len(sys.argv)>1 else 'results/trials.csv')
