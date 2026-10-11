"""Plot measured trial makespan and packet loss using matplotlib."""
import csv
import matplotlib.pyplot as plt

with open('results/trials.csv', newline='') as stream:
    rows=list(csv.DictReader(stream))
for loss in sorted({row['packet_loss'] for row in rows}):
    sample=[int(row['makespan']) for row in rows if row['packet_loss']==loss]
    plt.plot(range(1,len(sample)+1),sample,marker='o',label=f'packet loss {loss}')
plt.xlabel('Trial number'); plt.ylabel('Measured makespan (ticks)'); plt.grid(True); plt.legend(); plt.tight_layout(); plt.savefig('results/trials.png',dpi=160)
