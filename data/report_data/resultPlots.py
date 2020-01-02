import numpy as np
import matplotlib.pyplot as plt

def getStats(p,x,dt):
    maxP = np.max(p)
    idxP1 = np.argmax(p)
    settleArea = np.sum(np.abs(p[idxP1:]))
    if settleArea > 200:
        idxP2 = len(p) - 1
    else:
        idxP2 = np.where(p[idxP1:] < 0.1*maxP)[0][0] + idxP1
    settleP = p[idxP2]
    riseTime = (idxP1 - 10000) * dt
    fallTime = (idxP2 - idxP1) * dt
    return maxP, settleArea, settleP,idxP1, idxP2, riseTime, fallTime

def addTable(labels, stats, ax):
    columns = labels[0]
    rows = labels[1]

    # stats = list(map(list, zip(*stats)))

    n_rows = len(stats)
    cell_txt = []
    for row in range(n_rows):
        cell_txt.append(['%1.3f' % x for x in stats[row]])

    ax.table(cellText=cell_txt,
             rowLabels=rows,
             colLabels=columns,
             loc='top',
             bbox=[0.20, 1.05, 0.8, 0.4])



def annotate(p,x,dt,ax):
    maxP, _, settleP, idxP1, idxP2,_, _ = getStats(p,x,dt)
    arrowprops = dict(facecolor='black', width=0.5, headwidth=4, headlength=5)
    ax.annotate('max', xy=(idxP1*dt , maxP), xytext=(idxP2*dt - 1, maxP + 0.05) , arrowprops=arrowprops)
    ax.annotate('settled', xy=(idxP2*dt , settleP), xytext=(idxP2*dt + 1, settleP - 0.05) , arrowprops=arrowprops)
    # txt = []
    # txt.append(['max p:', '{:.3f} m'.format(maxP)]) 
    # txt.append(['max x:', '{:.3f} m'.format(maxX)])
    # txt.append(['rise time:', '{:.3f} s'.format(riseTime)])
    # txt.append(['settle time:', '{:.3f} s'.format(fallTime)])
    # plt.table(cellText = txt, loc='lower left', colWidths=[0.2]*len(txt), cellLoc='left')

def create_plot(filename):
    data = np.genfromtxt(filename, delimiter=',')

    p = data[:,0]
    x = data[:,1]

    dt = 0.001
    t = np.linspace(0, x.size*dt, x.size)


    plt.figure()
    ax = plt.subplot(1,1,1)
    ax.plot(t, p, linewidth=2.0, label='p')
    ax.plot(t, x, linewidth=2.0, label='x')
    if data.shape[0] == 20000 and data.shape[1] > 2:
        p_ref = data[:,2]
        ax.plot(t, p_ref, linewidth=1.0, label='$p_{ref}$')

    ax.plot([ 0.105]* 20000, 'g--', linewidth=2.0, label = 'SB boundry')
    ax.plot([-0.105]* 20000, 'g--', linewidth=2.0,)
    if data.shape[0] == 20000:
        annotate(p,x,dt,ax)


    ax.legend(loc=2)
    plt.title(filename[0:-4])
    plt.xlabel('t in [s]')
    plt.ylabel('displacement in [m]')
    plt.ylim([-0.21, 0.21])
    plt.xlim([0, 20])

    plt.savefig('figs/{}.pdf'.format(filename[0:-4]))
    # plt.show()


def create_combines_plot(filenames):
    data = {}
    dt = 0.001
    stats = []
    # plt.figure()
    fig, axes = plt.subplots(nrows=2, ncols=2, sharex='col', sharey='row')
    axes = axes.flatten()
    for i, filename in enumerate(filenames):
        data[filename] = np.genfromtxt(filename, delimiter=',')
        t = np.linspace(0, len(data[filename])*dt, len(data[filename]))
        p = data[filename][:,0] 
        x = data[filename][:,1]
        maxP, settleA, _, _, _,risetime, falltime = getStats(p,x,dt)
        stats.append([maxP, settleA, risetime, falltime])
        ax = axes[i]
        ax.plot(t, p, linewidth=2.0, label='p')
        ax.plot(t, x, linewidth=2.0, label='x')
        ax.plot(t, [ 0.105]* 20000, 'g--', linewidth=2.0, label = 'SB boundry')
        ax.plot(t, [-0.105]* 20000, 'g--', linewidth=2.0,)
        annotate(p,x,dt,ax)
        ax.set_ylim([-0.21, 0.21])
        ax.set_xlim([0, 20])

    plt.subplots_adjust(top=0.7)    

    fig.add_subplot(111, frameon=False)
    # hide tick and tick label of the big axes
    plt.tick_params(labelcolor='none', top='off', bottom='off', left='off', right='off')
    plt.grid(False)

    
    rows = [filename[:-4] for filename in filenames]
    columns = ['maxP', 'area after peak','riseTime', 'fallTime' ]
    labels = [columns, rows]
    addTable(labels, stats, fig.axes[-1])
    # plt.legend()
    plt.xlabel('t in [s]')
    plt.ylabel('displacement in [m]')
    # plt.show()
    plt.savefig('figs/combined.pdf')

filenames = ['NoControl.csv', 'PControl.csv', 'LQRControl.csv', 'CPControl.csv']
# for filename in filenames:
#     create_plot(filename)
create_combines_plot(filenames)