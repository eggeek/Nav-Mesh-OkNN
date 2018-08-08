import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

alias = {}
styles = {}

def load_data(fname):
    with open(fname, "r") as f:
        # ignore `vertices: ...` at head and `Total: ..` at tail
        raws = f.readlines()
    header = [i.strip() for i in raws[0].split(',')]
    lines = [[j.strip() for j in i.split(',')] for i in raws[1:]]
    t = pd.DataFrame.from_records(lines, columns=header)
    res = t[header].apply(pd.to_numeric, errors='ignore')
    return res

def load_files(paths):
    frames = []
    for i in paths:
        print(i)
        frames.append(load_data(i))
    res = pd.concat(frames)
    return res

def gen_xy(df=None, colx='', coly='', ignore=True, limit=20):
    tg = df.groupby(colx)
    x = []
    y = []
    for k, v in tg[coly].apply(lambda _: np.average(_)).items():
        if ignore and tg.size()[int(k)] < limit:
            continue
        x.append(int(k))
        y.append(v)
    return x, y

def plot_graph(xlabel='', ylabel='', xs=[[]], ys=[[]], labels=[], color=None, 
               yscale='log', xscale=None, ylim=None, xlim=None, saveto=None, xticks=None,
               loc='out'):
    
    fig, ax = plt.subplots()
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_yscale(yscale)
    font = {'family': 'monospace',
            'weight': 'bold',
            'size': 22}
    plt.rc('font',**font)
    plt.rc("text", usetex=True)
    if xscale is not None:
      ax.set_xscale(xscale)
    if ylim is not None:
        ax.set_ylim(ylim)
    if xlim is not None:
        ax.set_xlim(xlim)
    if xticks is not None:
      print(xticks)
      plt.xticks(xticks[0], xticks[1])

    n = len(xs)
    for i in range(n):
        x = xs[i]
        y = ys[i]
        if styles.get(labels[i]) is not None:
            plt.plot(x, y, styles[labels[i]], label=labels[i])
        else:
            ax.scatter(x, y)
            ax.plot(x, y, label=labels[i])
    ax.legend(labels)
    if loc == 'in':
      ax.legend(loc='best', fancybox=True, framealpha=0, prop={'size': 15})
    else:
      ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.25), ncol=3, fancybox=True, prop={'size': 15})
    plt.grid(True)
    if saveto is not None:
        fig.savefig(saveto, bbox_inches='tight')
