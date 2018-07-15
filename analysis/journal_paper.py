from utilities import alias
from utilities import plot_graph
from utilities import gen_xy

def nn_experiment_time(DF, time_ylim=None, saveto=None, limit=10):
  """
    x-aixs: number of targets
    y-aixs: time cost
  """
  df = DF.copy()
  xs = {}
  ys = {}
  algos = []

  xs[0], ys[0] = gen_xy(df, 'pts', 'cost_fi', limit=limit)
  algos.append(alias['fence nn'])

  xs[1], ys[1] = gen_xy(df, 'pts', 'cost_ffp', limit=limit)
  algos.append("ffp")

  xs[2], ys[2] = gen_xy(df, 'pts', 'cost_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[3], ys[3] = gen_xy(df, 'pts', 'cost_hi', limit=limit)
  algos.append(alias['target heuristic'])

  xs[4], ys[4] = gen_xy(df, 'pts', 'cost_poly', limit=limit)
  algos.append(alias['polyanya'])

  for i in xs.keys():
    xs[i] = [v for v in xs[i]]
    ys[i] = [v / 1000 for v in ys[i]]

  plot_graph('target', 'time(ms) (log10)', list(xs.values()), list(ys.values()), algos,
      ylim=time_ylim, saveto=saveto)


def nn_experiment_gen(DF, gen_ylim=None, saveto=None, limit=10):
  """
    x-aixs: number of targets
    y-aixs: number of generated nodes
  """
  df = DF.copy()
  xs = {}
  ys = {}
  algos = []

  xs[0], ys[0] = gen_xy(df, 'pts', 'gen_fi', limit=limit)
  algos.append(alias['fence nn'])

  xs[1], ys[1] = gen_xy(df, 'pts', 'gen_ffp', limit=limit)
  algos.append("ffp")

  xs[2], ys[2] = gen_xy(df, 'pts', 'gen_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[3], ys[3] = gen_xy(df, 'pts', 'gen_hi', limit=limit)
  algos.append(alias['target heuristic'])

  xs[4], ys[4] = gen_xy(df, 'pts', 'gen_poly', limit=limit)
  algos.append(alias['polyanya'])

  for i in xs.keys():
    xs[i] = [v for v in xs[i]]

  plot_graph('target', 'generated (log10)', list(xs.values()), list(ys.values()), algos,
      ylim=gen_ylim, saveto=saveto)


def cluster_improve_time(DF, size=100, limit=10, time_ylim=None, saveto=None):
  df = DF.copy()
  df.gen_ki0 = (df.gen_ki0 / size).astype(int)
  df['imp_ki'] = df.cost_ki0 / df.cost_ki
  df['imp_hi'] = df.cost_ki0 / df.cost_hi
  df['imp_poly'] = df.cost_ki0 / df.cost_poly
  xs = {}
  ys = {}
  algos = []

# xs[0], ys[0] = gen_xy(df, 'dist', 'cost_ki0', limit=blimit)
# algos.append(alias['poly-zero'])

  xs[1], ys[1] = gen_xy(df, 'gen_ki0', 'imp_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = gen_xy(df, 'gen_ki0', 'imp_hi', limit=limit)
  algos.append(alias['target heuristic'])

  xs[3], ys[3] = gen_xy(df, 'gen_ki0', 'imp_poly', limit=limit)
  algos.append(alias['polyanya'])

  for k in xs.keys():
      xs[k] = [v * size for v in xs[k]]
#       ys[k] = [v / 1000 for v in ys[k]]
#     ys[k] = np.log10(ys[k])
      
  plot_graph('gen poly0', 'speed up (log10)', list(xs.values()), list(ys.values()), algos, 
             ylim=time_ylim, saveto=saveto, xscale='log')


def cluster_improve_gen(DF, size=100, limit=10, gen_ylim=None, saveto=None):
  df = DF.copy()
  df['imp_ki'] = df.gen_ki0 / df.gen_ki
  df['imp_hi'] = df.gen_ki0 / df.gen_hi
  df['imp_poly'] = df.gen_ki0 / df.gen_poly
  df.gen_ki0 = (df.gen_ki0 / size).astype(int)
  xs = {}
  ys = {}
  algos = []

# xs[0], ys[0] = gen_xy(df, 'dist', 'cost_ki0', limit=blimit)
# algos.append(alias['poly-zero'])

  xs[1], ys[1] = gen_xy(df, 'gen_ki0', 'imp_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = gen_xy(df, 'gen_ki0', 'imp_hi', limit=limit)
  algos.append(alias['target heuristic'])

  xs[3], ys[3] = gen_xy(df, 'gen_ki0', 'imp_poly', limit=limit)
  algos.append(alias['polyanya'])

  for k in xs.keys():
      xs[k] = [v * size for v in xs[k]]
      
  plot_graph('gen poly0', 'reduce (log10)', list(xs.values()), list(ys.values()), algos, 
             ylim=gen_ylim, saveto=saveto, xscale='log')


def euclidean_rank_time(DF, limit=10, time_ylim=None, saveto=None):
  df = DF.copy()
  xs = {}
  ys = {}
  algos = []

  df['imp_ki'] = df.cost_ki0 / df.cost_ki
  df['imp_hi'] = df.cost_ki0 / df.cost_hi
  df['imp_poly'] = df.cost_ki0 / df.cost_poly

  xs[1], ys[1] = gen_xy(df, 'rank', 'imp_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = gen_xy(df, 'rank', 'imp_hi', limit=limit)
  algos.append(alias['target heuristic'])

  xs[3], ys[3] = gen_xy(df, 'rank', 'imp_poly', limit=limit)
  algos.append(alias['polyanya'])

  plot_graph('Euclidean rank', 'speedup vs poly-zero', list(xs.values()), list(ys.values()),
            algos, ylim=time_ylim, saveto=saveto)

def euclidean_rank_gen(DF, limit=10, gen_ylim=None, saveto=None):
  df = DF.copy()
  xs = {}
  ys = {}
  algos = []

  df['imp_ki'] = df.gen_ki0 / df.gen_ki
  df['imp_hi'] = df.gen_ki0 / df.gen_hi
  df['imp_poly'] = df.gen_ki0 / df.gen_poly

  xs[1], ys[1] = gen_xy(df, 'rank', 'imp_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = gen_xy(df, 'rank', 'imp_hi', limit=limit)
  algos.append(alias['target heuristic'])

  xs[3], ys[3] = gen_xy(df, 'rank', 'imp_poly', limit=limit)
  algos.append(alias['polyanya'])

  plot_graph('Euclidean rank', 'generated nodes vs poly-zero', list(xs.values()), list(ys.values()),
          algos, ylim=gen_ylim, saveto=saveto)
