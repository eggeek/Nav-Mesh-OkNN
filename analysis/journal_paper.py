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

  # xs[1], ys[1] = gen_xy(df, 'pts', 'cost_ki0', limit=limit)
  # algos.append(alias['poly-zero'])

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

  # xs[1], ys[1] = gen_xy(df, 'pts', 'gen_ki0', limit=limit)
  # algos.append(alias['poly-zero'])

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
