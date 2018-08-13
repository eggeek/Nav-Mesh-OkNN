import utilities as ut
from utilities import alias
from utilities import plot_graph
from utilities import gen_xy

def experiment1_dense_time(DF, time_ylim=None, size=50, blimit=10):
  saveto = './figs/e1_dense_time.png'
  df = DF.copy()
  df = df[df.k == 1]
  df.dist = (df.dist / size).astype(int)
  xs = {}
  ys = {}
  algos = []

  xs[0], ys[0] = gen_xy(df, 'dist', 'cost_ffp', limit=blimit)
  algos.append(alias['rePolyanya'])

  xs[1], ys[1] = ut.gen_xy(df, 'dist', 'cost_ki', limit=blimit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = ut.gen_xy(df, 'dist', 'cost_hi', limit=blimit)
  algos.append(alias['target heuristic'])

  xs[3], ys[3] = ut.gen_xy(df, 'dist', 'cost_edbt', limit=blimit)
  algos.append('LVG')

  for k in xs.keys():
      xs[k] = [v * size for v in xs[k]]
      ys[k] = [v / 1000 for v in ys[k]]
#     ys[k] = np.log10(ys[k])
      
  ut.plot_graph('dist', 'time(ms) (log10)', list(xs.values()), list(ys.values()), algos, 
             ylim=time_ylim, saveto=saveto)


def experiment1_dense_gen(DF, gen_ylim=None, size=50, limit=10):
  # dense: gen
  saveto = './figs/e1_dense_gen.png'
  df = DF.copy()
  df = df[df.k == 1]
  df.dist = (df.dist / size).astype(int)
  xs = {}
  ys = {}
  algos = []

  xs[0], ys[0] = gen_xy(df, 'dist', 'gen_ffp', limit=limit)
  algos.append(alias['rePolyanya'])

  xs[1], ys[1] = ut.gen_xy(df, 'dist', 'gen_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = ut.gen_xy(df, 'dist', 'gen_hi', limit=limit)
  algos.append(alias['target heuristic'])

  xs[3], ys[3] = ut.gen_xy(df, 'dist', 'gen_edbt', limit=limit)
  algos.append('LVG')

  for k in xs.keys():
      xs[k] = [v * size for v in xs[k]]
#     ys[k] = np.log10(ys[k])
      
  ut.plot_graph('dist', 'generated (log10)', list(xs.values()), list(ys.values()), algos, 
             ylim=gen_ylim, saveto=saveto)


def experiment1_sparse_time(DF, time_ylim, size=500, limit=10):
  # sparse: time
  saveto = './figs/e1_sparse_time.png'
  df = DF.copy()
  df = df[(df.k == 1) & (df.pts == 10)]
  df.dist = (df.dist / size).astype(int)
  xs = {}
  ys = {}
  algos = []

  xs[0], ys[0] = gen_xy(df, 'dist', 'cost_ffp', limit=limit)
  algos.append(alias['rePolyanya'])

  xs[1], ys[1] = ut.gen_xy(df, 'dist', 'cost_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = ut.gen_xy(df, 'dist', 'cost_hi', limit=limit)
  algos.append(alias['target heuristic'])


  for k in xs.keys():
      xs[k] = [v * size for v in xs[k]]
      ys[k] = [v / 1000 for v in ys[k]]
      
  ut.plot_graph('dist', 'time(ms) (log10)', list(xs.values()), list(ys.values()), algos,
             ylim=time_ylim, saveto=saveto)

def experiment1_sparse_gen(DF, gen_ylim, limit=10, size=500):
  # sparse: gen
  saveto = './figs/e1_sparse_gen.png'
  df = DF.copy()
  df = df[(df.k == 1) & (df.pts == 10)]
  df.dist = (df.dist / size).astype(int)
  xs = {}
  ys = {}
  algos = []

  xs[0], ys[0] = gen_xy(df, 'dist', 'gen_ffp', limit=limit)
  algos.append(alias['rePolyanya'])

  xs[1], ys[1] = ut.gen_xy(df, 'dist', 'gen_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = ut.gen_xy(df, 'dist', 'gen_hi', limit=limit)
  algos.append(alias['target heuristic'])

  for k in xs.keys():
      xs[k] = [v * size for v in xs[k]]
      
  ut.plot_graph('dist', 'generated (log10)', list(xs.values()), list(ys.values()), algos,
             ylim=gen_ylim, saveto=saveto)

def experiment2_dense_time(DF, time_ylim=None, limit=10):
  # dense: time
  saveto = './figs/e2_dense_time.png'
  df = DF.copy()
  df = df[(df.polys >= 8000) & (df.k <= 10)]
  xs = {}
  ys = {}
  algos = []

  xs[0], ys[0] = gen_xy(df, 'k', 'cost_ffp', limit=limit)
  algos.append(alias['rePolyanya'])

  xs[1], ys[1] = gen_xy(df, 'k', 'cost_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = gen_xy(df, 'k', 'cost_hi', limit=limit)
  algos.append(alias['target heuristic'])

  xs[3], ys[3] = gen_xy(df, 'k', 'cost_edbt', limit=limit)
  algos.append('LVG')

  xs[4], ys[4] = gen_xy(df, 'k', 'cost_fi', limit=limit)
  algos.append(alias['fence heuristic'])

  for k in xs.keys():
      xs[k] = [v for v in xs[k]]
      ys[k] = [v / 1000 for v in ys[k]]
      
  ut.plot_graph('k', 'time(ms) (log10)', list(xs.values()), list(ys.values()), algos,
             ylim=time_ylim, saveto=saveto)

def experiment2_dense_gen(DF, gen_ylim, limit=10):
  # dense: gen
  saveto = './figs/e2_dense_gen.png'

  df = DF.copy()
  df = df[(df.polys >= 8000) & (df.k <= 10)]
  xs = {}
  ys = {}
  algos = []

  xs[0], ys[0] = gen_xy(df, 'k', 'cost_ffp', limit=limit)
  algos.append(alias['rePolyanya'])

  xs[1], ys[1] = ut.gen_xy(df, 'k', 'gen_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = ut.gen_xy(df, 'k', 'gen_hi', limit=limit)
  algos.append(alias['target heuristic'])

  xs[3], ys[3] = ut.gen_xy(df, 'k', 'gen_edbt', limit=limit)
  algos.append('LVG')

  for k in xs.keys():
      xs[k] = [v for v in xs[k]]
#     ys[k] = np.log10(ys[k])
      
  ut.plot_graph('k', 'generated (log10)', list(xs.values()), list(ys.values()), algos,
             ylim=gen_ylim, saveto=saveto)


def experiment2_sparse_time(DF, time_ylim, limit=10):
  # sparse: time
  saveto = './figs/e2_sparse_time.png'
  df = DF.copy()
  df = df[(df.polys >= 8000) & (df.pts == 10)]
  xs = {}
  ys = {}
  algos = []

# xs[0], ys[0] = gen_xy(df, 'k', 'cost_ki0', limit=limit)
# algos.append(alias['poly-zero'])

  xs[1], ys[1] = ut.gen_xy(df, 'k', 'cost_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = ut.gen_xy(df, 'k', 'cost_hi', limit=limit)
  algos.append(alias['target heuristic'])

  xs[3], ys[3] = ut.gen_xy(df, 'k', 'cost_ffp', limit=limit)
  algos.append(alias['rePolyanya'])

  for k in xs.keys():
      xs[k] = [v for v in xs[k]]
      ys[k] = [v / 1000 for v in ys[k]]
#     ys[k] = np.log10(ys[k])
  ut.plot_graph('k', 'time(ms) (log10)', list(xs.values()), list(ys.values()), algos, yscale='log',
             ylim=time_ylim, saveto=saveto)


def experiment2_sparse_gen(DF, gen_ylim, limit=10):
  # sparse: expanded
  saveto = './figs/e2_sparse_gen.png'
  df = DF.copy()
  df = df[(df.polys >= 8000) & (df.pts == 10)]
  xs = {}
  ys = {}
  algos = []

# xs[0], ys[0] = gen_xy(df, 'k', 'gen_ki0', limit=limit)
# algos.append(alias['poly-zero'])

  xs[1], ys[1] = gen_xy(df, 'k', 'gen_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = gen_xy(df, 'k', 'gen_hi', limit=limit)
  algos.append(alias['target heuristic'])

  xs[3], ys[3] = gen_xy(df, 'k', 'gen_ffp', limit=limit)
  algos.append(alias['rePolyanya'])

  for k in xs.keys():
      xs[k] = [v for v in xs[k]]
#     ys[k] = np.log10(ys[k])
  plot_graph('k', 'generated (log10)', list(xs.values()), list(ys.values()), algos,
             ylim=gen_ylim, saveto=saveto)


def experiment3_time(DF, time_ylim=None, limit=10):
  # sparse: time
  saveto = './figs/e3_time.png'
  df = DF.copy()
  df = df[(df.polys >= 8000) & (df.k == 1) & (df.pts <= 10)]
  xs = {}
  ys = {}
  algos = []

# xs[0], ys[0] = gen_xy(df, 'pts', 'cost_ki0', limit=limit)
# algos.append(alias['poly-zero'])

  xs[1], ys[1] = gen_xy(df, 'pts', 'cost_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = gen_xy(df, 'pts', 'cost_hi', limit=limit)
  algos.append(alias['target heuristic'])

  xs[3], ys[3] = gen_xy(df, 'pts', 'cost_ffp', limit=limit)
  algos.append(alias['rePolyanya'])

  for i in xs.keys():
      xs[i] = [v for v in xs[i]]
      ys[i] = [v / 1000 for v in ys[i]]
      
  plot_graph('target', 'time(ms) (log10)', list(xs.values()), list(ys.values()), algos,
             ylim=time_ylim, saveto=saveto)

def experiment3_gen(DF, gen_ylim, limit=10):
  # sparse: expaned
  saveto = './figs/e3_gen.png'
  df = DF.copy()
  df = df[(df.polys >= 8000) & (df.k == 1) & (df.pts <= 10)]
  xs = {}
  ys = {}
  algos = []

# xs[0], ys[0] = gen_xy(df, 'pts', 'gen_ki0', limit=limit)
# algos.append(alias['poly-zero'])

  xs[1], ys[1] = gen_xy(df, 'pts', 'gen_ki', limit=limit)
  algos.append(alias['interval heuristic'])

  xs[2], ys[2] = gen_xy(df, 'pts', 'gen_hi', limit=limit)
  algos.append(alias['target heuristic'])

  xs[3], ys[3] = gen_xy(df, 'pts', 'gen_ffp', limit=limit)
  algos.append(alias['rePolyanya'])

  for i in xs.keys():
      xs[i] = [v for v in xs[i]]
#     ys[i] = np.log10(ys[i])
  plot_graph('target', 'generated (log10)', list(xs.values()), list(ys.values()), algos,
             ylim=gen_ylim, saveto=saveto)
