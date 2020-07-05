# Polyanya

Polyanya is a compromise-free pathfinding algorithm over navigation meshes (Read the paper [here](http://www.ijcai.org/proceedings/2017/0070.pdf).)
The implementation of Polyanya can be found [here](https://bitbucket.org/mlcui1/polyanya).

# Replicating our experiments

0. unzip `movingai.zip` to a folder (e.g. `input-data`)
1. `./gen_pts.sh` generate all entitiy set and query points
2. `make fast` to compile executables
3. `./run-exp.sh` to run all experiments (`small`, `random`, `cluster`, `nn`)
  * you can run single experiments by `./run-exp.sh <name>` (e.g. `./run-exp.sh small`)
  * you can run parallel by set `execute=0` in `run-exp.sh` and `./run-exp <name> | parallel`,
  this requires `GNU parallel` be installed.
4. logs are stored in `output/{small,random,cluster,nn}/*.log`.

# BibTeX
```
@proceedings{DBLP:conf/socs/2018,
  editor    = {Vadim Bulitko and
               Sabine Storandt},
  title     = {Proceedings of the Eleventh International Symposium on Combinatorial
               Search, {SOCS} 2018, Stockholm, Sweden - 14-15 July 2018},
  publisher = {{AAAI} Press},
  year      = {2018},
  url       = {http://www.aaai.org/Library/SOCS/socs18contents.php},
  isbn      = {978-1-57735-802-2},
  timestamp = {Tue, 24 Jul 2018 20:17:24 +0200},
  biburl    = {https://dblp.org/rec/bib/conf/socs/2018},
  bibsource = {dblp computer science bibliography, https://dblp.org}
}
```

# License

This implementation is licensed under GPLv2. Please refer to
`LICENSE` for more information.

Note that several source files from Daniel Harabor's
[Warthog project](https://bitbucket.org/dharabor/pathfinding)
(also licensed under GPLv2) were used in this project with their permission.
These files are:
`helpers/cfg.cpp`, `helpers/cfg.h`, `helpers/cpool.h`, `helpers/timer.cpp` and
`helpers/timer.h`.

Additionally, Fade2D is used to generate triangulations for use with this
with this implementation. Please note that commercial use of Fade2D requires
a valid commercial license.
