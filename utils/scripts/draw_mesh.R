load_mesh <- function(filename)
{
    rawmesh <- read.csv(filename, sep="\n", header=TRUE)
    mesh_maxcols <- max(count.fields(filename))
    rawmesh <- as.data.frame( 
                    read.table(filename, header=FALSE, sep=" ", 
                          col.names=paste("X", seq_len(mesh_maxcols)), 
                          colClasses=rep("numeric", mesh_maxcols), fill=TRUE, skip=2))

    # read vertices
    nverts <- rawmesh[1,1]
    npolys <- rawmesh[1,2]
    points <- rawmesh[seq(2, nverts+1), c(1,2)]

    # R is 1-indexed; increment vertex ids for all polygons
    verts_per_poly <- unlist(rawmesh[seq(nverts+2, nrow(rawmesh)), 1])
    polys <- rawmesh[seq(nverts+2, nrow(rawmesh)), seq(2, mesh_maxcols)]+1

    retval <- list("header"=rawmesh[1,1:2], "points"=points, "polys"=polys, "verts_per_poly"=verts_per_poly, "filename"=filename)
    retval
}

draw_mesh <- function(rawmesh, poly_colour="lightgray")
{
    yrange <- range(pretty(rawmesh$points[,2]-1))
    xrange <- range(pretty(rawmesh$points[,1])-1)
    plot(NA, xlim=xrange, ylim=yrange, main=rawmesh$filename, yaxt="n", xaxt="n", xlab="", ylab="")

    axis(1, at=pretty(xrange), tick=TRUE, labels=TRUE)
    axis(2, at=pretty(yrange), tick=TRUE, labels=TRUE)

    for(i in seq(1, nrow(rawmesh$polys)))
    {
        polygon(rawmesh$points[unlist(rawmesh$polys[i, seq_len(rawmesh$verts_per_poly[i])]),], col=poly_colour)
    }
}

load_paths <- function(filename, path_colour="red")
{
    rawpaths <- read.csv(filename, sep="\n", header=FALSE)
    unlist(rawpaths)
}

# txt_path should be a vector of strings each having the form
# (x1, y1) (x2, y2) ... (xn, yn)
draw_path <- function(rawpaths, path_colour="red", prev_path_col="black")
{
    symbol_sz = 0.6
    line_width = 1

    prev_xy <- c()
    for(i in seq_len(length(rawpaths)))
    {
        tmp <- as.numeric(unlist(strsplit(gsub("\\(|\\)|,", "", rawpaths[i]), " ")))
        if(i > 1)
        {
            lines(prev_xy$points[prev_xy$x], prev_xy$points[prev_xy$y], col=prev_path_col, lwd=line_width)
            points(prev_xy$points[prev_xy$x], prev_xy$points[prev_xy$y], col=prev_path_col, lwd=line_width, cex=symbol_sz)
        }
        path_xy <- list(
                        "points"=tmp,
                        "x"=seq(1, length(tmp), by=2),
                        "y"=seq(2, length(tmp), by=2))
        print(as.vector(rawpaths[i]))
        lines(path_xy$points[path_xy$x], path_xy$points[path_xy$y], col=path_colour, lwd=line_width)
        points(path_xy$points[path_xy$x], path_xy$points[path_xy$y], col=path_colour, lwd=line_width, cex=symbol_sz)
        readline(prompt="Press [enter] to continue")
        prev_xy <- path_xy
    }
}



