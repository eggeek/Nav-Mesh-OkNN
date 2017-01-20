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

    retval <- list("header"=rawmesh[1,1:2], "points"=points, "polys"=polys, "verts_per_poly"=verts_per_poly)
    retval
}

draw_mesh <- function(filename, poly_colour="lightgray")
{
    rawmesh <- load_mesh(filename)

    yrange <- range(pretty(rawmesh$points[,2]-1))
    xrange <- range(pretty(rawmesh$points[,1])-1)
    plot(NA, xlim=xrange, ylim=yrange, main=filename, yaxt="n", xaxt="n")

    axis(1, at=pretty(xrange), tick=TRUE, labels=TRUE)
    axis(2, at=pretty(yrange), tick=TRUE, labels=TRUE)

    for(i in seq(1, nrow(rawmesh$polys)))
    {
        polygon(rawmesh$points[unlist(rawmesh$polys[i, seq_len(aurora$verts_per_poly[i])]),], col=poly_colour)
    }
}

