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
    polys <- as.matrix(rawmesh[seq(nverts+2, nrow(rawmesh)), seq(2, mesh_maxcols)]+1)

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

    paste("drawing mesh")
    pb <- txtProgressBar(0, nrow(rawmesh$polys), style=3)
    for(i in seq(1, nrow(rawmesh$polys)))
    {
        setTxtProgressBar(pb, i)
        poly_points <- rawmesh$points[rawmesh$polys[i, seq_len(rawmesh$verts_per_poly[i])],]
        polygon(poly_points, col=poly_colour)
    }
}

draw_mesh2 <- function(rawmesh, poly_colour="lightgray")
{
    yrange <- range(pretty(rawmesh$points[,2]-1))
    xrange <- range(pretty(rawmesh$points[,1])-1)
    plot(NA, xlim=xrange, ylim=yrange, main=rawmesh$filename, yaxt="n", xaxt="n", xlab="", ylab="")

    axis(1, at=pretty(xrange), tick=TRUE, labels=TRUE)
    axis(2, at=pretty(yrange), tick=TRUE, labels=TRUE)

    print("computing poly lines")
    poly_lines <- mesh2lines(rawmesh)
    poly_lines <- poly_lines[unique(sort(poly_lines)), ]
    print("drawing poly lines")
    pb <- txtProgressBar(0, nrow(poly_lines), style=3)
    for(i in seq(1, nrow(poly_lines)))
    {
        setTxtProgressBar(pb, i)
        poly_points <- rawmesh$points[poly_lines[i,],]
        lines(poly_points, col=poly_colour)
    }
}

mesh2lines <- function(rawmesh)
{
    poly_lines <- matrix(NA, nrow=nrow(rawmesh$polys)*4, ncol=2)
    for(i in seq(1, nrow(rawmesh$polys)))
    {
        poly_points <- rawmesh$polys[i, seq_len(rawmesh$verts_per_poly[i])]
        for(j in seq(1, length(poly_points)-1))
        {
            poly_lines[i+(j-1), ] <- sort(poly_points[c(j, j+1)])
        }
        poly_lines[i, ] <- sort(poly_points[c(1, length(poly_points))])
    }
    poly_lines
}

# load 
load_trace <- function(filename)
{
    trace <- read.csv(filename, sep=";", header=FALSE, col.names=c("root", "left", "right", "priority"))
    trace$expanded <- grepl("popped off", trace$root)
    trace$start <- grepl("generating", trace$root)
    trace$root <- gsub("[\tA-Za-z: =()]+", "", trace$root)
    trace$left <- gsub("[\tA-Za-z: =()]+", "", trace$left)
    trace$right <- gsub("[\tA-Za-z: =()]+", "", trace$right)

    repeating_node <- which(trace$start)+1
    trace <- trace[-repeating_node,]
    trace$expanded <- trace$expanded | trace$start
    trace
}

# visualses polyanya search instances from a trace file
# used in combination with load_trace
# e.g. draw_trace(load_trace(search_output_file))
draw_trace <- function(search_trace)
{
    begin <- which(search_trace$start)
    end <- c(begin[-1]-1, nrow(search_trace))
    for(i in seq(1, length(begin)))
    {

        if(i > 1)
        {
            print(paste(begin[i-1], end[i-1]))
            draw_expansion(search_trace[begin[i-1] : end[i-1], ], TRUE)
        }
        print(paste(begin[i], end[i]))
        draw_expansion(search_trace[begin[i] : end[i], ], FALSE)
    }
}

draw_expansion <- function(exp_trace, prev=FALSE)
{
    # start node colour and symbol
    s_col = "orange"
    s_pch = 15

    # parent node colour and symbol
    p_col = "red"
    p_pch = 20

    # current successor node colour and symbol
    c_col = "green"
    c_pch = 20

    begin <- which(exp_trace$expanded)[1]
    current_root <- c()
    for(i in seq(begin, nrow(exp_trace)))
    {
        print(exp_trace[i,])
        root_xy <- as.numeric(unlist(strsplit(exp_trace$root[i], ",")))
        left_xy <- as.numeric(unlist(strsplit(exp_trace$left[i], ",")))
        right_xy <- as.numeric(unlist(strsplit(exp_trace$right[i], ",")))

        if(exp_trace$start[i])
        {
            mycol <- s_col
            mypch <- s_pch
        }
        else if(exp_trace$expanded[i])
        {
            mycol <- p_col
            mypch <- p_pch
        }
        else
        {
            mypch <- c_pch
            mycol <- c_col;
        }
        if(prev)
        { 
            mycol <- "black"
        }
        if(exp_trace$expanded[i] || exp_trace$start[i])
        {
            current_root <- root_xy
            points(root_xy[1], root_xy[2], pch=mypch, col=mycol, bg=mycol)
        }

        if(!all(current_root == root_xy))
        {
            points(root_xy[1], root_xy[2], pch=mypch, col=mycol, bg=mycol)
        }
        lines(c(left_xy[1], right_xy[1]), c(left_xy[2], right_xy[2]) , col=mycol)

        if(!prev)
        {
            readline(prompt="Press [enter] to continue")
        }
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



