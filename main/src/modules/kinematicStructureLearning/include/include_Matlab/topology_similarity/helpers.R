graphRemoveVert <- function(graph, vert2rem){

	#cat("[graphRemoveVert] Try to remove vertice ", V(graph)$name[vert2rem], "\n")

	#protection : vert2rem should be >1 and < length vertex graph
	if( (vert2rem < 1) || (vert2rem > length(V(graph)))){
		return ("ERROR : vert2rem is not compatible")
	}

	newGraph <- graph.empty(n=0, directed=FALSE)	
	oldGraph <- graph

	#check edges from/to the vertices
	#cat("before neighbors\n")
	
	vertexConnected <- neighbors(graph, V(graph)$name[vert2rem])
	#print(vertexConnected)
	
	#cat("checking vertex connected\n")
	if (length(vertexConnected == 1)){
		#cat("Only one edge for Vertex ", V(graph)$name[vert2rem], " : removing it \n")
		edgeConnected <- E(graph)[from(vert2rem)]
		#cat("Edge to be removed : ", edgeConnected, "\n")
		newGraph <- delete.edges(graph,edgeConnected)
	}

	#if 2 vertex connected (A -- B -- C) : link A -- C then remove B
	if ( length(vertexConnected) == 2 ) {
		#cat("Two edged for Vertex ", V(graph)$name[vert2rem], " : connecting nodes before removing the central one \n")
		newGraph <- graph
		newGraph <- add.edges(newGraph, c(vertexConnected[1], vertexConnected[2]))
		E(newGraph)$color <- "gray"
		E(newGraph)[length(E(newGraph))]$color <- "blue"
	}

	#if more  than 2 edges : link everything on the higher connectivity node
	#or not removing it?
	else if (length(vertexConnected) > 2) {
		#cat("Vertex ", vert2rem, " is linked to more than 2 nodes : ignoring!\n")
		return(graph) 
	}

	#x11()
	par(mfrow=c(1,2))	
	#plot(newGraph)

	#cat("Deleting the vertice\n")
	newGraph <- delete.vertices(newGraph, V(newGraph)$name[vert2rem])
	
	colorVert <- rep(1, length(V(oldGraph)))
	colorVert[vert2rem] <- 2

	#cat("Before coloring oldGraph\n")
	colorVert
	#V(oldGraph)$color
	V(oldGraph)$color <- colorVert
	#V(oldGraph)$color
	#plot(oldGraph)
	#plot(newGraph)
	
	#cat("Returning the new graph\n")
	return(newGraph)
}

matrix2file <- function(table.matrix, filepath ){
	
	if(is.matrix(table.matrix)){
		write(t(table.matrix), file = filepath, sep = "\t", ncolumns = ncol(table.matrix))
	}
	#go through the matrix list
	else {
		for(i in 1:length(table.matrix)){
			currentMatrix <- table.matrix[[i]]

			#file append false the first time
			isAppend = TRUE
			if(i == 1){
				isAppend = FALSE
			}

			write(t(currentMatrix), file = filepath, sep = "\t", ncolumns = ncol(currentMatrix), append=isAppend)
			write("\n", file=filepath, append=TRUE)
		}
	}

}

normalizeMatrix <- function(table.matrix){
	
	#if it is actually just a matrix
	if(is.matrix(table.matrix)){
		table.matrix <- sweep(table.matrix, 2, colSums(currentMatrix), FUN="/")
	}
	#go through the matrix list
	else {
		for(i in 1:length(table.matrix)){

			currentMatrix <- table.matrix[[i]]
			#cat("currentMatrix = ", currentMatrix, "\n")
			#normalized if it is a matrix
			if(is.matrix(currentMatrix)){
				#cat("normalized matrix #", i, "\n")
				table.matrix[[i]] <- sweep(currentMatrix, 2, colSums(currentMatrix), FUN="/")
			} 			

		}
	}
	
	return(table.matrix)

}

addMatrix <- function(mainMatrix, addMatrix, mainCoef=1, addCoef=1){ #mainMatrix row number >= addMatrix row numbers
	#print(mainCoef)
	for(name in rownames(addMatrix)){
		mainMatrix[name,] <- (mainMatrix[name,]*mainCoef) + (addMatrix[name,]*addCoef)
 		#mainMatrix[name,] <- mainMatrix[name,] + addMatrix[name,]
	}
	
	return(mainMatrix)

}


subisoEdgeConstraint <- function(smallGraph, bigGraph, neighbors.constraint.total, neighbors.constraint.solo = 3, usePlot=TRUE){

	useDebug <- FALSE

	#Extract the neighbors for each vertices of the small graph
	smallGraph.neighbors <- c()
	for(v in 1:length(V(smallGraph))){
    	smallGraph.neighbors <- append(smallGraph.neighbors, c(length(neighbors(smallGraph, v, mode = 1))))
	}
	smallGraph.neighbors

	#prepare the correspondance matrix
	matrix.correspondance = matrix(0, nrow = length(V(bigGraph)), ncol = length(get.subG[[1]]))
	rownames(matrix.correspondance) = V(bigGraph)$name
	colnames(matrix.correspondance) = V(smallGraph)$name
	
	#remove warning : In disjoint_union(x, y): Duplicate vertex names in disjoint union
	oldw <- getOption("warn")
	options(warn = -1)

	#list of matrix : matrix #i group all subisomorphism with a total edge difference of i-1 (because the list start at 1)
	list.matrix.constraint <- list()
	for(i in 1:(neighbors.constraint.total+1)){
		#cat("matrix in the list #", i, "\n")
   		list.matrix.constraint[[i]] <- 0   
	}

	#print(list.matrix.constraint)

	#count and extract all the subisomorphisms
	count.sub <- graph.count.subisomorphisms.vf2(bigGraph,smallGraph, vertex.color1 = NULL, vertex.color2 = NULL)
	if(count.sub == 0){
		#cat("No subisomorphism available here : go out")
		return(list.matrix.constraint) ########################################### comment for debug, need to put it back after
	}

	get.subG <- graph.get.subisomorphisms.vf2(bigGraph,smallGraph, vertex.color1 = NULL, vertex.color2 = NULL)

	#go through all the possible subismorphism
	for(i in 1:count.sub){
		concat.edges <- c()
		#cat("\nCheck SubIsomorphism #", i, "\n")
		isWorthKeeping <- TRUE
		subG <- get.subG[[i]]
    
		#check the edge neighbors for the proposed subisomorphism
		subG.neighbors <- c()
		diffNeighbors.total <- 0
		for(v in 1:length(subG)){

			#compare the edge difference between smallGraph and bigGraph for the current vertice
			neighbors.curr <- length(neighbors(bigGraph, subG[v], mode = 1))
			subG.neighbors <- append(subG.neighbors, c(neighbors.curr))
			diffNeighbors  <- abs(neighbors.curr-smallGraph.neighbors[v])

			#Filter 1 : Reject subisomorphism as soon as ONE vertice has a difference of neighbors.constraint.solo (2 by default) or more
			if(diffNeighbors >= neighbors.constraint.solo){
		    		isWorthKeeping <- FALSE
				#cat("Vertice ", V(bigGraph)$name[subG[v]], " has edge difference of ", diffNeighbors, " >= ", neighbors.constraint.solo, ")\n")
		    		next #go only after the current loop, should go out after
			}

		#add the difference for the current vertice to the total
		diffNeighbors.total <- diffNeighbors.total + diffNeighbors
	    	}	

		if(isWorthKeeping == FALSE){
			next
		}

		#Filter 2 : Reject subismorphism as soon as the total edge difference is bigger than neighbors.constraint.total
		if(diffNeighbors.total > neighbors.constraint.total){
	            	#cat("Total difference in neighbors is too big (i.e. ", diffNeighbors.total, " > ", neighbors.constraint.total, ")\n")
	            	next 
		}

    		#cat("=====> subIsomorphism #", i, " used!\n")
    
		#prepare the correspondance matrix for the current subisomorphism
    		matrix.correspondance.intermediate = matrix(0, nrow = length(V(bigGraph)), ncol = length(get.subG[[1]]))
    		rownames(matrix.correspondance.intermediate) = V(bigGraph)$name
    		colnames(matrix.correspondance.intermediate) = V(smallGraph)$name

		#Calculate the correspondance matrix for the current subisomorphism (+ update color for bigGraph to highlight the matching in the plot)
		bigGraph_col <- bigGraph
		myColor <- rep(1, length(V(bigGraph_col)))
		for(j in 1:length(subG)){
			myColor[subG[j]] = 2
			#matrix.correspondance[subG[j],j] <- matrix.correspondance[subG[j],j] + 1
        		matrix.correspondance.intermediate[subG[j],j] <- matrix.correspondance.intermediate[subG[j],j] + 1
			concat.edges <- append(concat.edges, c(length(V(bigGraph_col))+j,as.integer(subG[j]))) 
		}

		#update the list of matrix with the one from the current subisomorphism : will be added to the list at the corresponding place depeding on the total edge difference
		#print(list.matrix.constraint[[(diffNeighbors.total+1)]])
		#cat("Matrix will be added to an existing one in pos #", diffNeighbors.total+1)
       		list.matrix.constraint[[(diffNeighbors.total+1)]] <- list.matrix.constraint[[(diffNeighbors.total+1)]] + matrix.correspondance.intermediate 
		#print(list.matrix.constraint)       


		#some plot
		if(usePlot){
			V(bigGraph_col)$color <- "darkorange"
			V(smallGraph)$color <- "deepskyblue3"

			#par(mfrow=c(1,2))
    
			g <- bigGraph_col %du% smallGraph
			E(g)$color <- "red"
			g <- g + edges(concat.edges, color="gray") 
			#plot(g, main = i)

			V(bigGraph_col)$color <- myColor
			#plot(bigGraph_col, mark.groups=list(subG), mark.col = "lightblue", main = "Perfect subisomorphism")

			#to print in a pdf one precise plot
			#cat("i =  #", i, "\n")
			#if(i==10){
		#		cat("WILL PRINT PLOT #", i, "\n")
			#	dev.off()
			#	x11()
			#	pdf("/home/maxime/CloudStation/R/igraph/topology/plot10.pdf")

				#plot
			#	plot(g)
			#	legend(bty = "n", legend = c("nodes from g1", "nodes from g2"), x = "bottomleft", col = c("black", "black"), pt.bg = c("deepskyblue3", "darkorange"), pch = c(21, 21), pt.cex = 4, y.intersp = 2, x.intersp = 1.5)

			#	dev.off()

			#}
		}
	}

	#print(list.matrix.constraint)

	options(warn = oldw) #end warning removal

	return(list.matrix.constraint) 

}

graphToLeda <- function(graph, inFilename, outFilename){ #mainMatrix row number >= addMatrix row numbers
	
	#write in Leda : but comment #vertex #edges that can mess : remove after
	write.graph(graph, file = inFilename, format=c("leda"), vertex.attr = "name")

	text.graph <- readLines(inFilename)
	file.graph <- file(outFilename)

	#go through the generated leda filename inFilename
	text.graph.substr <- ""
	for(i in 1:length(text.graph)){

		#check if the line is a comment : start with #. Not doing anything if
    		if(substr(text.graph[i],1,1) != "#"){

			#first line
        		if(text.graph.substr == ""){
            			text.graph.substr <- text.graph[i]
        		} else {
				#copy the line
            			text.graph.substr <- paste(text.graph.substr, text.graph[i], sep = "\n")            
        		}
    		}
	}

	file.graph <- writeLines(text.graph.substr, file.graph)

	return(text.graph.substr)
}
