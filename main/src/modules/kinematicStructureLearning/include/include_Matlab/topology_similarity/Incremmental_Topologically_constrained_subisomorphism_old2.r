
require(igraph)     #graph method
require(reshape2)   #print matrix
require(ggplot2)    #print matrix
source("../../include/include_Matlab/topology_similarity/helpers.R")

##should be > 0 : total edge difference between graph suisomorphism
neighbors.constraint.total <- 3

#g1 and g2, g2 included in g1
#g1 <- graph( c(1,2, 2,3, 3,4, 3,5, 5,6, 3,7, 2,5), n=7, directed=FALSE)  #to show not induce
#g2 <- graph( c(1,2, 2,3, 3,4, 3,5, 5,6), n=6, directed=FALSE)

#build graph with .txt
df1 <- read.table("../../include/include_Matlab/topology_similarity/myGraph.txt", sep='\t', dec=',', header=T)
dg1 <- subset(df1, (df1$graphType == "g1"))
g1  <- graph.data.frame(dg1, directed=F)
dg2 <- subset(df1, (df1$graphType == "g2"))
g2  <- graph.data.frame(dg2, directed=F)
dg1
dg2

#graphToLeda(g1, "topology/g1-leda-1.gw", "topology/g1.gw")
#graphToLeda(g2, "topology/g2-leda-1.gw", "topology/g2.gw")

graphToLeda(g1, "../../include/include_Matlab/topology_similarity/graph_alignment/MAGNA++_CLI_Linux64_01_30_2015/graph_P-leda-temp.gw", "../../include/include_Matlab/topology_similarity/graph_alignment/MAGNA++_CLI_Linux64_01_30_2015/graph_P.gw")
graphToLeda(g2, "../../include/include_Matlab/topology_similarity/graph_alignment/MAGNA++_CLI_Linux64_01_30_2015/graph_Q-leda-temp.gw", "../../include/include_Matlab/topology_similarity/graph_alignment/MAGNA++_CLI_Linux64_01_30_2015/graph_Q.gw")


#switch g1/g2 if g1 < g2
if(length(V(g1))<=length(V(g2))){
    cat("WARNING : switch g1/g2 because g1 was smaller than g2")
    g3 <- g1
    g1 <- g2
    g2 <- g3
}

V(g2)$color <- 2

#plot them, side by side
par(mfrow=c(1,2))
plot(g1, main = "g1 : more complex skeleton")
plot(g2, main = "g2 : skeleton used as a template")

count.sub <- graph.count.subisomorphisms.vf2(g1,g2, vertex.color1 = NULL, vertex.color2 = NULL)
get.subG <- graph.get.subisomorphisms.vf2(g1,g2, vertex.color1 = NULL, vertex.color2 = NULL)

matrix.correspondance = matrix(0, nrow = length(V(g1)), ncol = length(get.subG[[1]]))
rownames(matrix.correspondance) = V(g1)$name
colnames(matrix.correspondance) = V(g2)$name

source("../../include/include_Matlab/topology_similarity/helpers.R")
list.matrix.constraint <- subisoEdgeConstraint(g2,g1, neighbors.constraint.total)
#print(list.matrix.constraint)

print(list.matrix.constraint)
list.matrix.constraint <- normalizeMatrix(list.matrix.constraint)
print(list.matrix.constraint)

matrix.correspondance.g1constraint = matrix(0, nrow = length(V(g1)), ncol = length(get.subG[[1]]))
rownames(matrix.correspondance.g1constraint) = V(g1)$name
colnames(matrix.correspondance.g1constraint) = V(g2)$name

for(i in 1:length(list.matrix.constraint)){
    m <- list.matrix.constraint[[i]]
    #cat("matrix #", i ,":\n" )
    #print(m)
    if(is.matrix(m)){
        #cat("m is a matrix\n")
        matrix.correspondance.g1constraint <- matrix.correspondance.g1constraint + (list.matrix.constraint[[i]])/i
    }  
    #cat("matrix.normalized.g1constraint : \n")
    #print(matrix.normalized.g1constraint)
}
matrix.correspondance.g1constraint <- sweep(matrix.correspondance.g1constraint, 2, colSums(matrix.correspondance.g1constraint), FUN="/")
#cat("END : \n")
print(matrix.correspondance.g1constraint)

#assumption : all is Nan is [1,1] is NaN
if(is.nan(matrix.correspondance.g1constraint[1,1])){
    print("[WARNING] matrix.correspondance.g1constraint is all NaN : transform to uniform")
    matrix.correspondance.g1constraint = matrix((1/length(V(g1))), nrow = length(V(g1)), ncol = length(get.subG[[1]]))  
    rownames(matrix.correspondance.g1constraint) = V(g1)$name
    colnames(matrix.correspondance.g1constraint) = V(g2)$name
}
print(matrix.correspondance.g1constraint)

matrix.correspondance.g1constraint <- matrix.correspondance.g1constraint[order(strtoi(rownames(matrix.correspondance.g1constraint))),]
matrix.correspondance.g1constraint <- matrix.correspondance.g1constraint[,order(strtoi(colnames(matrix.correspondance.g1constraint)))]

print(matrix.correspondance.g1constraint)

matrix2file(matrix.correspondance.g1constraint, "../../include/include_Matlab/topology_similarity/matrix-correspondance-no-removal.output")

matrix.melt=melt(matrix.correspondance.g1constraint)
names(matrix.melt)=c("g1","g2","color")

#remove warning : Non Lab interpolation is deprecated
oldw <- getOption("warn")
options(warn = -1)

myqplot <- qplot(g2, g1, fill=color, data=matrix.melt,geom='tile', main = "Edge Constraint")
myqplot <- myqplot + scale_fill_gradient2("correspondance %", low = "blue", high = "red")

options(warn = oldw)
#print only if none default uniform matrix otherwise error because continuous color scale data with only one possible value
if(length(levels(as.factor(matrix.melt$color))) != 1){
    plot(myqplot)
}

source("../../include/include_Matlab/topology_similarity/helpers.R")

#create subgraph by removing one vertice (if edges <=2)
list.matrix.removeVertice <- list()

list.matrix.removeVertice <- list()
for(i in 1:(length(V(g1)))){
	list.matrix.removeVertice[[V(g1)$name[i]]] <- 0  
    #list.matrix.removeVertice[[i]] <- 0 
}

#print(list.matrix.removeVertice)
    
for(v in 1:length(V(g1))){      
    g1.sub <- graphRemoveVert(g1,v)
    
    list.matrix.constraint <- list()
    #print(g1.sub)
    #cat("after graphRemoveVert : length fo from ", length(V(g1)), " to ", length(V(g1.sub)), "\n")
    
    #g1.sub
    
    #if g1.sub and g1 have same length, no vertices has been removed, just do nothing
    if(length(V(g1.sub)) == length(V(g1))){
        #list.matrix[[v]] <- matrix(0, nrow = 0, ncol = 0)
        cat("===========> no change from removing vertex ", v, " (name = )", g1$name[v], "\n\n")
        list.matrix.constraint <- matrix()
        next
    }
    
    #subiso with edge constraint
    list.matrix.constraint <- subisoEdgeConstraint(g2,g1.sub, neighbors.constraint.total, usePlot=FALSE)
    list.matrix.removeVertice[[v]] <- list.matrix.constraint
}
cat("\n\n\n=====================================================================\n\n\n")
print(list.matrix.removeVertice)
cat("=====================================================================\n")

#cat("name = ", V(g1)$name[3], "\n")
#print(list.matrix.removeVertice[[V(g1)$name[3]]][2][[1]]) #list of matrix when remove vertice #3, we take the matrix 2 (i.e. with edge difference = 1), and just the matrix without the key [[1]]
#m <- list.matrix.removeVertice[[V(g1)$name[3]]][2][[1]]
#print(m)


#1 addition matrix with the same edge difference
#list.matrix.removeVertice[[V(g1)$name[3]]][2][[1]]

#First fill in with a zero : no matrix
list.matrix.g1.sub <- list()
for(i in 1:(neighbors.constraint.total+1)){
     list.matrix.g1.sub[[i]] <- 0
}

#go through all the edge removal list
for(v in 1:length(V(g1)$name)){
    
    #go through all the edge difference
    for(i in 1:length(list.matrix.removeVertice[[V(g1)$name[v]]])){
        
        #if we have a matrix
        if(is.matrix(list.matrix.removeVertice[[V(g1)$name[v]]][i][[1]])){

            #initialize also the matrix.normalized.g1.sub if was not set
            if(!is.matrix(list.matrix.g1.sub[[i]])){              
               list.matrix.g1.sub[[i]] = matrix(0, nrow = length(V(g1)), ncol = length(V(g2)))
                rownames(list.matrix.g1.sub[[i]]) = V(g1)$name
                colnames(list.matrix.g1.sub[[i]]) = V(g2)$name
                
                #order them by row name
                list.matrix.g1.sub[[i]] <- list.matrix.g1.sub[[i]][order(strtoi(rownames(list.matrix.g1.sub[[i]]))),]
                list.matrix.g1.sub[[i]] <- list.matrix.g1.sub[[i]][,order(strtoi(colnames(list.matrix.g1.sub[[i]])))]
            }
            
            #add them using the method in helpers to add row by row          
            list.matrix.g1.sub[[i]] <- addMatrix(list.matrix.g1.sub[[i]], list.matrix.removeVertice[[V(g1)$name[v]]][i][[1]])
        }
        
    }    
}

cat("============= list.matrix.g1.sub ==============\n")
print(list.matrix.g1.sub)

#normalized them
cat("\n============= normalization ==============\n")
list.matrix.g1.sub <- normalizeMatrix(list.matrix.g1.sub)
print(list.matrix.g1.sub)

#we have a list of matrix, one matrix per edge removal

#mean between matrix
matrix.correspondance.g1.sub = matrix((1/(length(V(g1)))), nrow = length(V(g1)), ncol = length(V(g2)))
rownames(matrix.correspondance.g1.sub) = V(g1)$name
colnames(matrix.correspondance.g1.sub) = V(g2)$name
matrix.correspondance.g1.sub <- matrix.correspondance.g1.sub[order(strtoi(rownames(matrix.correspondance.g1.sub))),]
matrix.correspondance.g1.sub <- matrix.correspondance.g1.sub[,order(strtoi(colnames(matrix.correspondance.g1.sub)))]


for(i in 1:length(list.matrix.g1.sub)){
    m <- list.matrix.g1.sub[[i]]
    #cat("matrix #", i ,":\n" )
    #print(m)
    if(is.matrix(m)){
        #cat("m is a matrix\n")
        matrix.correspondance.g1.sub <- matrix.correspondance.g1.sub + ((list.matrix.g1.sub[[i]])/i)
    }  
    #cat("matrix.normalized.g1constraint : \n")
    #print(matrix.normalized.g1constraint)
}
#normalized them
matrix.correspondance.g1.sub <- sweep(matrix.correspondance.g1.sub, 2, colSums(matrix.correspondance.g1.sub), FUN="/")

#assumption : all is Nan is [1,1] is NaN
if(is.nan(matrix.correspondance.g1.sub[1,1])){
    print("[WARNING] matrix.correspondance.g1.sub is all NaN : transform to uniform")
    matrix.correspondance.g1.sub = matrix((1/length(V(g1))), nrow = length(V(g1)), ncol = length(V(g2)))  
    rownames(matrix.correspondance.g1.sub) = V(g1)$name
    colnames(matrix.correspondance.g1.sub) = V(g2)$name
}

cat("\n============= matrix.correspondance.g1.sub ==============\n")
print(matrix.correspondance.g1.sub)

if(length(V(g1)) == length(V(g2))){
    cat("WARNING : matrix made from the removal is set to 0 because g1/g2 have same length\n\n")
    matrix.correspondance.g1.sub = matrix(0, nrow = length(V(g1)), ncol = length(V(g2)))
    rownames(matrix.correspondance.g1.sub) = V(g1)$name
    colnames(matrix.correspondance.g1.sub) = V(g2)$name
    matrix.correspondance.g1.sub <- matrix.correspondance.g1.sub[order(strtoi(rownames(matrix.correspondance.g1.sub))),]
    matrix.correspondance.g1.sub <- matrix.correspondance.g1.sub[,order(strtoi(colnames(matrix.correspondance.g1.sub)))]

}

print(matrix.correspondance.g1.sub)

matrix.correspondance.g1constraint
matrix.correspondance.g1.sub

matrix.correspondance.normalized <- (matrix.correspondance.g1constraint+matrix.correspondance.g1.sub)/2.0
matrix.correspondance.normalized

matrix2file(matrix.correspondance.normalized, "../../include/include_Matlab/topology_similarity/matrix-correspondance-with-removal.output")


matrix.melt=melt(matrix.correspondance.normalized)

names(matrix.melt)=c("g1","g2","color")

#remove warning : Non Lab interpolation is deprecated
oldw <- getOption("warn")
options(warn = -1)

myqplot <- qplot(g2, g1, fill=color, data=matrix.melt,geom='tile')
myqplot <- myqplot + scale_fill_gradient2("correspondance %", low = "blue", high = "red")

options(warn = oldw)

if(length(levels(as.factor(matrix.melt$color))) != 1){
    plot(myqplot)
}
