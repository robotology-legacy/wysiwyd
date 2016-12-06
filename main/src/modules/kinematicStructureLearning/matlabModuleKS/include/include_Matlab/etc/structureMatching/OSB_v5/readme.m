
1. The directory FullSequences
      a. OSBv5.m - the original MatLab function (also contains the MatLab subfunction findpathDAG)
      b. OSBv5_C.m - the MatLab function without the subfunction findpathDAG. This version is to be used with findpathDAG.c
      c. findpathDAG.c - the C version of findpathDAG. Is MUCH faster than the MatLab version

2. The directory PartialSequences
      a. OSBv5_PartialSeq.m - as explained above, the all MatLab version, but for partial sequences
      b. OSBv5_PartialSeq_C.m - without the subfunction findpathDAG, instead calls the C version of findpathDAG
      c. findpathDAG.c - the C version for partial sequences
