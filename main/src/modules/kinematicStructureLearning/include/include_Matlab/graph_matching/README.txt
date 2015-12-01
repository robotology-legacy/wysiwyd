============================================
C-Matlab Code for "A Flexible Block Coordinate Ascent Scheme for Hypergraph Matching", CVPR 2015

This code is free for research purposes.
Please cite our paper 
    @inproceedings{Quynh15,
       author       = {Quynh Nguyen and Antoine Gautier and Matthias Hein},
       title        = {A Flexible Tensor Block Coordinate Ascent Scheme for Hypergraph Matching},
       booktitle    = {IEEE Conference on Computer Vision and Pattern Recognition (CVPR)},
       year         = {2015},
    }
if you find this code useful for your research. 
If there is any problem using the code, contact Quynh Nguyen: quynh@cs.uni-saarland.de

written by Quynh Nguyen, 2014, Saarland University, Germany
http://www.ml.uni-saarland.de/people/nguyen.htm 

Version: 1.0
============================================
1. INSTALLATION
        - run 'compile' to compile all C-MEX files

============================================
2. DOCUMENTATION
a) To valuate matching algorithms on the synthetic dataset 
        - test_synthetic.m      : there are different experimental settings corresponding to two main tests as described in the paper
        (deformation test and outlier test). Each setting is stored in a separate script file whose name starts with 'setSettings_*.m'.
        All the setting files can be found in 'extract_figures.m'. 
        - example               : the following the command
                    test_synthetic(1, 'def_20inlier', 1)
        will run all algorithms on the setting stored in 'setSettings_def_20inlier.m' and plot the results.
        More details on how to use this function can be found in the function script.

b) To evaluate matching algorithms on the CMU house dataset
        - test_house.m          : test algorithms with a specific number of points in the first image passed as a parameter to the function.
        - example               : the following command
                    test_house(1, 10, 1)
        will test algorithms on the "10pts vs 30 pts" matching tasks.

c) To show demo matching for a random problem in the synthetic dataset
        - show_matching.m       : refer to the function script for usage.

============================================
3. REFERENCES
***) This package includes the implementation of our Block Coordinate Ascent Graph Matching (BCAGM) algorithms and the author's implementation of other papers

M. Cho, J. Lee, and K. M. Lee. "Reweighted random walks for graph matching". ECCV 2010.
http://cv.snu.ac.kr/research/~RRWM/

J. Lee, M. Cho, and K. M. Lee. "Hyper-graph matching via reweighted random walks". CVPR 2011. 
http://cv.snu.ac.kr/research/~RRWHM/

M. Cho, J. Sun, O. Duchenne, and J. Ponce. "Finding matches in a haystack: A max-pooling strategy for graph matching in the presence of outliers", CVPR 2014.
http://www.di.ens.fr/willow/research/maxpoolingmatching/

O. Duchenne, F. Bach, I. Kweon, and J. Ponce. "A tensor-based algorithm for high-order graph matching". CVPR 2009. 
http://www.cs.cmu.edu/~olivierd/

M. Leordeanu, M. Hebert, and R. Sukthankar. "An integer projected fixed point method for graph matching and map inference". NIPS 2009.
https://sites.google.com/site/graphmatchingmethods/

M. Leordeanu and M. Hebert. "A spectral technique for correspondence problems using pairwise constraints". ICCV 2005. 
https://sites.google.com/site/graphmatchingmethods/

R. Zass and A. Shashua. "Probabilistic graph and hypergraph matching". CVPR 2008. 
http://www.cs.huji.ac.il/~zass/gm

*** We also utilized some scripts for particular purposes from the following public code:

(for all the scripts related to experiment setup in the synthetic dataset)
http://cv.snu.ac.kr/research/~RRWHM/

(for tensor construction from two point sets)
http://www.cs.cmu.edu/~olivierd/

Datasets are taken from the following sites:
(for the Car and Motorbike dataset)
https://sites.google.com/site/graphmatchingmethods/
This dataset has been introduced by Marius Leordeanu, Rahul Sukthankar and Martial Hebert in "Unsupervised Learning for Graph Matching", IJCV 2011 and later used by other work for testing graph matching.

(for the CMU House dataset)
http://vasc.ri.cmu.edu/idb/html/motion/house/

We appreciate all the authors for their generosity in sharing code and dataset.

