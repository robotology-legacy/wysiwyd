cd utils
    mex mexComputeFeature.cpp
    mex mexGetMatchScore.cpp
    mex assignmentoptimal.cpp
    
    cd ann_mwrapper
        ann_compile_mex
    cd ..
    
cd ..

cd methods

    cd TensorMatching
        mex mexTensorMatching.cpp
    cd ..
    
    cd RRWHM
        mex mexRRWHM.cpp
    cd ..
    
    cd 'BCAGM/cpp';
        mex -largeArrayDims hungarian_Q.cpp;
        mex -largeArrayDims mexBCAGMMatching.cpp;
        mex -largeArrayDims mexBCAGM_QUADMatching.cpp;
    cd ..
    cd ..
    
    % second order methods
    cd RRWM
        mex mexBistocNormalize_match_slack.cpp 
    cd ..
    cd MPM
        mex -largeArrayDims RMP_mult.cpp 
    cd ..
    
cd ..

