clear;
clc
load (['56/save56_5.mat']);
datapath = fullfile('56/');
datafilename = dir(fullfile('56/*.pgm'));
datanum = length(datafilename);
sample_num = 50;


setCostsFirst = setCosts;
nsetCostsLen = length( setCosts );

for i =1: nsetCostsLen
    RowH{ i } = i;
end
num_i = 0;
while(1)
    num_i = num_i+1
    b=0;
    c=0;
    RowHOld = RowH;
    nsetCostsLen = length( setCosts );
    minCost = min( setCosts( find( setCosts~=0 ) ) );
    [ Coll, Roww ] = find( setCosts == minCost );
    Coll = Coll(1);
    Roww = Roww(1);
    Row = min(Coll, Roww);
    Col = max( Coll, Roww );
    RowH{ Row } = [ RowH{ Row }, RowH{ Col } ];
    RowHNew = { RowH{1:Col-1}, RowH{ Col+1: length( RowH ) } };
    RowH = RowHNew;
    setCostNew = setCosts;
    for u  =1:length(RowHNew)
        if length(RowHNew{u})~=1
            fprintf(1, '[');
            fprintf(1, '%d ', RowHNew{u});
            fprintf(1, ']  ');
        else
            fprintf(1, '%d  ', RowHNew{u});
        end
    end
    fprintf(1, '\n');
    [ PointSetJoints, JuncSetJoints ] = CommonStructureFirstCircle_56( Row, Col, PointSet, RowHOld, nsetCostsLen,JuncSet,Save_C);
    PointSet{Row} = PointSetJoints;
    End_PointSet = length( PointSet );
    JuncSet{Row} = JuncSetJoints;
    End_JuncSet = length( JuncSet );
    PointSet = { PointSet{ 1: Col-1 }, PointSet{ Col+1: End_PointSet } };
    JuncSet = { JuncSet{ 1: Col-1 }, JuncSet{ Col+1: End_JuncSet } };
    PointSetRow = PointSet{ Row };
    JuncSetRow = JuncSet{ Row };
    %     end
    [ PointSetJoints, JuncSetJoints, setCostNew ] = CommonStructure_56( Row, Col, PointSetRow, JuncSetRow, PointSet, JuncSet, RowHOld,RowHNew,setCostNew,Save_C );
    b = tril( setCostNew, -1 );
    c = tril( setCostNew', -1 );
    setCostNew = b+c';
    setCosts = setCostNew;
    if length( setCostNew ) == 14
        break;
    end
end
Shape = setCosts;