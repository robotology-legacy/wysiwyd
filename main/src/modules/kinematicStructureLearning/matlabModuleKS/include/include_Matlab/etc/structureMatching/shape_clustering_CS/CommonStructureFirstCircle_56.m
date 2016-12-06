function [ PointSetJoints, JuncSetJoints ] = CommonStructureFirstCircle_56( Row, Col, PointSet, RowHOld, nsetCostsLen,JuncSet,Save_C)
datapath = fullfile('56/');
datafilename = dir(fullfile('56/*.pgm'));
datanum = length(datafilename);
sample_num = 50;
alpha = 100;
% RowHOld  = RowH;
PointSetRow = PointSet{Row};
PointSetii = PointSet{Col};
JuncSetRow = JuncSet{Row};
JuncSetii = JuncSet{Col};
i = Col;
show=0;
niLength = length( RowHOld{ i } );
nRowLength = length( RowHOld{ Row } );
%         niLength = 1;
%         nRowLength = 1;
%         nColLength = 1;
Cost1 = 0;
Cost2 = 0;
PointSetJoint = ClusterEP( PointSetRow, PointSetii, Save_C, RowHOld, Row, i );
PointSetJoints= PointSetJoint;


for n1 = 1: size( JuncSetRow, 1 )
    %the clustering number of junc in Row
    for n2 = 1: size( JuncSetii, 1 )
        %the clustering number of junc in ii
        diff1 = 0;
        diff2 = 0;
        for n3 = 1: length( PointSetJoint )
            k = 0;
            M = 0;
            countRow = 0;
            for n4 = 1: nRowLength
                if PointSetJoint{ n3 }( n4 ) == 0
                    countRow = countRow +1;
                end
                jj = n4;
                JPtoEPdfjj = [datapath datafilename(RowHOld{ Row }( jj )).name '_JPtoEP.mat'];
                load(JPtoEPdfjj);
                JPtoEPjj = JPtoEP;
                MMdfjj = [datapath datafilename(RowHOld{ Row }( jj )).name '_MM.mat'];
                load(MMdfjj);
                MMjj = MM;
                countii=0;
                for n5 = 1: niLength
                    if PointSetJoint{ n3 }( length(PointSetRow{1})+n5 ) ==0
                        countii = countii +1;
                    end
                    ii = n5;
                    JPtoEPdfii = [datapath datafilename(RowHOld{ i }( ii )).name '_JPtoEP.mat'];
                    load(JPtoEPdfii);
                    JPtoEPii = JPtoEP;
                    MMdfii = [datapath datafilename(RowHOld{ i }( ii )).name '_MM.mat'];
                    load(MMdfii);
                    MMii = MM;
                    %                             m = 0;
                    for n6 = 1: length( JuncSetRow{ n1, n4 } )
                        for n7 = 1: length( JuncSetii{ n2, n5 } )
                            if PointSetJoint{ n3 }( n4 )~=0&&PointSetJoint{ n3 }( length(PointSetRow{1})+n5 )~=0
                                k = k + 1;
                                M( k ) = sum((JPtoEPjj{JuncSetRow{n1,n4}(n6)}(1:end-1,PointSetJoint{n3}(n4))-JPtoEPii{JuncSetii{n2,n5}(n7)}(1:end-1,PointSetJoint{n3}(length(PointSetRow{1})+n5))).^2./abs((JPtoEPjj{JuncSetRow{n1,n4}(n6)}(1:end-1,PointSetJoint{n3}(n4)))+JPtoEPii{JuncSetii{n2,n5}(n7)}(1:end-1,PointSetJoint{n3}(length(PointSetRow{1})+n5)))) +...
                                    alpha*(JPtoEPjj{JuncSetRow{n1,n4}(n6)}(end,PointSetJoint{n3}(n4))-JPtoEPii{JuncSetii{n2,n5}(n7)}(end,PointSetJoint{n3}(length(PointSetRow{1})+n5)))^2/abs(JPtoEPjj{JuncSetRow{n1,n4}(n6)}(end,PointSetJoint{n3}(n4))+JPtoEPii{JuncSetii{n2,n5}(n7)}(end,PointSetJoint{n3}(length(PointSetRow{1})+n5)));
                                %                                            SaveNum{k} = [n1,n2; n4, n5; JuncSetRow{n1,n4}(n6), JuncSetii{n2,n5}(n7); PointSetRow{n3}(n4), EPii(n4,n5)];
                            end
                        end
                    end
                end
            end

            %             if M(1) == 0 && countRow(1) == nRowLength(1)
            %                 diff1 = diff1+length( PointSetii{n3}(find(PointSetii{n3}~=0)))/length( PointSetii{n3});
            %             elseif M(1) == 0 && countii(1) == niLength(1)
            %                 diff2 = diff2 +length( PointSetRow{n3}(find(PointSetRow{n3}~=0)))/length( PointSetRow{n3});
            %             elseif M~=0
            if M(1) == 0 && countRow(1) == nRowLength(1)
                Joint1 = PointSetJoint{n3}(nRowLength+1:nRowLength+niLength(1));
                diff1 = diff1+length( Joint1(find(Joint1~=0)))/niLength(1);
            elseif M(1) == 0 && countii(1) == niLength(1)
                Joint2= PointSetJoint{n3}(1:nRowLength(1));
                diff2 = diff2 +length( Joint2(find(Joint2~=0)))/nRowLength(1);
            elseif M~=0
                %                         [ MinMk, MinNum ] = min( M(find(M~=0)) );%find
                %                         the minimum end point and junc
                %                         Ma{ n1, n2 }( n3 ) = M( MinNum );
                Ma{n1, n2}(n3) = mean( M(find(M~=0)) );
            end
            %                 Ma( n3 ) = M( MinNum );
        end%n3 the end point of shapes in Row
        jumpcost=mean( Ma{ n1, n2 } )+std( Ma{ n1, n2 } );
        %                 punish = jumpcost*2.5;
        numerRow = size( JuncSetRow, 2 );
        clear a;
        clear aa;
        for rowNum = 1: size( JuncSetRow, 2 )
            a( rowNum ) = isempty( JuncSetRow{ n1, rowNum } );
        end
        NumZero1 = sum(a);
        denoRow = size( JuncSetRow,2 ) - NumZero1;%%%%
        numerii = size( JuncSetii, 2 );
        for iiNum = 1: size( JuncSetii, 2 )
            aa( iiNum ) = isempty( JuncSetii{ n2, iiNum } );
        end
        NumZero2 = sum(aa);
        denoii = size( JuncSetii,2 ) - NumZero2;
        %                 sumWi = denoRow/numerRow*diff1+denoii/numerii*diff2;
        sumWi = diff1+diff2;
        %                 sumWi = denoRow/numerRow +
        %                 denoii/numerii;
        punish = jumpcost * ( sumWi )*2.5;
        Dis_UJ( n1, n2 ) = (sum( Ma{ n1, n2 } ) + punish)/length(Ma{n1,n2})*( 1+abs(denoRow/numerRow - denoii/numerii)/(denoRow/numerRow + denoii/numerii ));
    end
end
[ Corres_UJ, Cost_UJ ] = Hungarian( Dis_UJ );
[ UJRow, UJii ] = find( Corres_UJ == 1 );
nUJLength = length( UJRow );
if  size(JuncSetRow,1)==size(JuncSetii,1)
    flag = 0;
    JuncSetJoint = cell( nUJLength, size( JuncSetRow, 2 )+size( JuncSetii, 2 ) );
elseif size(JuncSetRow,1)>size(JuncSetii,1)
    flag = 1;
    JuncSetJoint = cell( size(JuncSetRow,1), size( JuncSetRow, 2 )+size( JuncSetii, 2 ) );
else
    flag = 2;
    JuncSetJoint = cell( size(JuncSetii,1), size( JuncSetRow, 2 )+size( JuncSetii, 2 ) );
end
%         JuncSetJoint = cell( nUJLength, size( JuncSetRow, 2 )+size( JuncSetii, 2 ) );
for nUJ = 1: nUJLength
    %     JuncSetJoin = { JuncSetRow{ :, UJRow( nUJ ) }, JuncSetii{ :, UJii( nUJ ) } }';
    JuncSetJoin = { JuncSetRow{ UJRow( nUJ ), : }, JuncSetii{ UJii( nUJ ), : } };
    JuncSetJoint( nUJ,: ) = JuncSetJoin;
end
if flag == 1
    %             JuncSetiiZeros=cell(1, size(JuncSetii, 2));
    kk=0;
    for nUJ = 1: size(JuncSetRow,1)
        if isempty( find( UJRow == nUJ ) )
            kk = kk+1;
            JuncSetJoint( nUJLength+kk, 1: size(JuncSetRow,2) ) = { JuncSetRow{( nUJ ),:} };
        end
    end
end
if flag ==2
    %             JuncSetRowZeros{1}=zeros(1, size(JuncSetRow,2));
    kk=0;
    for nUJ = 1: size(JuncSetii,1)
        if isempty( find( UJii == nUJ ) )
            kk = kk+1;
            JuncSetJoint( nUJLength+kk,size(JuncSetRow,2)+1: end ) = { JuncSetii{( nUJ ),:} };
        end
    end
end
JuncSetJoints= JuncSetJoint;
if(show)
    %                 [cc1,cc2] = find(Match ==1);
    %                 cc1 = [ ClassMMi{ Row }( cc1 ) ];
    %                 cc2 = [ ClassMMj{ Col }( cc2 ) ];
    cc1 = [2,1];
    cc2 = [1,3];
    c1= [1,2,3,4,5];
    c2 = [1,2,3,4,5];
    %                 load( [datapath num2str(RowHOld{ Row }( jj )) '.jpg_IM.mat']);
    load([datapath datafilename(RowHOld{ Row }( jj )).name '_IM.mat']);
    BWi = bwmorph(IM,'remove');
    %                 load([datapath num2str(RowHOld{ i }( ii )) '.jpg_IM.mat']);
    load([datapath datafilename(RowHOld{ i }( ii )).name '_IM.mat']);
    BWj = bwmorph(IM,'remove');
    %                 load([datapath num2str(RowHOld{ Row }( jj )) '.jpg_SK.mat']);
    load([datapath datafilename(RowHOld{ Row }( jj )).name '_SK.mat']);
    SKi = SK;
    SKi = SKMorph( SKi );
    %                 load([datapath num2str(RowHOld{ i }( ii )) '.jpg_SK.mat']);
    load([datapath datafilename(RowHOld{ i }( ii )).name '_SK.mat']);
    SKj = SK;
    SKj = SKMorph( SKj );
    %                 load([datapath num2str(RowHOld{ Row }( jj )) '.jpg_EP.mat']);
    load([datapath datafilename(RowHOld{ Row }( jj )).name '_EP.mat']);
    EPi = EP;
    %                 load([datapath num2str(RowHOld{ i }( ii )) '.jpg_EP.mat']);
    load([datapath datafilename(RowHOld{ i }( ii )).name '_EP.mat']);
    EPj = EP;
    load([datapath datafilename(RowHOld{Row}(jj)).name '_JP.mat']);
    %                 load([datapath num2str(RowHOld{Row}(jj)) '.jpg_JP.mat']);
    JPi = JP;
    load([datapath datafilename(RowHOld{i}(ii)).name '_JP.mat']);
    %                 load([datapath num2str(RowHOld{i}(ii)) '.jpg_JP.mat']);
    JPj = JP;
    subplot( 2,1,1 );
    imshow(~(SKi+BWi));
    hold on
    for kk = 1: length( cc1 )
        text( JPi( ( cc1( kk ) ), 2 ), JPi( ( cc1( kk ) ), 1 ), num2str( kk ),'BackgroundColor',[ 0 1 0 ] );
    end
    for kk = 1:length(c1)
        text(EPi((c1(kk)),2), EPi((c1(kk)),1), num2str(kk),'BackgroundColor',[1 0 0]);
    end
    subplot( 2,1,2 );
    imshow( ~( SKj+BWj ) );
    hold on
    for kk = 1:length( cc2 )
        text( JPj( ( cc2( kk ) ), 2 ), JPj( ( cc2( kk ) ), 1 ), num2str( kk ),'BackgroundColor',[ 0 1 0 ] );
    end
    for kk = 1:length(c2)
        text(EPj((c2(kk)),2), EPj((c2(kk)),1), num2str(kk),'BackgroundColor',[1 0 0]);
    end
    %                 Match
    close
end