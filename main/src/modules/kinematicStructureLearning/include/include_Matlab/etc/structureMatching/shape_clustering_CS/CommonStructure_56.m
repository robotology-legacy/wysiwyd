function [ PointSetJoints, JuncSetJoints,setCostNew ] = CommonStructure_56( Row, Col, PointSetRow, JuncSetRow, PointSet, JuncSet, RowHOld, RowHNew, setCostNew, Save_C )
datapath = fullfile('56/');
datafilename = dir(fullfile('56/*.pgm'));
datanum = length(datafilename);
sample_num = 50;
alpha = 100;
show=0;
% RowHOld = RowHNew;
setCostNew( Col, : ) = [];
setCostNew( :, Col ) = [];
nsetCostsLen = length(RowHNew);
for i = 1: nsetCostsLen
    if i~=Row
        %     if i(1)~=Row&&i(1)~=Col
        PointSetii = PointSet{ i };
        JuncSetii = JuncSet{ i };
        niLength = length( RowHNew{ i } );
        nRowLength = length( RowHNew{ Row } );
        %         nColLength = length( RowHOld{ Col } );
        Cost1 = 0;
        Cost2 = 0;

        PointSetJoint = ClusterEP( PointSetRow, PointSetii, Save_C, RowHNew, Row, i );
        PointSetJoints{ i } = PointSetJoint;
        clear Ma;
        clear Dis_UJ;
        for n1 = 1: size( JuncSetRow, 1 )
            %the clustering number of junc in Row
            for n2 = 1: size( JuncSetii, 1 )
                %the clustering number of junc in ii
                diff1 =0;
                diff2 = 0;
                for n3 = 1: length( PointSetJoint )
                    %                     for n3 = 5
                    k = 0;
                    M=0;
                    countRow = 0;
                    for n4 = 1: nRowLength
                        if PointSetJoint{ n3 }( n4 ) == 0
                            countRow = countRow +1;
                        end
                        jj = n4;
                        JPtoEPdfjj = [datapath datafilename(RowHNew{ Row }( jj )).name '_JPtoEP.mat'];
                        load(JPtoEPdfjj);
                        JPtoEPjj = JPtoEP;
                        MMdfjj = [datapath datafilename(RowHNew{ Row }( jj )).name '_MM.mat'];
                        load(MMdfjj);
                        MMjj = MM;
                        countii=0;
                        for n5 = 1: niLength

                            if PointSetJoint{ n3 }( length(PointSetRow{1})+n5 ) ==0
                                countii = countii +1;
                            end
                            ii = n5;
                            JPtoEPdfii = [datapath datafilename(RowHNew{ i }( ii )).name '_JPtoEP.mat'];
                            load(JPtoEPdfii);
                            JPtoEPii = JPtoEP;
                            MMdfii = [datapath datafilename(RowHNew{ i }( ii )).name '_MM.mat'];
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
                    %                             end
                    %#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                    if M(1) == 0 && countRow(1) == nRowLength(1)
                        Joint1 = PointSetJoint{n3}(nRowLength(1)+1:nRowLength(1)+niLength(1));
                        diff1 = diff1+length( Joint1(find(Joint1~=0)))/niLength(1);
                    elseif M(1) == 0 && countii(1) == niLength(1)
                        Joint2= PointSetJoint{n3}(1:nRowLength(1));
                        diff2 = diff2 +length( Joint2(find(Joint2~=0)))/nRowLength(1);
                    elseif M~=0
                        %                         [ MinMk, MinNum ] = min( M(find(M~=0)) );%find the minimum end point and junc
                        %                     SaveNuma{ n1, n2 } = [ SaveNuma, SaveNum{ MinNum } ];%n3 = size(SaveNuma{ n1, n2 },2)/2
                        %                 SaveNuma{ n3 } = SaveNum{ MinNum };
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
                %             end
                %         end
                %                 Dis_UJ( n1, n2 ) = (sum( Ma{ n1, n2 } ) + diff*punish)/length(Ma{n1,n2});
            end
        end

        [ Corres_UJ, Cost_UJ ] = Hungarian( Dis_UJ );
        [ UJRow, UJii ] = find( Corres_UJ == 1 );
        Cost_UJ = Cost_UJ / length( UJRow );
        %         Cost_UJs( 1, i ) = Cost_UJ;

        setCostNew( Row, i ) = Cost_UJ;
        setCostNew( i, Row ) = Cost_UJ;

        nUJLength = length( UJRow );
        %         JuncSetJoint = cell( nUJLength, size( JuncSetRow, 2 )+size( JuncSetii, 2 ) );
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
        JuncSetJoints{ i } = JuncSetJoint;



        if (show)
            for jj = 1:nRowLength
                for ii = 1:niLength
                    for nc1 = 1:length(PointSetJoint)
                        if PointSetJoint{nc1}(jj) == 0
                            c1(nc1) = 1;
                        else
                            c1(nc1) = PointSetJoint{nc1}(jj);
                        end
                        if PointSetJoint{nc1}(nRowLength+ii)==0
                            c2(nc1)=1;
                        else
                            c2(nc1) = PointSetJoint{nc1}(nRowLength+ii);
                        end
                    end
                    for ncc1 = 1:size(JuncSetJoint,1)
                        if isempty(JuncSetJoint{ncc1,jj})==1
                            cc1(ncc1) = 1;
                        else
                            cc1(ncc1) = JuncSetJoint{ncc1,jj}(1);
                        end
                        if isempty(JuncSetJoint{ncc1, nRowLength+ii})==1
                            cc2(ncc1) = 1;
                        else
                            cc2(ncc1) = JuncSetJoint{ncc1,nRowLength+ii}(1);
                        end
                    end
                    load( [datapath num2str(RowHNew{ Row }( jj )) '.mat_IM.mat']);
                    %                 load([datapath datafilename(i).name '_IM.mat']);
                    BWj = bwmorph(IM,'remove');
                    load([datapath num2str(RowHNew{ i }( ii )) '.mat_IM.mat']);
                    %                 load([datapath datafilename(j).name '_IM.mat']);
                    BWi = bwmorph(IM,'remove');
                    load([datapath num2str(RowHNew{ Row }( jj )) '.mat_SK.mat']);
                    %                 load([datapath datafilename(i).name '_SK.mat']);
                    SKj = SK;
                    SKj = SKMorph( SKj );
                    load([datapath num2str(RowHNew{ i }( ii )) '.mat_SK.mat']);
                    %                 load([datapath datafilename(j).name '_SK.mat']);
                    SKi = SK;
                    SKi = SKMorph( SKi );
                    load([datapath num2str(RowHNew{ Row }( jj )) '.mat_EP.mat']);
                    %                 load([datapath datafilename(i).name '_EP.mat']);
                    EPj = EP;
                    load([datapath num2str(RowHNew{ i }( ii )) '.mat_EP.mat']);
                    %                 load([datapath datafilename(j).name '_EP.mat']);
                    EPi = EP;
                    load([datapath num2str(RowHNew{Row}(jj)) '.mat_JP.mat']);
                    JPj = JP;
                    load([datapath num2str(RowHNew{i}(ii)) '.mat_JP.mat']);
                    JPi = JP;
                    subplot( 2,1,1 );
                    imshow(~(SKj+BWj));
                    hold on
                    for kk = 1: length( cc1 )
                        text( JPj( ( cc1( kk ) ), 2 ), JPj( ( cc1( kk ) ), 1 ), num2str( kk ),'BackgroundColor',[ 0 1 0 ] );
                    end
                    for kk = 1:length(c1)
                        text(EPj((c1(kk)),2), EPj((c1(kk)),1), num2str(kk),'BackgroundColor',[1 0 0]);
                    end
                    subplot( 2,1,2 );
                    imshow( ~( SKi+BWi ) );
                    hold on
                    for kk = 1:length( cc2 )
                        text( JPi( ( cc2( kk ) ), 2 ), JPi( ( cc2( kk ) ), 1 ), num2str( kk ),'BackgroundColor',[ 0 1 0 ] );
                    end
                    for kk = 1:length(c2)
                        text(EPi((c2(kk)),2), EPi((c2(kk)),1), num2str(kk),'BackgroundColor',[1 0 0]);
                    end
                    %                 Match
                    close
                end
            end
        end
    end
end
% correspondence between end points
%according to the UJRow and UJii, we have SaveNuma{ UJRow( 1 ), UJii( 1 )
%}, SaveNuma{ UJRow( 2 ), UJii( 2 ) }...
% for nn1 = 1: nUJLength
%     for nn2 = 0.5 : size( SaveNuma{ 1, 1 }, 2 ) / 2
%         if mod( nn2, 1) == 0
%             SaveNuma{ UJRow( nn1 ), UJii( nn2 ) }(  )
% flag = 0;
% if length( PointSetRow ) >= length( PointSetii )
%     PointSetJoint = cell( 1, length( PointSetRow ) );
% else
%     PointSetJoint = cell( 1, length( PointSetii ) );
%     flag =1;
% end
% for nn1 = 1 : size( SaveNuma{ 1, 1 }, 2 )
%     nn2 = nn1 / 2;
%     if mod( nn2, 1 ) == 0;
%         find( PointSetRow{ nn1-1 } == SaveNuma{ 1,1 }( 2, nn1-1 ) );
%         for nn3 = 1: length( PointSetii )
%             if PointSetii{ nn3 }( SaveNuma{ 1, 1 }( 2, nn1 ) ) == SaveNuma{ 1, 1 }( 4, nn1 );
%                 NumPointSetii( nn2 ) = nn3;
%                 break;
%             end
%         end
%         PointSetJoint( nn1-1 ) = { PointSetRow{ nn1-1 }, PointSetii{ NumPointSetii( nn2 ) } };
%     end
% end
% kk = nn1-1;
% if flag == 1
%     zeroPointSetRow = zeros( 1, length( PointSetRow{ 1 } ) );
%     for nn4 = 1: length( PointSetii )
%         if isempty( find( NumPointSetii == nn4 ) )
%             kk = kk+1;
%             PointSetJoint( kk ) = { zeroPointSetRow, PointSetii{ nn4 } };
%         end
%     end
% end