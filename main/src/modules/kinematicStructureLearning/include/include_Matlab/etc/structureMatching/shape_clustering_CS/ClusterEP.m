function PointSetJoint = ClusterEP( PointSetRow, PointSetii, Save_C, RowHOld, Row, i )
% RowHOld = RowHNew;
count = zeros( length( PointSetRow ), length( PointSetii ) );
cost = zeros( length( PointSetRow ), length( PointSetii ) );
for n1 = 1: length( PointSetRow )
    for n2 = 1: length( PointSetii )
        for n3 = 1: length( PointSetRow{ 1 } )%number of shape in Row, means the number of shapes
            for n4 = 1: length( PointSetii{ 1 } )
                if PointSetRow{n1}(n3)~=0 && PointSetii{n2}(n4)~=0
%                     fprintf(1, 'n1=%d, n2=%d, n3=%d, n4=%d\n a=%d, b=%d\n c=%d, d=%d\n', n1, n2,n3,n4,...
%                        RowHOld{ Row }( n3 ), RowHOld{ i }( n4 ), PointSetRow{n1}(n3),PointSetii{n2}(n4) );
                    numerator = abs( length(PointSetRow{n1}(find(PointSetRow{n1}~=0)))/length(PointSetRow{n1}) - length(PointSetii{n2}(find(PointSetii{n2}~=0)))/length(PointSetii{n2}) );
                    denomin =  length(PointSetRow{n1}(find(PointSetRow{n1}~=0)))/length(PointSetRow{n1}) + length(PointSetii{n2}(find(PointSetii{n2}~=0)))/length(PointSetii{n2});
                    cost( n1, n2 ) = ( cost( n1, n2 )+...
                        Save_C{ RowHOld{ Row }( n3 ), RowHOld{ i }( n4 ) }( PointSetRow{n1}(n3), PointSetii{n2}(n4) ) )*(1 + numerator/denomin);
                    count( n1, n2 ) = count( n1, n2 )+1;
                end
            end
        end
    end
end
costCount = cost ./ count;
[ Cost_EP,Corres_EP ] = OSB( costCount, 2.2 );
% [Corres_EP, Cost_EP] = Hungarian( costCount );
[ c1, c2 ] = find( Corres_EP == 1 );

% flag = 0;
% if length( PointSetRow ) >= length( PointSetii )
%     PointSetJoint = cell( 1, length( PointSetRow ) );
% else
%     PointSetJoint = cell( 1, length( PointSetii ) );
%     flag =1;
% end

% PointSetJoint{ 1: length(c1) }( 1: length( PointSetRow{ 1 } ) ) = PointSetRow{ c1 };
% PointSetJoint{ 1: length(c1) }( 1: length( PointSetii{ 1 } ) ) = PointSetii{ c2 };
for nn1 =1: length(c1)
    PointSetJoint{ nn1 } = [ PointSetRow{ c1(nn1) }, PointSetii{ c2(nn1) } ];
end
% % if length( PointSetii ) > length( PointSetRow )
% %     kk = length(c1);
% %     zeroPointSetRow{ 1 } = zeros( 1, length( PointSetRow{ 1 } ) );
% %     for nn4 = 1: length( PointSetii )
% %         if isempty( find( c2 == nn4 ) )%if nn4 does not in c2
% %             kk = kk+1;
% %             PointSetJoint{ kk } = [ zeroPointSetRow{1}, PointSetii{ nn4 } ];
% % %             PointSetJoint{ length(c1)+1: length(PointSetRow) }( 1:length(PointSetRow{1}) ) = { zeroPointSetRow, PointSetii{ nn4 } };
% %         end
% %     end
% % else
    kk = length(c1);
    zeroPointSetii{ 1 } = zeros( 1, length( PointSetii{ 1 } ) );
    for nn4 = 1: length( PointSetRow )
        if isempty( find( c1 == nn4 ) )
            kk=kk+1;
            PointSetJoint{ kk } = [ PointSetRow{ nn4 }, zeroPointSetii{ 1 } ];
        end
    end
% end
zeroPointSetRow{ 1 } = zeros( 1, length( PointSetRow{ 1 } ) );
for nn5 = 1: length( PointSetii )
    if isempty( find( c2 == nn5 ) )
        kk = kk +1;
        PointSetJoint{ kk }  = [ zeroPointSetRow{1}, PointSetii{ nn5 } ];
    end
end
