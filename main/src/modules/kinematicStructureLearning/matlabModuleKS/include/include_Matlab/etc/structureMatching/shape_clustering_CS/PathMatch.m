function [ Matching, Cost, Dis ] =PathMatch( JPtoEPMMi, JPtoEPMMj, alpha, c1, c2 )

nJPtoEPMMi = length( JPtoEPMMi );
nJPtoEPMMj = length( JPtoEPMMj );

for i = 1 : nJPtoEPMMi
    for j = 1 : nJPtoEPMMj
        Dis( i , j ) = vertice_distan(JPtoEPMMi{ i }, JPtoEPMMj{ j }, alpha, c1, c2);
    end
end

[ Matching, Cost ] = Hungarian( Dis );
[x, y] = find(Matching == 1);
Cost = Cost / length(x);
