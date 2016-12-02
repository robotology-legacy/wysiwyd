function [ ret ] = Choose( sets )

nSetLength = length( sets );

if 1 == nSetLength
    nSetOneLen = length( sets{ 1 } );
    ret = cell( 1, nSetOneLen );
    
    for i = 1 : length( sets{ 1 } )
        ret{ i } = sets{ 1 }( i );
    end
else
    result = Choose( sets( 2 : end ) );
    nSetOneLen = length( sets{ 1 } );
    nRetLen = length( result );
    ret = cell( 1, nSetOneLen * nRetLen );
    
    for i = 1 : nSetOneLen
        for j = 1 : nRetLen
            ret{ ( i - 1 ) * nRetLen + j  } = [ sets{ 1 }( i ) result{ j } ];
        end
    end
end

end