function [Z, stdX, meanX] = normalize(X)

    stdX = std(X);
    meanX = mean(X);

    for i=1:size(X, 1),
        for j=1:size(X, 2),
            if stdX(j)==0
                Z(i,j)=0;
            else
                Z(i,j)=(X(i,j)-meanX(j))/stdX(j);
            end
        end
    end

