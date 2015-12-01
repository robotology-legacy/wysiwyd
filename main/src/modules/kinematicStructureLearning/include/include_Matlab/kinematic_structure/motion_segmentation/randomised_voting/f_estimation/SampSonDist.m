function dist = SampSonDist(F, x1, x2)

    a = F*x1;
    b = F'*x2;
    
    A = (x2'*F)';
    v = sum(A.*x1);

    dist = (v).^2 ./ (a(1,:).^2 + a(2,:).^2 + b(1,:).^2 + b(2,:).^2);
end