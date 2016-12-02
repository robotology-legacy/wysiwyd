function out = numElements(nP1, nP2, order)

    out = factorial(nP1) / factorial(nP1-order) * factorial(nP2) / factorial(nP2-order);

end