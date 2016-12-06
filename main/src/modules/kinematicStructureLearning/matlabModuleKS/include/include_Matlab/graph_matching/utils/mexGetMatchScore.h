void getMatchScore(int* indH, double* valH, int size, double* X, double* pMatchScore)
{
    double score = 0;
    for(int i = 0; i < size; i++)
        score += valH[i]*X[indH[i]]*X[indH[i+size]]*X[indH[i+2*size]];
    pMatchScore[0] = score;
}
