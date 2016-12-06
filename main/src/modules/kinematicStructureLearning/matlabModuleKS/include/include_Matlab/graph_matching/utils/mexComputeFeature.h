/* feat1 feat2 <- P1 P2 t1 type */

void computeFeatureSimple( double* pP1, int i , int j, int k , double* pF);
void computeFeature( double* pP1 , int nP1 , double* pP2 , int nP2 ,
                      int* pT1 , int nT1 , double* pF1 , double* pF2);

void computeFeature( double* pP1 , int nP1 , double* pP2 , int nP2 ,
                      int* pT1 , int nT1 , double* pF1 , double* pF2)
{ 
  const int nFeature=3;
  for(int t=0;t<nT1;t++)
  {
    computeFeatureSimple(pP1,pT1[t*3],pT1[t*3+1],pT1[t*3+2],pF1+t*nFeature);
  }
  
  for(int i=0;i<nP2;i++)
    for(int j=0;j<nP2;j++)
      for(int k=0;k<nP2;k++)
        computeFeatureSimple(pP2,i,j,k,pF2+((i*nP2+j)*nP2+k)*nFeature);
  
}

void computeFeatureSimple( double* pP1, int i , int j, int k , double* pF)
{ 
  const int nFeature=3;
  double vecX[nFeature];
  double vecY[nFeature];
  int ind[nFeature];
  ind[0]=i;ind[1]=j;ind[2]=k;
  double n;
  int f;
  if((ind[0]==ind[1])||(ind[0]==ind[2])||(ind[1]==ind[2]))
  {
    pF[0]=pF[1]=pF[2]=-10;
    return;
  }
  for(f=0;f<nFeature;f++)
  {
    vecX[f]=pP1[ind[((f+1)%3)]*2]-pP1[ind[f]*2];
    vecY[f]=pP1[ind[((f+1)%3)]*2+1]-pP1[ind[f]*2+1];
    double norm=sqrt(vecX[f]*vecX[f]+vecY[f]*vecY[f]);
    if(norm!=0)
    {
      vecX[f]/=norm;
      vecY[f]/=norm;
    }else{
      vecX[f]=0;
      vecY[f]=0;
    }
  }
  for(f=0;f<nFeature;f++)
  {
    pF[f] = vecX[((f+1)%3)]*vecY[f]-vecY[((f+1)%3)]*vecX[f];
  }
}
















