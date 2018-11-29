#ifndef Y_INTERP_GUARD// See Yager, R.J. "Basic Searching, Interpolating, and 
#define Y_INTERP_GUARD// Curve Fitting Algorithms in C++" (ARL-TN-XXX) 
#include <cmath>//..................................................fmod(),pow()

namespace yInterp
{
    /*  BinarySearch
    *   FIND THE POINTER c | *c <= k < *(c+1)
    *   *a, *b ARRAY START & END POINTS (ARRAY MUST BE SORTED IN INC. ORDER)
    *   k KEY
    */
    template<class T>T*BinarySearch(//<======FIND THE POINTER c | *c <= k < *(c+1)
    T*a,T*b,//<--ARRAY START & END POINTS (ARRAY MUST BE SORTED IN INC. ORDER)
    T k){//<---------------------------------------------------------------KEY
    if(k<*a)return a-1;//..........note that a-1 may point to an invalid address
    for(T*c;k<*--b;k>*c?a=c:b=c+1)c=a+(b-a)/2;/*->*/return b;
    }//~~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~21JUL2014~~~~~~


    template<class T>T*PeriodicSearch(//<=======FOR ARRAYS WITH PERIODIC VARIABLES
    T*a,T*b,//<--STARTING & ENDING POINTS (ARRAY MUST BE SORTED IN INC. ORDER)
    T&k,//<---------------------------------------KEY (WILL BE SET TO k'=k+np)
    T p){//<------------------------------------------------------PERIOD (p>0)
    return BinarySearch(a,b,k=fmod(k-*a,p)+*a+(k-*a<0?p:0));
    }//~~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~21JUL2014~~~~~~

    template<class T>T NNInterp(//<==================NEAREST-NEIGHBOR INTERPOLATOR
    const T*X,//<--------------BRACKETING X VALUES (*X AND X[1] MUST BE VALID)
    const T*Y,//<--------------BRACKETING Y VALUES (*Y AND Y[1] MUST BE VALID)
    T x){//<--------------VALUE TO INTERPOLATE AT (TYPICALLY, *X <= x <= x[1])
    return x>(*X+X[1])/2?Y[1]:*Y;
    }//~~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~21JUL2014~~~~~~

    template<class T>T LinInterp(//<===========================LINEAR INTERPOLATOR
    const T*X,//<--------------BRACKETING X VALUES (*X AND X[1] MUST BE VALID)
    const T*Y,//<--------------BRACKETING Y VALUES (*Y AND Y[1] MUST BE VALID)
    T x){//<--------------VALUE TO INTERPOLATE AT (TYPICALLY, *X <= x <= x[1])
    return*Y+(Y[1]-*Y)*(x-*X)/(X[1]-*X);
    }//~~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~21JUL2014~~~~~~

    template<class T>T CubeInterp(//<==========CUBIC (HERMITE SPLINE) INTERPOLATOR 
    const T*X,//<--------------BRACKETING X VALUES (*X AND X[1] MUST BE VALID) 
    const T*Y,//<--------------BRACKETING Y VALUES (*Y AND Y[1] MUST BE VALID) 
    T x,//<---------------VALUE TO INTERPOLATE AT (TYPICALLY, *X <= x <= X[1]) 
    T m0,T m1){//<---------------------------------------SLOPES AT *X AND X[1] 
    T t=(x-*X)/(X[1]-*X); 
    return*Y+(x-*X)*(((m0+m1)*t-(2*m0+m1))*t+m0)+(Y[1]-*Y)*(3-2*t)*t*t; 
    }//~~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~21JUL2014~~~~~~


    template<class T>T CardinalSlope(//<========CALCULATES SLOPES FOR CubeInterp()
    const T*X,//<-----------BRACKETING X VALUES (X[-1] AND X[1] MUST BE VALID)
    const T*Y,//<-----------BRACKETING Y VALUES (Y[-1] AND Y[1] MUST BE VALID)
    T t){//<-------------------------------------------------TENSION PARAMETER
    return(1-t)*(Y[1]-Y[-1])/(X[1]-X[-1]);
    }//~~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~21JUL2014~~~~~~


    template<class T>void PolyFit(//<=========FITS A POLYNOMIAL TO A SET OF POINTS
    const T*X,//<-------INDEPENDENT-VARIABLE VALUES (EACH X[i] MUST BE UNIQUE)
    const T*Y,//<-------------------DEPENDENT-VARIABLE VALUES (SAME SIZE AS X)
    int n,//<-------------------------------------NUMBER OF ELEMENTS IN X OR Y
    int m,//<----------------NUMBER OF COEFFICIENTS IN THE BEST-FIT POLYNOMIAL
    T*C){//<------STORAGE FOR COEFFICIENTS (SIZE=m) y=C[0]+C[1]*x+C[2]*x^2+...
    T**A=new T*[m];/*<-*/for(int i=0,j,k;i<m;++i){//............augmented matrix
    for(A[i]=new T[m+1],j=0;j<m+1;++j)A[i][j]=0;
    for(k=0;k<n;++k)for(A[i][m]+=pow(X[k],i)*Y[k],j=0;j<m;++j)
    A[i][j]+=pow(X[k],i+j);}
    for(int i=1,j,k;i<m;++i)for(k=0;k<i;++k)for(j=m;j>=0;--j)//.........Gaussian
    A[i][j]-=A[i][k]*A[k][j]/A[k][k];// elimination
    for(int i=m-1,j;i>=0;--i)for(C[i]=A[i][m]/A[i][i],j=i+1;j<m;++j)//..backward
    C[i]-=C[j]*A[i][j]/A[i][i];// substitution
    for(int i=0;i<m;++i)delete[]A[i];/*&*/delete[]A;
    }//~~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~21JUL2014~~~~~~
        
}

#endif