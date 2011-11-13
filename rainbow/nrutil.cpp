#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "nrutil.h"


// #if defined(__STDC__) || defined(ANSI) || defined(NRANSI) /* ANSI */

//#include <stdio.h>
//#include <stddef.h>


#define NR_END 1
#define FREE_ARG char*


//void nrerror(char error_text[])
///* Numerical Recipes standard error handler */
//{
//	fprintf(stderr,"Numerical Recipes run-time error...\n");
//	fprintf(stderr,"%s\n",error_text);
//	fprintf(stderr,"...now exiting to system...\n");
//	exit(1);
//}

void nrerror(char error_text[])
/* Numerical Recipes standard error handler */
{
	printf("\nNumerical Recipes run-time error...\n");
	printf("%s\n",error_text);
}

float *vector(unsigned int nl, unsigned int nh)
/* allocate a float vector with subscript range v[nl..nh] */
{
	float *v;

	v=(float *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(float)));
	if (!v) nrerror("allocation failure in vector()");
	return v-nl+NR_END;
}

int *ivector(unsigned int nl, unsigned int nh)
/* allocate an int vector with subscript range v[nl..nh] */
{
	int *v;

	v=(int *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(int)));
	if (!v) nrerror("allocation failure in ivector()");
	return v-nl+NR_END;
}

unsigned char *cvector(long nl, long nh)
/* allocate an unsigned char vector with subscript range v[nl..nh] */
{
	unsigned char *v;

	v=(unsigned char *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(unsigned char)));
	if (!v) nrerror("allocation failure in cvector()");
	return v-nl+NR_END;
}

unsigned long *lvector(long nl, long nh)
/* allocate an unsigned long vector with subscript range v[nl..nh] */
{
	unsigned long *v;

	v=(unsigned long *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(long)));
	if (!v) nrerror("allocation failure in lvector()");
	return v-nl+NR_END;
}

double *dvector(long nl, long nh)
/* allocate a double vector with subscript range v[nl..nh] */
{
	double *v;

	v=(double *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(double)));
	if (!v) nrerror("allocation failure in dvector()");
	return v-nl+NR_END;
}

float **matrix(unsigned int nrl, unsigned int nrh, unsigned int ncl, unsigned int nch)
/* allocate a float matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	unsigned int i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	float **m;

	/* allocate pointers to rows */
	m=(float **) malloc((size_t)((nrow+NR_END)*sizeof(float*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;

	/* allocate rows and set pointers to them */
	m[nrl]=(float *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(float)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

double **dmatrix(long nrl, long nrh, long ncl, long nch)
/* allocate a double matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	double **m;

	/* allocate pointers to rows */
	m=(double **) malloc((size_t)((nrow+NR_END)*sizeof(double*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;

	/* allocate rows and set pointers to them */
	m[nrl]=(double *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(double)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

int **imatrix(long nrl, long nrh, long ncl, long nch)
/* allocate a int matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	int **m;

	/* allocate pointers to rows */
	m=(int **) malloc((size_t)((nrow+NR_END)*sizeof(int*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;


	/* allocate rows and set pointers to them */
	m[nrl]=(int *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(int)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

float **submatrix(float **a, long oldrl, long oldrh, long oldcl, long oldch,
	long newrl, long newcl)
/* point a submatrix [newrl..][newcl..] to a[oldrl..oldrh][oldcl..oldch] */
{
	long i,j,nrow=oldrh-oldrl+1,ncol=oldcl-newcl;
	float **m;

	/* allocate array of pointers to rows */
	m=(float **) malloc((size_t) ((nrow+NR_END)*sizeof(float*)));
	if (!m) nrerror("allocation failure in submatrix()");
	m += NR_END;
	m -= newrl;

	/* set pointers to rows */
	for(i=oldrl,j=newrl;i<=oldrh;i++,j++) m[j]=a[i]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

float **convert_matrix(float *a, long nrl, long nrh, long ncl, long nch)
/* allocate a float matrix m[nrl..nrh][ncl..nch] that points to the matrix
declared in the standard C manner as a[nrow][ncol], where nrow=nrh-nrl+1
and ncol=nch-ncl+1. The routine should be called with the address
&a[0][0] as the first argument. */
{
	long i,j,nrow=nrh-nrl+1,ncol=nch-ncl+1;
	float **m;

	/* allocate pointers to rows */
	m=(float **) malloc((size_t) ((nrow+NR_END)*sizeof(float*)));
	if (!m) nrerror("allocation failure in convert_matrix()");
	m += NR_END;
	m -= nrl;

	/* set pointers to rows */
	m[nrl]=a-ncl;
	for(i=1,j=nrl+1;i<nrow;i++,j++) m[j]=m[j-1]+ncol;
	/* return pointer to array of pointers to rows */
	return m;
}

float ***f3tensor(long nrl, long nrh, long ncl, long nch, long ndl, long ndh)
/* allocate a float 3tensor with range t[nrl..nrh][ncl..nch][ndl..ndh] */
{
	long i,j,nrow=nrh-nrl+1,ncol=nch-ncl+1,ndep=ndh-ndl+1;
	float ***t;

	/* allocate pointers to pointers to rows */
	t=(float ***) malloc((size_t)((nrow+NR_END)*sizeof(float**)));
	if (!t) nrerror("allocation failure 1 in f3tensor()");
	t += NR_END;
	t -= nrl;

	/* allocate pointers to rows and set pointers to them */
	t[nrl]=(float **) malloc((size_t)((nrow*ncol+NR_END)*sizeof(float*)));
	if (!t[nrl]) nrerror("allocation failure 2 in f3tensor()");
	t[nrl] += NR_END;
	t[nrl] -= ncl;

	/* allocate rows and set pointers to them */
	t[nrl][ncl]=(float *) malloc((size_t)((nrow*ncol*ndep+NR_END)*sizeof(float)));
	if (!t[nrl][ncl]) nrerror("allocation failure 3 in f3tensor()");
	t[nrl][ncl] += NR_END;
	t[nrl][ncl] -= ndl;

	for(j=ncl+1;j<=nch;j++) t[nrl][j]=t[nrl][j-1]+ndep;
	for(i=nrl+1;i<=nrh;i++) {
		t[i]=t[i-1]+ncol;
		t[i][ncl]=t[i-1][ncl]+ncol*ndep;
		for(j=ncl+1;j<=nch;j++) t[i][j]=t[i][j-1]+ndep;
	}

	/* return pointer to array of pointers to rows */
	return t;
}

void free_vector(float *v, unsigned int nl, unsigned int nh)
/* free a float vector allocated with vector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

void free_ivector(int *v, unsigned int nl, unsigned int nh)
/* free an int vector allocated with ivector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

void free_cvector(unsigned char *v, long nl, long nh)
/* free an unsigned char vector allocated with cvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

void free_lvector(unsigned long *v, long nl, long nh)
/* free an unsigned long vector allocated with lvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

void free_dvector(double *v, long nl, long nh)
/* free a double vector allocated with dvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

void free_matrix(float **m, unsigned int nrl, unsigned int nrh, unsigned int ncl, unsigned int nch)
/* free a float matrix allocated by matrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}

void free_dmatrix(double **m, long nrl, long nrh, long ncl, long nch)
/* free a double matrix allocated by dmatrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}

void free_imatrix(int **m, long nrl, long nrh, long ncl, long nch)
/* free an int matrix allocated by imatrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}

void free_submatrix(float **b, long nrl, long nrh, long ncl, long nch)
/* free a submatrix allocated by submatrix() */
{
	free((FREE_ARG) (b+nrl-NR_END));
}

void free_convert_matrix(float **b, long nrl, long nrh, long ncl, long nch)
/* free a matrix allocated by convert_matrix() */
{
	free((FREE_ARG) (b+nrl-NR_END));
}

void free_f3tensor(float ***t, long nrl, long nrh, long ncl, long nch,
	long ndl, long ndh)
/* free a float f3tensor allocated by f3tensor() */
{
	free((FREE_ARG) (t[nrl][ncl]+ndl-NR_END));
	free((FREE_ARG) (t[nrl]+ncl-NR_END));
	free((FREE_ARG) (t+nrl-NR_END));
}




/////////////////////////////////////////////////////// added by Inhyeok

int mult_mv(const float **matrix_mxn, unsigned int m, unsigned int n, const float *vector_nx1, float *result_mx1)	// matrix x vector
{
	unsigned int i, j;
	float sum = 0.;

	for(i=1; i<=m; i++)
	{
		sum = 0.;		
		for(j=1; j<=n; j++)
			sum += matrix_mxn[i][j]*vector_nx1[j];
		
		result_mx1[i] = sum;
	}
	return 0;
}


int mult_mm(const float *matrix_m1xn1[], unsigned int m1, unsigned int n1, const float *matrix_n1xn2[], unsigned int n2, float *result_m1xn2[]) // matrix x matrix
{
	unsigned int i,j,k;
	float sum = 0.;

	for(i=1; i<=m1; i++)
	{				
		for(j=1; j<=n2; j++)
		{
			sum = 0.;
			for(k=1; k<=n1; k++)
				sum += matrix_m1xn1[i][k]*matrix_n1xn2[k][j];
			
			result_m1xn2[i][j] = sum;
		}
	}
	return 0;
}


float dot(const float *vector1_nx1, unsigned int n, const float *vector2_nx1) // vector dot vector
{
	unsigned int i;
	float sum = 0.;

	for(i=1; i<=n; i++)
		sum += vector1_nx1[i]*vector2_nx1[i];

	return sum;
}


int mult_vvt(float scalar, const float *vector_mx1, unsigned int m, const float *vector_nx1, unsigned int n, float **result_mxn) // vector * vector transpose
{
	unsigned int i, j;

	for(i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = scalar*vector_mx1[i]*vector_nx1[j];

	return 0;

}


int mult_vtm(float scalar, const float *vector_mx1, unsigned int m, const float **matrix_mxn, unsigned int n, float *result_nx1) // scalar * vector transpose * matrix
{
	unsigned int i, j;
	float sum;

	for(i=1; i<=n; i++)
	{
		sum = 0.;
		for(j=1; j<=m; j++)
			sum += vector_mx1[j]*matrix_mxn[j][i];
		
		result_nx1[i] = scalar*sum;
	}
	return 0;
}


int trans(float scalar, const float **matrix_mxn, unsigned int m, unsigned int n, float **result_nxm) // scalar * matrix transpose
{
	unsigned int i,j;

	for(i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_nxm[j][i] = scalar*matrix_mxn[i][j];

	return 0;
}

int trans2(float scalar, float **matrix_mxn, unsigned int m, unsigned int n) // scalar * matrix transpose
{
	float temp;
	unsigned int i,j;

	for(i=1; i<=m; i++)
	{
		for(j=1; j<i; j++)
		{
			temp = matrix_mxn[i][j];
			matrix_mxn[i][j] = scalar*matrix_mxn[j][i];
			matrix_mxn[j][i] = scalar*temp;
		}
	}

	return 0;
}



int sum_mm(const float **matrix1_mxn, unsigned int m, unsigned int n, const float **matrix2_mxn, float **result_mxn) // matrix + matrix
{
	unsigned int i,j;

	for (i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = matrix1_mxn[i][j] + matrix2_mxn[i][j];

	return 0;
}


int diff_mm(const float **matrix1_mxn, unsigned int m, unsigned int n, const float **matrix2_mxn, float **result_mxn) // matrix - matrix
{
	unsigned int i,j;

	for (i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = matrix1_mxn[i][j] - matrix2_mxn[i][j];

	return 0;
}


int sum_vv(const float *vector1_mx1, unsigned int m, const float *vector2_mx1, float *result_mx1) // vector + vector
{
	unsigned int i;

	for(i=1; i<=m; i++)
		result_mx1[i] = vector1_mx1[i] + vector2_mx1[i];

	return 0;
}


int diff_vv(const float *vector1_mx1, unsigned int m, const float *vector2_mx1, float *result_mx1) // vector - vector
{
	unsigned int i;

	for(i=1; i<=m; i++)
		result_mx1[i] = vector1_mx1[i] - vector2_mx1[i];

	return 0;
}


int cross(float scalar, const float vector1_3x1[4], const float vector2_3x1[4], float result_3x1[4])	// scalar * vector cross product
{
	result_3x1[1] = scalar*(vector1_3x1[2]*vector2_3x1[3] - vector1_3x1[3]*vector2_3x1[2]);
	result_3x1[2] = scalar*(vector1_3x1[3]*vector2_3x1[1] - vector1_3x1[1]*vector2_3x1[3]);
	result_3x1[3] = scalar*(vector1_3x1[1]*vector2_3x1[2] - vector1_3x1[2]*vector2_3x1[1]);

	return 0;
}


int mult_sv(const float *vector_nx1, unsigned int n, float scalar, float *result_nx1)		// scalar * vector
{
	unsigned int i;

	for(i=1; i<=n; i++)
		result_nx1[i] = scalar*vector_nx1[i];

	return 0;
}


int mult_sm(const float **matrix_mxn, unsigned int m, unsigned int n, float scalar, float **result_mxn) // scalar * matrix
{
	unsigned int i,j;

	for(i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = scalar*matrix_mxn[i][j];

	return 0;
}

int subs_v(const float *vector_nx1, unsigned int n, float *result_nx1)		// result_nx1 = vector_nx1
{
	unsigned int i;
	
	for(i=1; i<=n; i++)
		result_nx1[i] = vector_nx1[i];

	return 0;
}
int subs_m(const float **matrix_mxn, unsigned int m, unsigned int n, float **result_mxn) // result_mxn = matrix_mxn
{

	unsigned int i,j;

	for(i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = matrix_mxn[i][j];

	return 0;
}


int sum_svsv(float scalar1, const float *vector1_nx1, unsigned int n, float scalar2, const float *vector2_nx1, float *result_nx1) // s1*v1 + s2*v2
{
	unsigned int i;

	for(i=1; i<=n; i++)
		result_nx1[i] = scalar1*vector1_nx1[i] + scalar2*vector2_nx1[i];

	return 0;
}


int sum_smsm(float scalar1, const float **matrix1_mxn, unsigned int m, unsigned int n, float scalar2, const float **matrix2_mxn, float **result_mxn) // s1*m1 + s2*m2
{
	unsigned int i,j;

	for (i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = scalar1*matrix1_mxn[i][j] + scalar2*matrix2_mxn[i][j];

	return 0;
}


int mult_smv(float scalar, const float **matrix_mxn, unsigned int m, unsigned int n, const float *vector_nx1, float *result_mx1)	// scalar * matrix * vector
{
	unsigned int i, j;
	float sum = 0.;

	for(i=1; i<=m; i++)
	{
		sum = 0.;		
		for(j=1; j<=n; j++)
			sum += matrix_mxn[i][j]*vector_nx1[j];
		
		result_mx1[i] = scalar*sum;
	}
	return 0;
}


int mult_smm(float scalar, const float **matrix_m1xn1, unsigned int m1, unsigned int n1, const float **matrix_n1xn2, unsigned int n2, float **result_m1xn2) // scalar*matrix*matrix
{
	unsigned int i,j,k;
	float sum = 0.;

	for(i=1; i<=m1; i++)
	{				
		for(j=1; j<=n2; j++)
		{
			sum = 0.;
			for(k=1; k<=n1; k++)
				sum += matrix_m1xn1[i][k]*matrix_n1xn2[k][j];
			
			result_m1xn2[i][j] = scalar*sum;
		}
	}
	return 0;
}

int accu_vv(float scalar, const float *vector_n1x1, unsigned int n1, const float *vector_n2x1, unsigned int n2, float *result_n1n2x1) // accumulate two vectors
{
	unsigned int i;

	for(i=1; i<=n1; i++)
		result_n1n2x1[i] = scalar*vector_n1x1[i];

	for(i=1; i<=n2; i++)
		result_n1n2x1[n1+i] = scalar*vector_n2x1[i];

	return 0;
}

int aug_vv(float scalar, const float *vector1_nx1, unsigned int n, const float *vector2_nx1, float **result_nx2) // augment a vector
{
	unsigned int i;

	for(i=1; i<=n; i++)
	{
		result_nx2[i][1] = scalar*vector1_nx1[i];
		result_nx2[i][2] = scalar*vector2_nx1[i];
	}

	return 0;
}

int accu_mm(float scalar, const float **matrix_m1xn, unsigned int m1, unsigned int n, const float **matrix_m2xn, unsigned int m2, float **result_m1m2xn) // accumulate two matrices
{
	unsigned int i, j;

	for(j=1; j<=n; j++)
	{
		for(i=1; i<=m1; i++)		
			result_m1m2xn[i][j] = scalar*matrix_m1xn[i][j];

		for(i=1; i<=m2; i++)
			result_m1m2xn[m1+i][j] = scalar*matrix_m2xn[i][j];
	}

	return 0;
}


int aug_mm(float scalar, const float **matrix_mxn1, unsigned int m, unsigned int n1, const float **matrix_mxn2, unsigned int n2, float **result_mxn1n2) // augment a matrix
{
	unsigned int i, j;

	for(i=1; i<=m; i++)
	{
		for(j=1; j<=n1; j++)
			result_mxn1n2[i][j] = matrix_mxn1[i][j];

		for(j=1; j<=n2; j++)
			result_mxn1n2[i][j+n1] = matrix_mxn2[i][j];
	}

	return 0;
}


int aug_mv(float scalar, const float **matrix_mxn, unsigned int m, unsigned int n, const float *vector_mx1, float **result_mxn1) // augment a matrix
{
	unsigned int i,j;

	for(i=1; i<=m; i++)
	{
		for(j=1; j<=n; j++)
			result_mxn1[i][j] = matrix_mxn[i][j];

		result_mxn1[i][n+1] = vector_mx1[i];
	}
	
	return 0;
}


#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int gaussj_mod(float **A, int n, float *X)
{
	int *indxc,*indxr,*ipiv;
	int i,icol,irow,j,k,l,ll;
	float big,dum,pivinv,temp;

	indxc=ivector(1,n);
	indxr=ivector(1,n);
	ipiv=ivector(1,n);
	for (j=1;j<=n;j++) 
		ipiv[j]=0;
	
	for (i=1;i<=n;i++)
	{
		big=0.;
		for (j=1;j<=n;j++)
		{
			if (ipiv[j] != 1)
			{
				for (k=1;k<=n;k++)
				{
					if (ipiv[k] == 0)
					{
						if (fabs(A[j][k]) >= big)
						{
							big=(float)fabs(A[j][k]);
							irow=j;	
							icol=k;
						}
					}
				}
			}
		}
		++(ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=n;l++)
				SWAP(A[irow][l],A[icol][l])
			SWAP(X[irow],X[icol])
		}
		indxr[i]=irow;
		indxc[i]=icol;
		if (A[icol][icol] == 0.0) 
			return 1;
		pivinv=1.f/A[icol][icol];
		A[icol][icol]=1.f;
		for (l=1;l<=n;l++) 
			A[icol][l] *= pivinv;
		X[icol] *= pivinv;
		for (ll=1;ll<=n;ll++)
			if (ll != icol) {
				dum=A[ll][icol];
				A[ll][icol]=0.0;
				for (l=1;l<=n;l++) 
					A[ll][l] -= A[icol][l]*dum;
				X[ll] -= X[icol]*dum;
			}
	}

	for (l=n;l>=1;l--) {
		if (indxr[l] != indxc[l])
			for (k=1;k<=n;k++)
				SWAP(A[k][indxr[l]],A[k][indxc[l]]);
	}
	free_ivector(ipiv,1,n);
	free_ivector(indxr,1,n);
	free_ivector(indxc,1,n);

	return 0;
}
#undef SWAP


#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int inv(float **Ai, int n)
{
	int *indxc,*indxr,*ipiv;
	int i,icol,irow,j,k,l,ll;
	float big,dum,pivinv,temp;
	
	indxc=ivector(1,n);
	indxr=ivector(1,n);
	ipiv=ivector(1,n);
	for (j=1;j<=n;j++) 
		ipiv[j]=0;
	
	for (i=1;i<=n;i++)
	{
		big=0.;
		for (j=1;j<=n;j++)
		{
			if (ipiv[j] != 1)
			{
				for (k=1;k<=n;k++)
				{
					if (ipiv[k] == 0)
					{
						if (fabs(Ai[j][k]) >= big)
						{
							big=(float)fabs(Ai[j][k]);
							irow=j;	
							icol=k;
						}
					}
				}
			}
		}
		++(ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=n;l++)
				SWAP(Ai[irow][l],Ai[icol][l])
		}
		indxr[i]=irow;
		indxc[i]=icol;
		if (Ai[icol][icol] == 0.0) 
			return 1;
		pivinv=1.f/Ai[icol][icol];
		Ai[icol][icol]=1.f;
		for (l=1;l<=n;l++) 
			Ai[icol][l] *= pivinv;
		for (ll=1;ll<=n;ll++)
			if (ll != icol) {
				dum=Ai[ll][icol];
				Ai[ll][icol]=0.0;
				for (l=1;l<=n;l++) 
					Ai[ll][l] -= Ai[icol][l]*dum;
			}
	}

	for (l=n;l>=1;l--) {
		if (indxr[l] != indxc[l])
			for (k=1;k<=n;k++)
				SWAP(Ai[k][indxr[l]],Ai[k][indxc[l]]);
	}
	free_ivector(ipiv,1,n);
	free_ivector(indxr,1,n);
	free_ivector(indxc,1,n);
	
	return 0;
}
#undef SWAP


#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int inv2(const float **A, int n, float **Ai)
{
	int *indxc,*indxr,*ipiv;
	int i,icol,irow,j,k,l,ll;
	float big,dum,pivinv,temp;

	subs_m(A,n,n, Ai);

	indxc=ivector(1,n);
	indxr=ivector(1,n);
	ipiv=ivector(1,n);
	for (j=1;j<=n;j++) 
		ipiv[j]=0;
	
	for (i=1;i<=n;i++)
	{
		big=0.;
		for (j=1;j<=n;j++)
		{
			if (ipiv[j] != 1)
			{
				for (k=1;k<=n;k++)
				{
					if (ipiv[k] == 0)
					{
						if (fabs(Ai[j][k]) >= big)
						{
							big=(float)fabs(Ai[j][k]);
							irow=j;	
							icol=k;
						}
					}
				}
			}
		}
		++(ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=n;l++)
				SWAP(Ai[irow][l],Ai[icol][l])
		}
		indxr[i]=irow;
		indxc[i]=icol;
		if (Ai[icol][icol] == 0.0) 
			return 1;
		pivinv=1.f/Ai[icol][icol];
		Ai[icol][icol]=1.f;
		for (l=1;l<=n;l++) 
			Ai[icol][l] *= pivinv;
		for (ll=1;ll<=n;ll++)
			if (ll != icol) {
				dum=Ai[ll][icol];
				Ai[ll][icol]=0.0;
				for (l=1;l<=n;l++) 
					Ai[ll][l] -= Ai[icol][l]*dum;
			}
	}

	for (l=n;l>=1;l--) {
		if (indxr[l] != indxc[l])
			for (k=1;k<=n;k++)
				SWAP(Ai[k][indxr[l]],Ai[k][indxc[l]]);
	}
	free_ivector(ipiv,1,n);
	free_ivector(indxr,1,n);
	free_ivector(indxc,1,n);
	
	return 0;
}
#undef SWAP




#define SWAP(a,b) temp=(a);(a)=(b);(b)=temp;
#define SWAPi(a,b) tempi=(a);(a)=(b);(b)=tempi;

// example 
// arr_4x1[] = {0, 4, 3, 2, 1}
// index_4x1[] = {0, 1, 2, 3, 4}
//
// nrselect(1,4, arr_4x1, index_4x1)
// arr_4x1 : {0, 1, 2, 3, 4}
// index_4x1 : {0, 4, 3, 2, 1}
// returned value : 1

float nrselect(unsigned int k, unsigned int n, float arr[], unsigned int index[])	
{
	unsigned int i,ir,j,l,mid;
	float a,temp;
	unsigned int ind, tempi;

	l=1;
	ir=n;
	for (;;) {
		if (ir <= l+1) {
			if (ir == l+1 && arr[ir] < arr[l]) {
				SWAP(arr[l],arr[ir])
				SWAPi(index[l],index[ir])
			}
			return arr[k];
		} else {
			mid=(l+ir) >> 1;
			SWAP(arr[mid],arr[l+1])
			SWAPi(index[mid], index[l+1])
			if (arr[l] > arr[ir]) {
				SWAP(arr[l],arr[ir])
				SWAPi(index[l],index[ir])
			}
			if (arr[l+1] > arr[ir]) {
				SWAP(arr[l+1],arr[ir])
				SWAPi(index[l+1],index[ir])
			}
			if (arr[l] > arr[l+1]) {
				SWAP(arr[l],arr[l+1])
				SWAPi(index[l],index[l+1])
			}
			i=l+1;
			j=ir;
			a=arr[l+1];
			ind = index[l+1];
			for (;;) {
				do i++; while (arr[i] < a);
				do j--; while (arr[j] > a);
				if (j < i) break;
				SWAP(arr[i],arr[j])
				SWAPi(index[i],index[j])
			}
			arr[l+1]=arr[j];
			index[l+1] = index[j];
			arr[j]=a;
			index[j] = ind;
			if (j >= k) ir=j-1;
			if (j <= k) l=i;
		}
	}
}
#undef SWAP
#undef SWAPi


float norm_v(const float* vector_nx1, unsigned int n)
{
	unsigned int i;
	float sum = 0.;

	for(i=1; i<=n; i++)
		sum += vector_nx1[i]*vector_nx1[i];

	return((float)sqrt(sum));
}



int findminmax(float data[], unsigned int n, float *result_max, float *result_min, unsigned int *i_max, unsigned int *i_min)
{
	unsigned int i, n_max, n_min;
	float max = -99999999.f;
	float min = 99999999.f;

	for(i=0; i<n; i++)
	{
		if(max < data[i])
		{
			max = data[i];
			n_max = i;
		}		
		
		if(min > data[i])
		{
			min = data[i];
			n_min = i;
		}
	}

	*result_max = max;
	*i_max = n_max;

	*result_min = min;
	*i_min = n_min;

	return 0;
}


int findmax(float data[], unsigned int n, float *result_max, unsigned int *i_max)
{
	unsigned int i, n_max;
	float max = -99999999.f;

	for(i=0; i<n; i++)
	{
		if(max < data[i])
		{
			max = data[i];
			n_max = i;
		}				
	}

	*result_max = max;
	*i_max = n_max;

	return 0;
}


int findmin(float data[], unsigned int n, float *result_min, unsigned int *i_min)
{
	unsigned int i, n_min;
	float min = 99999999.f;

	for(i=0; i<n; i++)
	{
		if(min > data[i])
		{
			min = data[i];
			n_min = i;
		}				
	}

	*result_min = min;
	*i_min = n_min;

	return 0;
}

///////////////////////////////////////////////////////