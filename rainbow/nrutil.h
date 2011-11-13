#ifndef _NR_UTILS_H_
#define _NR_UTILS_H_

extern "C" {

static float sqrarg;
#define SQR(a) ((sqrarg=(a)) == 0.0 ? 0.0 : sqrarg*sqrarg)

static double dsqrarg;
#define DSQR(a) ((dsqrarg=(a)) == 0.0 ? 0.0 : dsqrarg*dsqrarg)

static double dmaxarg1,dmaxarg2;
#define DMAX(a,b) (dmaxarg1=(a),dmaxarg2=(b),(dmaxarg1) > (dmaxarg2) ?\
        (dmaxarg1) : (dmaxarg2))

static double dminarg1,dminarg2;
#define DMIN(a,b) (dminarg1=(a),dminarg2=(b),(dminarg1) < (dminarg2) ?\
        (dminarg1) : (dminarg2))

static float maxarg1,maxarg2;
#define FMAX(a,b) (maxarg1=(a),maxarg2=(b),(maxarg1) > (maxarg2) ?\
        (maxarg1) : (maxarg2))

static float minarg1,minarg2;
#define FMIN(a,b) (minarg1=(a),minarg2=(b),(minarg1) < (minarg2) ?\
        (minarg1) : (minarg2))

static long lmaxarg1,lmaxarg2;
#define LMAX(a,b) (lmaxarg1=(a),lmaxarg2=(b),(lmaxarg1) > (lmaxarg2) ?\
        (lmaxarg1) : (lmaxarg2))

static long lminarg1,lminarg2;
#define LMIN(a,b) (lminarg1=(a),lminarg2=(b),(lminarg1) < (lminarg2) ?\
        (lminarg1) : (lminarg2))

static int imaxarg1,imaxarg2;
#define IMAX(a,b) (imaxarg1=(a),imaxarg2=(b),(imaxarg1) > (imaxarg2) ?\
        (imaxarg1) : (imaxarg2))

static int iminarg1,iminarg2;
#define IMIN(a,b) (iminarg1=(a),iminarg2=(b),(iminarg1) < (iminarg2) ?\
        (iminarg1) : (iminarg2))

#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

void nrerror(char error_text[]);
float *vector(unsigned int nl, unsigned int nh);
int *ivector(unsigned int nl, unsigned int nh);
unsigned char *cvector(long nl, long nh);
unsigned long *lvector(long nl, long nh);
double *dvector(long nl, long nh);
float **matrix(unsigned int nrl, unsigned int nrh, unsigned int ncl, unsigned int nch);
double **dmatrix(long nrl, long nrh, long ncl, long nch);
int **imatrix(long nrl, long nrh, long ncl, long nch);
float **submatrix(float **a, long oldrl, long oldrh, long oldcl, long oldch,
	long newrl, long newcl);
float **convert_matrix(float *a, long nrl, long nrh, long ncl, long nch);
float ***f3tensor(long nrl, long nrh, long ncl, long nch, long ndl, long ndh);
void free_vector(float *v, unsigned int nl, unsigned int nh);
void free_ivector(int *v, unsigned int nl, unsigned int nh);
void free_cvector(unsigned char *v, long nl, long nh);
void free_lvector(unsigned long *v, long nl, long nh);
void free_dvector(double *v, long nl, long nh);
void free_matrix(float **m, unsigned int nrl, unsigned int nrh, unsigned int ncl, unsigned int nch);
void free_dmatrix(double **m, long nrl, long nrh, long ncl, long nch);
void free_imatrix(int **m, long nrl, long nrh, long ncl, long nch);
void free_submatrix(float **b, long nrl, long nrh, long ncl, long nch);
void free_convert_matrix(float **b, long nrl, long nrh, long ncl, long nch);
void free_f3tensor(float ***t, long nrl, long nrh, long ncl, long nch,
	long ndl, long ndh);


/////////////////////////////////////////////////////// added by Inhyeok

int mult_mv(const float **matrix_mxn, unsigned int m, unsigned int n, const float *vector_nx1, float *result_mx1);	// matrix x vector
int mult_mm(const float **matrix_m1xn1, unsigned int m1, unsigned int n1, const float **matrix_n1xn2, unsigned int n2, float **result_m1xn2); // matrix x matrix
float dot(const float *vector1_nx1, unsigned int n, const float *vector2_nx1); // vector dot vector

int sum_mm(const float **matrix1_mxn, unsigned int m, unsigned int n, const float **matrix2_mxn, float **result_mxn); // matrix + matrix
int diff_mm(const float **matrix1_mxn, unsigned int m, unsigned int n, const float **matrix2_mxn, float **result_mxn); // matrix - matrix
int sum_vv(const float *vector1_mx1, unsigned int m, const float *vector2_mx1, float *result_mx1); // vector + vector
int diff_vv(const float *vector1_mx1, unsigned int m, const float *vector2_mx1, float *result_mx1); // vector - vector
int mult_sv(const float *vector_nx1, unsigned int n, float scalar, float *result_nx1);		// scalar * vector
int mult_sm(const float **matrix_mxn, unsigned int m, unsigned int n, float scalar, float **result_mxn); // scalar * matrix
int subs_v(const float *vector_nx1, unsigned int n, float *result_nx1);		// result_nx1 = vector_nx1
int subs_m(const float **matrix_mxn, unsigned int m, unsigned int n, float **result_mxn); // result_mxn = matrix_mxn

int sum_svsv(float scalar1, const float *vector1_nx1, unsigned int n, float scalar2, const float *vector2_nx1, float *result_nx1); // s1*v1 + s2*v2
int sum_smsm(float scalar1, const float **matrix1_mxn, unsigned int m, unsigned int n, float scalar2, const float **matrix2_mxn, float **result_mxn); // s1*m1 + s2*m2
int mult_smv(float scalar, const float **matrix_mxn, unsigned int m, unsigned int n, const float *vector_nx1, float *result_mx1);	// scalar * matrix * vector
int mult_smm(float scalar, const float **matrix_m1xn1, unsigned int m1, unsigned int n1, const float **matrix_n1xn2, unsigned int n2, float **result_m1xn2); // scalar*matrix*matrix
int trans(float scalar, const float **matrix_mxn, unsigned int m, unsigned int n, float **result_nxm); // scalar * matrix transpose
int trans2(float scalar, float **matrix_mxn, unsigned int m, unsigned int n); // scalar * matrix transpose
int cross(float scalar, const float vector1_3x1[4], const float vector2_3x1[4], float result_3x1[4]);	// scalar * vector cross product
int mult_vvt(float scalar, const float *vector_mx1, unsigned int m, const float *vector_nx1, unsigned int n, float **result_mxn); // vector * vector transpose
int mult_vtm(float scalar, const float *vector_mx1, unsigned int m, const float **matrix_mxn, unsigned int n, float *result_nx1); // scalar * vector transpose * matrix

int accu_vv(float scalar, const float *vector_n1x1, unsigned int n1, const float *vector_n2x1, unsigned int n2, float *result_n1n2x1); // accumulate two vectors
int aug_vv(float scalar, const float *vector1_nx1, unsigned int n, const float *vector2_nx1, float **result_nx2); // augment a vector
int accu_mm(float scalar, const float **matrix_m1xn, unsigned int m1, unsigned int n, const float **matrix_m2xn, unsigned int m2, float **result_m1m2xn); // accumulate two matrices
int aug_mm(float scalar, const float **matrix_mxn1, unsigned int m, unsigned int n1, const float **matrix_mxn2, unsigned int n2, float **result_mxn1n2); // augment a matrix
int aug_mv(float scalar, const float **matrix_mxn, unsigned int m, unsigned int n, const float *vector_mx1, float **result_mxn1); // augment a matrix


int gaussj_mod(float **A, int n, float *X);
int inv(float **Ai, int n);
int inv2(const float **A, int n, float **Ai);

float nrselect(unsigned int k, unsigned int n, float arr[], unsigned int index[]);	// modified by Inhyeok

float norm_v(const float* vector_nx1, unsigned int n);
//void pinv(const float **A, int m, int n, float **Ai);

int findminmax(float data[], unsigned int n, float *result_max, float *result_min, unsigned int *i_max, unsigned int *i_min);
int findmax(float data[], unsigned int n, float *result_max, unsigned int *i_max);
int findmin(float data[], unsigned int n, float *result_min, unsigned int *i_min);

///////////////////////////////////////////////////////////////////////

}
#endif /* _NR_UTILS_H_ */
