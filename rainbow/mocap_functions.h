#ifndef MOCAP_FUNCTIONS_H
#define MOCAP_FUNCTIONS_H

//typedef unsigned int BOOL;
//#define TRUE		1
//#define FALSE		0

// #define PI			3.141592653589793f
#define D2R			0.017453292519943f
#define R2D			57.295779513082323f
#define MARGIN		D2R

#define DT			0.005f


// feedback control gains
#define KP_MOCAP	100.f				// wn=1/dt*0.05, zeta=1, Kp_mo=wn^2, Kv_mo=2*zeta*wn	
#define KV_MOCAP	20.f				// wn=1/dt*0.05, zeta=1, Kp_mo=wn^2, Kv_mo=2*zeta*wn

#define KP_SUPP		22500.f				// wn=1/dt*0.75, zeta=1, Kp_supp=wn^2, Kv_supp=2*zeta*wn
#define KV_SUPP		300.f

#define KP_COM		1600.f				// wn=1/dt*0.2, zeta=1, Kp_supp=wn^2, Kv_supp=2*zeta*wn
#define KV_COM		80.f

#define KP_PEL		1600.f				// wn=1/dt*0.2, zeta=1, Kp_supp=wn^2, Kv_supp=2*zeta*wn
#define KV_PEL		80.f

#define KP_PCZ		1600.f				// wn=1/dt*0.2, zeta=1, Kp_supp=wn^2, Kv_supp=2*zeta*wn
#define KV_PCZ		80.f


// feedback control gains during recovery
#define KP_COM_RECOV		100.f				// wn=1/dt*0.75, zeta=1, Kp_supp=wn^2, Kv_supp=2*zeta*wn
#define KV_COM_RECOV		20.f

#define KP_PEL_RECOV		100.f				// wn=1/dt*0.75, zeta=1, Kp_supp=wn^2, Kv_supp=2*zeta*wn
#define KV_PEL_RECOV		20.f

#define KP_PCZ_RECOV		100.f				// wn=1/dt*0.5, zeta=1, Kp_supp=wn^2, Kv_supp=2*zeta*wn
#define KV_PCZ_RECOV		20.f


#define CX_FOOT		0.0216f
#define CY_FOOT		-0.0014f
#define CZ_FOOT		-0.0160f
#define CX_PEL		-0.0119f
#define CY_PEL		0.
#define CZ_PEL		0.1323f
#define CX_LARM		-0.0197f
#define CY_LARM		-0.0006f
#define CZ_LARM		-0.0470f
#define CX_LLEG		0.0146f
#define CY_LLEG		-0.0146f
#define CZ_LLEG		-0.1845f
#define CX_TOR		-0.0115f
#define CY_TOR		0.
#define CZ_TOR		0.1153f
#define CX_UARM		0.0062f
#define CY_UARM		0.0178f
#define CZ_UARM		-0.0451f
#define CX_ULEG		0.0175f
#define CY_ULEG		-0.0099f
#define CZ_ULEG		-0.0995f
	
#define L_PEL		0.177f
#define L_LEG		0.28f
#define L_PEL2		0.1913f
#define LZ_SHLD		0.3728f
#define LY_SHLD		0.429f
#define L_FOOT		0.101f
#define LX_UARM		0.02f
#define LZ_UARM		0.182f
#define L_LARM		0.1314f

#define m_FOOT		2.6626f
#define m_PEL		3.8736f
#define m_LARM		0.5542f
#define m_LLEG		1.9592f
#define m_TOR		7.2025f
#define m_UARM		2.3147f
#define m_ULEG		6.3512f
#define m_TOTAL		38.7599f //2.*(m_ULEG+m_LLEG+m_UARM+m_LARM+m_FOOT)+m_TOR+m_PEL

//-------- initial position in degree
#define RHP0		-21.7868*D2R	//-12.f*D2R			
#define RKN0		-2.f*RHP0
#define RAP0		RHP0
#define LHP0		RHP0
#define LKN0		RKN0
#define LAP0		RAP0
#define REP0		D2R
#define LEP0		D2R

#define COMX0		0.0162075f
#define COMY0		0.f
#define PCZ0		0.648762656410931f

#define OFFSET_RSR	-10.f
#define OFFSET_LSR	10.f

//---------------

#define ZMPXMAX		0.14f
#define ZMPXMIN		-0.08f
#define ZMPYMAX		0.1685f
#define ZMPYMIN		-0.1685f


//--------------------------------------------- JOINT LIMIT VALUES
#define WSTmax          1.064650843716541f  // 61.f*D2R
#define WSTmin          -1.064650843716541f // -61.f*D2R
#define WSTpmax         3.141592653589793f // 4.188790204786391f // 240.f*D2R
#define WSTppmax        6.283185307179586f //8.377580409572781f// XXXpmax/(100.f*DT)

#define RSPmax          1.570796326794897f // 90.f*D2R
#define RSPmin          -3.141592653589793f // -180.f*D2R
#define RSPpmax         4.188790204786391f // 240.f*D2R
#define RSPppmax        8.377580409572781f// XXXpmax/(100.f*DT)

#define RSRmax          0.087266462599716f // 5.f*D2R
#define RSRmin          -2.094395102393195f //-120.f*D2R
#define RSRpmax         4.188790204786391f // 240.f*D2R
#define RSRppmax        8.377580409572781f// XXXpmax/(100.f*DT)

#define RSYmax          1.570796326794897f // 90.f*D2R
#define RSYmin          -1.570796326794897f// -90.f*D2R
#define RSYpmax         4.188790204786391f // 240.f*D2R
#define RSYppmax        8.377580409572781f// XXXpmax/(100.f*DT)

#define REPmax          0.
#define REPmin          -1.919862177193763f // -110.f*D2R
#define REPpmax         4.188790204786391f // 240.f*D2R
#define REPppmax        8.377580409572781f// XXXpmax/(100.f*DT)

#define REYmax          1.570796326794897f // 90.f*D2R
#define REYmin          -1.570796326794897f // -90.f*D2R
#define REYpmax         4.188790204786391f // 240.f*D2R
#define REYppmax        8.377580409572781f// XXXpmax/(100.f*DT)

#define RWPmax			0.785398163397448f	// 45.f*D2R
#define RWPmin			-0.785398163397448f // -45.f*D2R
#define RWPpmax			4.188790204786391f // 240.f*D2R
#define RWPppmax		8.377580409572781f// XXXpmax/(100.f*DT)

#define LSPmax          RSPmax
#define LSPmin          RSPmin
#define LSPpmax         RSPpmax
#define LSPppmax        RSPppmax

#define LSRmax          2.094395102393195f // 120.f*D2R
#define LSRmin          -0.087266462599716f // -5.f*D2R
#define LSRpmax         RSRpmax
#define LSRppmax        RSRppmax

#define LSYmax          1.570796326794897f // 90.f*D2R
#define LSYmin          -1.570796326794897f // -90.f*D2R
#define LSYpmax         RSYpmax
#define LSYppmax        RSYppmax

#define LEPmax          REPmax
#define LEPmin          REPmin
#define LEPpmax         REPpmax
#define LEPppmax        REPppmax

#define LEYmax          1.570796326794897f // 90.f*D2R
#define LEYmin          -1.570796326794897f // -90.f*D2R
#define LEYpmax         REYpmax
#define LEYppmax        REYppmax

#define LWPmax			RWPmax
#define LWPmin			RWPmin
#define LWPpmax			RWPpmax
#define	LWPppmax		RWPppmax

#define RHYmax          0.558505360638185f // 32.f*D2R
#define RHYmin          -0.558505360638185f  // -32.f*D2R
#define RHYpmax         4.188790204786391f // 240.f*D2R
#define RHYppmax        8.377580409572781f// XXXpmax/(100.f*DT)

#define RHRmax          0.366519142918809f  // 21.f*D2R
#define RHRmin          -0.453785605518526f  // -26.f*D2R
#define RHRpmax         4.188790204786391f // 240.f*D2R
#define RHRppmax        8.377580409572781f// XXXpmax/(100.f*DT)

#define RHPmax          1.012290966156711f // 58.f*D2R
#define RHPmin          -1.483529864195180f// -85.f*D2R // -1.291543646475804f // -74.f*D2R       // -1.012290966156711f // -58.f*D2R
#define RHPpmax         4.188790204786391f // 240.f*D2R
#define RHPppmax        8.377580409572781f// XXXpmax/(100.f*DT)

#define RKNmax          2.146754979953025f // 123.f*D2R
#define RKNmin          0.
#define RKNpmax         4.188790204786391f // 240.f*D2R
#define RKNppmax        8.377580409572781f// XXXpmax/(100.f*DT)

#define RAPmax          1.064650843716541f // 61.f*D2R
#define RAPmin          -1.064650843716541f // -61.f*D2R
#define RAPpmax         4.188790204786391f // 240.f*D2R
#define RAPppmax        8.377580409572781f// XXXpmax/(100.f*DT)

#define RARmax          0.383972435438752f  // 22.f*D2R
#define RARmin          -0.383972435438752f  // -22.f*D2R
#define RARpmax         4.188790204786391f // 240.f*D2R
#define RARppmax        8.377580409572781f// XXXpmax/(100.f*DT)

#define LHYmax          RHYmax
#define LHYmin          RHYmin
#define LHYpmax         RHYpmax
#define LHYppmax        RHYppmax

#define LHRmax          0.453785605518526f  // 26.f*D2R
#define LHRmin          -0.366519142918809f  // -21.f*D2R
#define LHRpmax         RHRpmax
#define LHRppmax        RHRppmax

#define LHPmax          RHPmax
#define LHPmin          RHPmin
#define LHPpmax         RHPpmax
#define LHPppmax        RHPppmax

#define LKNmax          RKNmax
#define LKNmin          RKNmin
#define LKNpmax         RKNpmax
#define LKNppmax        RKNppmax

#define LAPmax          RAPmax
#define LAPmin          RAPmin
#define LAPpmax         RAPpmax
#define LAPppmax        RAPppmax

#define LARmax          0.383972435438752f  // 22.f*D2R
#define LARmin          -0.383972435438752f  // -22.f*D2R
#define LARpmax         RARpmax
#define LARppmax		RARppmax


//---------------------------------------------------------------- for upper body motion window
#define WIND_SIZE_WST	67		// pmax/ppmax/dt -> 최고 속도에서 최대로 감속할 수 있는 window size
#define WIND_SIZE_RSP	67
#define WIND_SIZE_RSR	67
#define WIND_SIZE_RSY	67
#define WIND_SIZE_REB	67
#define WIND_SIZE_RWY	67
#define WIND_SIZE_RWP	67
#define WIND_SIZE_LSP	67
#define WIND_SIZE_LSR	67
#define WIND_SIZE_LSY	67
#define WIND_SIZE_LEB	67
#define WIND_SIZE_LWY	67
#define WIND_SIZE_LWP	67
#define WIND_SIZE_BP	67
#define WIND_SIZE_MAX	67		// window 크기 중 가장 큰 값

typedef struct {
	float wind_WST[WIND_SIZE_WST];
	float wind_RSP[WIND_SIZE_RSP];
	float wind_RSR[WIND_SIZE_RSR];
	float wind_RSY[WIND_SIZE_RSY];
	float wind_REB[WIND_SIZE_REB];
	float wind_RWY[WIND_SIZE_RWY];
	float wind_RWP[WIND_SIZE_RWP];
	float wind_LSP[WIND_SIZE_LSP];
	float wind_LSR[WIND_SIZE_LSR];
	float wind_LSY[WIND_SIZE_LSY];
	float wind_LEB[WIND_SIZE_LEB];
	float wind_LWY[WIND_SIZE_LWY];
	float wind_LWP[WIND_SIZE_LWP];
	float wind_BP[WIND_SIZE_BP];

	float dNK_JW_3x1[4];
	float dRF_JW_5x1[6];
	float dLF_JW_5x1[6];

	float maxWST;
	float maxRSP;
	float maxRSR;
	float maxRSY;
	float maxREB;
	float maxRWY;
	float maxRWP;
	float maxLSP;
	float maxLSR;
	float maxLSY;
	float maxLEB;
	float maxLWY;
	float maxLWP;

	float minWST;
	float minRSP;
	float minRSR;
	float minRSY;
	float minREB;
	float minRWY;
	float minRWP;
	float minLSP;
	float minLSR;
	float minLSY;
	float minLEB;
	float minLWY;
	float minLWP;

	unsigned int index_maxWST;
	unsigned int index_maxRSP;
	unsigned int index_maxRSR;
	unsigned int index_maxRSY;
	unsigned int index_maxREB;
	unsigned int index_maxRWY;
	unsigned int index_maxRWP;
	unsigned int index_maxLSP;
	unsigned int index_maxLSR;
	unsigned int index_maxLSY;
	unsigned int index_maxLEB;
	unsigned int index_maxLWY;
	unsigned int index_maxLWP;

	unsigned int index_minWST;
	unsigned int index_minRSP;
	unsigned int index_minRSR;
	unsigned int index_minRSY;
	unsigned int index_minREB;
	unsigned int index_minRWY;
	unsigned int index_minRWP;
	unsigned int index_minLSP;
	unsigned int index_minLSR;
	unsigned int index_minLSY;
	unsigned int index_minLEB;
	unsigned int index_minLWY;
	unsigned int index_minLWP;

	unsigned int head_WST;
	unsigned int head_RSP;
	unsigned int head_RSR;
	unsigned int head_RSY;
	unsigned int head_REB;
	unsigned int head_RWY;
	unsigned int head_RWP;
	unsigned int head_LSP;
	unsigned int head_LSR;
	unsigned int head_LSY;
	unsigned int head_LEB;
	unsigned int head_LWY;
	unsigned int head_LWP;
	unsigned int head_BP;

	unsigned int tail_WST;
	unsigned int tail_RSP;
	unsigned int tail_RSR;
	unsigned int tail_RSY;
	unsigned int tail_REB;
	unsigned int tail_RWY;
	unsigned int tail_RWP;
	unsigned int tail_LSP;
	unsigned int tail_LSR;
	unsigned int tail_LSY;
	unsigned int tail_LEB;
	unsigned int tail_LWY;
	unsigned int tail_LWP;
	unsigned int tail_BP;

} QUB_WIND;

//---------------------------------------------


int InitGlobalMotionVariables(void);
int FreeGlobalMotionVariables(void);

int UpdateGlobalMotionVariables(void);

int LoadMocapData(int MocapNo);
int ClearMocapData(void);

int Pos_PEL(const float *Qlb_19x1, float *pPC_3x1);
int Att_PEL(const float *Qlb_19x1, float *qPEL_4x1);

int Jac_RF(const float *Qlb_19x1, float **jRFlb_6x18);
int Pos_RF(const float *Qlb_19x1, float *pRFC_3x1);	// right foot contact point
int Att_RF(const float *Qlb_19x1, float *qRF_4x1);
int JacPosAtt_RF(const float *Qlb_19x1, float **jRFlb_6x18, float *pRFC_3x1, float *qRF_4x1);


int Jac_LF(const float *Qlb_19x1, float **jLFlb_6x18);
int Pos_LF(const float *Qlb_19x1, float *pLFC_3x1);	// left foot contact point
int Att_LF(const float *Qlb_19x1, float *qLF_4x1);
int JacPosAtt_LF(const float *Qlb_19x1, float **jLFlb_6x18, float *pLFC_3x1, float *qLF_4x1);

int Jac_COM(const float *Qlb_19x1, const float *Qub_11x1, float **jCOMlb_3x18, float **jCOMub_3x11);	// Jacobian for the lower body(jCOMlb), Jacobian for the upper body(jCOMub)
int Pos_COM(const float *Qlb_19x1, const float *Qub_11x1, float *pCOM_3x1);


int MocapKine(const float *Qlb_19x1, const float *Qub_11x1,	// inputs
				float *pPC_3x1, float *qPEL_4x1,	// outputs
				float **jRFlb_6x18, float *pRFC_3x1, float *qRF_4x1,// outputs
				float **jLFlb_6x18, float *pLFC_3x1, float *qLF_4x1,// outputs
				float **jCOMlb_3x18, float **jCOMub_3x11, float *pCOM_3x1);// outputs


int QT2DC(const float *QT_4x1, float **DC_3x3);		// convert a quaternion to a direction cosine matrix
int DC2QT(const float **DC_3x3, float *QT_4x1);		// convert a direction cosine matrix to a quaternion
int QTdel(const float *des_QT_4x1, const float *QT_4x1, float *result_3x1); // delta quaternion
int Wmatrix(const float *w_3x1, float **wmatrix_4x4);
int QThat(const float *QT_4x1, float **qthat_4x3);
int Crsmatrix(const float *vector_3x1, float **matrix_3x3);	// skew-symmetric cross product matrix

int RX(float theta, float **R_3x3);
int RY(float theta, float **R_3x3);
int RZ(float theta, float **R_3x3);

int RX(float theta, float **R_3x3);
int RY(float theta, float **R_3x3);
int RZ(float theta, float **R_3x3);

int JAVLC(float x, float xp, float xpp_ref, float xpmax, float xppmax, float xmin, float xmax, float margin, float *result_);	// Joint-range, acc and vel limit controller
int ZLC(float Cx, float Cxp, float Cxpp_ref, float Cz, float Czpp, float ZMPxmin, float ZMPxmax, float ZMPz, float margin, float *result_);	// ZMP-lmited COM controller

int MocapOnestep(unsigned int n);
int UpdatePassiveCoord_DSP(void);

void wberror(char error_text[]);

int derive3(float x0, float x1, float x2, float *xd1, float *xdd1, float dt);

int JW_motion_window(float wst, float rsp, float rsr, float rsy, float reb, float rwy, float rwp,
				     float lsp, float lsr, float lsy, float leb, float lwy, float lwp,
					 float dNKY, float dNK1, float dNK2,
					 float dRF1, float dRF2, float dRF3, float dRF4, float dRF5,
					 float dLF1, float dLF2, float dLF3, float dLF4, float dLF5,
					 float body_pitch);
int MocapOnestep_JW(void);

int InitQubWindow(float wst, float rsp, float rsr, float rsy, float reb, float rwy, float rwp,
				     float lsp, float lsr, float lsy, float leb, float lwy, float lwp,
					 float dNKY, float dNK1, float dNK2,
					 float dRF1, float dRF2, float dRF3, float dRF4, float dRF5,
					 float dLF1, float dLF2, float dLF3, float dLF4, float dLF5,
					 float body_pitch);

#endif

