#include <windows.h>
#include <stdio.h>
//#include <stdlib.h>
#include <math.h>
#include "mocap_functions.h"
#include "nrutil.h"
#include "CommonDefinition.h"



extern JOINT Joint[NO_OF_JOINT];		// Joint struct variable

const float _AXIS_X[4] = {0., 1.f, 0., 0.};
const float _AXIS_Y[4] = {0., 0., 1.f, 0.};
const float _AXIS_Z[4] = {0., 0., 0., 1.f};


//------------- Constants for link geometries
const float _LINK_LEG[4] = {0., 0., 0., -L_LEG};
const float _LINK_RPEL[4] = {0., 0., -L_PEL*0.5f, 0.};
const float _LINK_LPEL[4] = {0., 0., L_PEL*0.5f, 0.};
const float _LINK_FOOT[4] = {0., 0., 0., -L_FOOT};
const float _LINK_PEL[4] =  {0., 0., 0., L_PEL2};
const float _LINK_RSHLD[4] =  {0., 0., -LY_SHLD*0.5f, LZ_SHLD};
const float _LINK_LSHLD[4] =  {0., 0., LY_SHLD*0.5f, LZ_SHLD};
const float _LINK_UARM[4] =   {0., LX_UARM, 0., -LZ_UARM};
//-------------


//------------- Local coordinates of mass centers 
const float _C_PEL[4] = {0., CX_PEL, CY_PEL, CZ_PEL};
const float _C_RUL[4] = {0., CX_ULEG, CY_ULEG, CZ_ULEG};
const float _C_RLL[4] = {0., CX_LLEG, CY_LLEG, CZ_LLEG};
const float _C_RF[4] = {0., CX_FOOT, CY_FOOT, CZ_FOOT};
const float _C_LUL[4] = {0., CX_ULEG, -CY_ULEG, CZ_ULEG};
const float _C_LLL[4] = {0., CX_LLEG, -CY_LLEG, CZ_LLEG};
const float _C_LF[4] = {0., CX_FOOT, -CY_FOOT, CZ_FOOT};
const float _C_TOR[4] = {0., CX_TOR, CY_TOR, CZ_TOR};
const float _C_RUA[4] = {0., CX_UARM, CY_UARM, CZ_UARM};
const float _C_RLA[4] = {0., CX_LARM, CY_LARM, CZ_LARM};
const float _C_LUA[4] = {0., CX_UARM, -CY_UARM, CZ_UARM};
const float _C_LLA[4] = {0., CX_LARM, -CY_LARM, CZ_LARM};
//--------------



//-------------- For kinematics
float **_Rz_RHY_3x3, **_Rx_RHR_3x3, **_Ry_RHP_3x3, **_Ry_RKN_3x3, **_Ry_RAP_3x3, **_Rx_RAR_3x3;
float **_Rz_LHY_3x3, **_Rx_LHR_3x3, **_Ry_LHP_3x3, **_Ry_LKN_3x3, **_Ry_LAP_3x3, **_Rx_LAR_3x3;
float **_Rz_WST_3x3, **_Ry_RSP_3x3, **_Rx_RSR_3x3, **_Rz_RSY_3x3, **_Ry_REP_3x3, **_Rz_REY_3x3;
float **_Ry_LSP_3x3, **_Rx_LSR_3x3, **_Rz_LSY_3x3, **_Ry_LEP_3x3, **_Rz_LEY_3x3;
float **_dcPEL_3x3, **_dcRUL_3x3, **_dcRLL_3x3, **_dcRF_3x3;
float **_dcLUL_3x3, **_dcLLL_3x3, **_dcLF_3x3;
float **_dcTOR_3x3, **_dcRUA_3x3, **_dcRLA_3x3;
float **_dcLUA_3x3, **_dcLLA_3x3;
float **_TEMP1_18x18, **_TEMP2_18x18, **_TEMP3_18x18, **_TEMP4_18x18;
float **_EYE_18;
float **_Wmat_4x4, **_Wpmat_4x4;

float **_jPELlb_att_3x18;
float **_jRFlb_6x18;
float **_jLFlb_6x18;
float **_jCOMlb_3x18, **_jCOMub_3x11;

float **_jRFlb_old_6x18;
float **_jLFlb_old_6x18;
float **_jCOMlb_old_3x18, **_jCOMub_old_3x11;

float **_jRFlbp_6x18;
float **_jLFlbp_6x18;
float **_jCOMlbp_3x18, **_jCOMubp_3x11;

float **_jAUG_18x18;
float **_jAUGi_18x18;	// inverse of _jAUG_18x18

float **_jCONST_DSPi_18x14, **_NCONST_DSP_18x18;		// inverse Jacobian for the constraint of DSP
float **_jTASK_18x18, **_jTASKN_18x18, **_jTASKNi_18x18;		// Jtask * Nconst
//---------------


//--------------------- For motion capture data
unsigned int _mocap_count;
unsigned int _MOTION_LENGTH;
float *_des_PCz, **_des_qPEL, *_des_WST, *_des_RSP, *_des_RSR, *_des_RSY, *_des_REP, *_des_REY, *_des_LSP, *_des_LSR, *_des_LSY, *_des_LEP, *_des_LEY;
float *_des_PCzp, **_des_wPEL, *_des_WSTp, *_des_RSPp, *_des_RSRp, *_des_RSYp, *_des_REPp, *_des_REYp, *_des_LSPp, *_des_LSRp, *_des_LSYp, *_des_LEPp, *_des_LEYp;
float *_des_PCzpp, *_des_WSTpp, *_des_RSPpp, *_des_RSRpp, *_des_RSYpp, *_des_REPpp, *_des_REYpp, *_des_LSPpp, *_des_LSRpp, *_des_LSYpp, *_des_LEPpp, *_des_LEYpp;
//--------------------- 


//--------------------- Joint variables
float _Qlb_19x1[20];	// [x,y,z,qPEL[4],rhy,rhr,rhp,rkn,rap,rar,lhy,lhr,lhp,lkn,lap,lar]
float _Qlbp_18x1[19];	// [x,y,z,wPEL[3],rhy,rhr,rhp,rkn,rap,rar,lhy,lhr,lhp,lkn,lap,lar]
float _Qub_11x1[12], _Qubp_11x1[12];	// [wst, rsp,rsr,rsy,rep,rey, lsp,lsr,lsy,lep,ley]

float _Qub0_11x1[12];	// for recovery
float _pPCz0;
float _pCOM0_3x1[4];
//---------------------

//--------------------- For JW's motion capture data
float _RWP, _RWPp, _LWP, _LWPp;
QUB_WIND _Qub_window;
//---------------------


int InitGlobalMotionVariables(void)
{
	int i,j;

	_MOTION_LENGTH = 0;
	_mocap_count = 0;

	_jPELlb_att_3x18 = matrix(1,3,1,18);
	_jRFlb_6x18 = matrix(1,6,1,18);
	_jLFlb_6x18 = matrix(1,6,1,18);
	_jCOMlb_3x18 = matrix(1,3,1,18);
	_jCOMub_3x11 = matrix(1,3,1,11);

	_jRFlb_old_6x18 = matrix(1,6,1,18);
	_jLFlb_old_6x18 = matrix(1,6,1,18);
	_jCOMlb_old_3x18 = matrix(1,3,1,18);
	_jCOMub_old_3x11 = matrix(1,3,1,11);

	_jRFlbp_6x18 = matrix(1,6,1,18);
	_jLFlbp_6x18 = matrix(1,6,1,18);
	_jCOMlbp_3x18 = matrix(1,3,1,18);
	_jCOMubp_3x11 = matrix(1,3,1,11);

	_jAUG_18x18 = matrix(1,18,1,18);
	_jAUGi_18x18 = matrix(1,18,1,18);
	_jAUGi_18x18 = matrix(1,18,1,18);
	_jCONST_DSPi_18x14 = matrix(1,18,1,14);
	_NCONST_DSP_18x18 = matrix(1,18, 1,18);
	_jTASK_18x18 = matrix(1,18,1,18);
	_jTASKN_18x18 = matrix(1,18,1,18);
	_jTASKNi_18x18 = matrix(1,18,1,18);

	_Rz_RHY_3x3 = matrix(1,3,1,3);
	_Rx_RHR_3x3 = matrix(1,3,1,3);
	_Ry_RHP_3x3 = matrix(1,3,1,3);
	_Ry_RKN_3x3 = matrix(1,3,1,3);
	_Ry_RAP_3x3 = matrix(1,3,1,3);
	_Rx_RAR_3x3 = matrix(1,3,1,3);

	_Rz_LHY_3x3 = matrix(1,3,1,3);
	_Rx_LHR_3x3 = matrix(1,3,1,3);
	_Ry_LHP_3x3 = matrix(1,3,1,3);
	_Ry_LKN_3x3 = matrix(1,3,1,3);
	_Ry_LAP_3x3 = matrix(1,3,1,3);
	_Rx_LAR_3x3 = matrix(1,3,1,3);

	_Rz_WST_3x3 = matrix(1,3,1,3);
	_Ry_RSP_3x3 = matrix(1,3,1,3);
	_Rx_RSR_3x3 = matrix(1,3,1,3);
	_Rz_RSY_3x3 = matrix(1,3,1,3);
	_Ry_REP_3x3 = matrix(1,3,1,3);
	_Rz_REY_3x3 = matrix(1,3,1,3);

	_Ry_LSP_3x3 = matrix(1,3,1,3);
	_Rx_LSR_3x3 = matrix(1,3,1,3);
	_Rz_LSY_3x3 = matrix(1,3,1,3);
	_Ry_LEP_3x3 = matrix(1,3,1,3);
	_Rz_LEY_3x3 = matrix(1,3,1,3);


	_dcPEL_3x3 = matrix(1,3,1,3);
	_dcRUL_3x3 = matrix(1,3,1,3);
	_dcRLL_3x3 = matrix(1,3,1,3);
	_dcRF_3x3 = matrix(1,3,1,3);
	_dcLUL_3x3 = matrix(1,3,1,3);
	_dcLLL_3x3 = matrix(1,3,1,3);
	_dcLF_3x3 = matrix(1,3,1,3);

	_dcTOR_3x3 = matrix(1,3,1,3);
	_dcRUA_3x3 = matrix(1,3,1,3);
	_dcRLA_3x3 = matrix(1,3,1,3);
	_dcLUA_3x3 = matrix(1,3,1,3);
	_dcLLA_3x3 = matrix(1,3,1,3);

	_TEMP1_18x18 = matrix(1,18,1,18);
	_TEMP2_18x18 = matrix(1,18,1,18);
	_TEMP3_18x18 = matrix(1,18,1,18);
	_TEMP4_18x18 = matrix(1,18,1,18);
	_EYE_18 = matrix(1,18,1,18);
	_Wmat_4x4 = matrix(1,4,1,4);
	_Wpmat_4x4 = matrix(1,4,1,4);

	for(i=1; i<=6; i++)
	{
		for(j=1; j<=18; j++)
		{			
			_jRFlb_6x18[i][j] = 0.;
			_jLFlb_6x18[i][j] = 0.;

			_jRFlbp_6x18[i][j] = 0.;
			_jLFlbp_6x18[i][j] = 0.;
		}
	}

	for(i=1; i<=3; i++)
	{
		for(j=1; j<=18; j++)
		{
			_jPELlb_att_3x18[i][j] = 0.;
			_jCOMlb_3x18[i][j] = 0.;
			_jCOMlbp_3x18[i][j] = 0.;
		}

		for(j=1; j<=11; j++)
		{
			_jCOMub_3x11[i][j] = 0.;
			_jCOMubp_3x11[i][j] = 0.;
		}
	}

	for(i=1; i<=18; i++)
	{
		for(j=1; j<=18; j++)
		{
			if(i == j)
				_EYE_18[i][j] = 1.f;
			else
				_EYE_18[i][j] = 0.;	
		}
	}

	_jPELlb_att_3x18[1][4] = 1.f;
	_jPELlb_att_3x18[2][5] = 1.f;
	_jPELlb_att_3x18[3][6] = 1.f;
	
	return 0;
}


int UpdateGlobalMotionVariables(void)
{
	int i;

	subs_m((const float**)_jRFlb_6x18,6,18, _jRFlb_old_6x18);
	subs_m((const float**)_jLFlb_6x18,6,18, _jLFlb_old_6x18);
	subs_m((const float**)_jCOMlb_3x18,3,18, _jCOMlb_old_3x18);
	subs_m((const float**)_jCOMub_3x11,3,11, _jCOMub_old_3x11);

/*
	_Qlb_19x1[1] = 0.;		//x
	_Qlb_19x1[2] = 0.;		//y
	
	_Qlb_19x1[4] = 1.f;		// qPEL0
	_Qlb_19x1[5] = 0.;		// qPEL1
	_Qlb_19x1[6] = 0.;		// qPEL2
	_Qlb_19x1[7] = 0.;		// qPEL3

	_Qlb_19x1[8] = 0.;		// rhy
	_Qlb_19x1[9] = 0.;		//rhr
	_Qlb_19x1[10] = RHP0;	// rhp
	_Qlb_19x1[11] = RKN0;	// rkn
	_Qlb_19x1[12] = RAP0;	// rap
	_Qlb_19x1[13] = 0.;		// rar
	_Qlb_19x1[14] = 0.;		// lhy
	_Qlb_19x1[15] = 0.;		// lhr
	_Qlb_19x1[16] = LHP0;	// lhp
	_Qlb_19x1[17] = LKN0;	// lkn
	_Qlb_19x1[18] = LAP0;	// lap
	_Qlb_19x1[19] = 0.;		// lar

	_Qub_11x1[1] = 0.;		// wst
	_Qub_11x1[2] = 0.;		// rsp
	_Qub_11x1[3] = 0.;		//rsr
	_Qub_11x1[4] = 0.;		// rsy
	_Qub_11x1[5] = REP0;	// rep
	_Qub_11x1[6] = 0.;		// rey
	_Qub_11x1[7] = 0.;		//lsp
	_Qub_11x1[8] = 0.;		// lsr
	_Qub_11x1[9] = 0.;		// lsy
	_Qub_11x1[10] = LEP0;	// lep
	_Qub_11x1[11] = 0.;		// ley

	_Qlb_19x1[3] = (float)(2.f*L_LEG*cos(_Qlb_19x1[10])+L_FOOT); // z
*/

	_Qlb_19x1[8] = Joint[RHY].RefAngleCurrent*D2R;
	_Qlb_19x1[9] = Joint[RHR].RefAngleCurrent*D2R;
	_Qlb_19x1[10] = Joint[RHP].RefAngleCurrent*D2R;
	_Qlb_19x1[11] = Joint[RKN].RefAngleCurrent*D2R;
	_Qlb_19x1[12] = Joint[RAP].RefAngleCurrent*D2R;
	_Qlb_19x1[13] = Joint[RAR].RefAngleCurrent*D2R;
	_Qlb_19x1[14] = Joint[LHY].RefAngleCurrent*D2R;
	_Qlb_19x1[15] = Joint[LHR].RefAngleCurrent*D2R;
	_Qlb_19x1[16] = Joint[LHP].RefAngleCurrent*D2R;
	_Qlb_19x1[17] = Joint[LKN].RefAngleCurrent*D2R;
	_Qlb_19x1[18] = Joint[LAP].RefAngleCurrent*D2R;
	_Qlb_19x1[19] = Joint[LAR].RefAngleCurrent*D2R;

	_Qub_11x1[1] = Joint[WST].RefAngleCurrent*D2R;
	_Qub_11x1[2] = Joint[RSP].RefAngleCurrent*D2R;
	_Qub_11x1[3] = (Joint[RSR].RefAngleCurrent + OFFSET_RSR)*D2R ;
	_Qub_11x1[4] = Joint[RSY].RefAngleCurrent*D2R;
	_Qub_11x1[5] = Joint[REB].RefAngleCurrent*D2R;
	_Qub_11x1[6] = Joint[RWY].RefAngleCurrent*D2R;
	_Qub_11x1[7] = Joint[LSP].RefAngleCurrent*D2R;
	_Qub_11x1[8] = (Joint[LSR].RefAngleCurrent + OFFSET_LSR)*D2R;
	_Qub_11x1[9] = Joint[LSY].RefAngleCurrent*D2R;
	_Qub_11x1[10] = Joint[LEB].RefAngleCurrent*D2R;
	_Qub_11x1[11] = Joint[LWY].RefAngleCurrent*D2R;

	_RWP = Joint[RWP].RefAngleCurrent*D2R;
	_LWP = Joint[LWP].RefAngleCurrent*D2R;

	_Qlbp_18x1[1] = 0.;		// xp
	_Qlbp_18x1[2] = 0.;		// yp
	_Qlbp_18x1[3] = 0.;		// zp
	_Qlbp_18x1[4] = 0.;		// wPELx
	_Qlbp_18x1[5] = 0.;		// wPELy
	_Qlbp_18x1[6] = 0.;		// wPELz
	_Qlbp_18x1[7] = 0.;		// rhyp
	_Qlbp_18x1[8] = 0.;		// rhrp
	_Qlbp_18x1[9] = 0.;		// rhpp
	_Qlbp_18x1[10] = 0.;		// rknp
	_Qlbp_18x1[11] = 0.;		// rapp
	_Qlbp_18x1[12] = 0.;		// rarp
	_Qlbp_18x1[13] = 0.;		// lhyp
	_Qlbp_18x1[14] = 0.;		// lhrp
	_Qlbp_18x1[15] = 0.;		// lhpp
	_Qlbp_18x1[16] = 0.;		// lknp
	_Qlbp_18x1[17] = 0.;		// lapp
	_Qlbp_18x1[18] = 0.;		// larp
	
	_Qubp_11x1[1] = 0.;		// wstp
	_Qubp_11x1[2] = 0.;		// rspp
	_Qubp_11x1[3] = 0.;		// rsrp
	_Qubp_11x1[4] = 0.;		// rsyp
	_Qubp_11x1[5] = 0.;		// repp
	_Qubp_11x1[6] = 0.;		// reyp
	_Qubp_11x1[7] = 0.;		// lspp
	_Qubp_11x1[8] = 0.;		// lsrp
	_Qubp_11x1[9] = 0.;		// lsyp
	_Qubp_11x1[10] = 0.;		// lepp
	_Qubp_11x1[11] = 0.;		// leyp

	_RWPp = 0.;
	_LWPp = 0.;

	for(i=1; i<=11; i++)
		_Qub0_11x1[i] = _Qub_11x1[i];

	return 0;

}

int FreeGlobalMotionVariables(void)
{
	free_matrix(_jPELlb_att_3x18, 1,3,1,18);
	free_matrix(_jRFlb_6x18, 1,6,1,18);
	free_matrix(_jLFlb_6x18, 1,6,1,18);
	free_matrix(_jCOMlb_3x18, 1,3,1,18);
	free_matrix(_jCOMub_3x11, 1,3,1,11);

	free_matrix(_jRFlb_old_6x18, 1,6,1,18);
	free_matrix(_jLFlb_old_6x18, 1,6,1,18);
	free_matrix(_jCOMlb_old_3x18, 1,3,1,18);
	free_matrix(_jCOMub_old_3x11, 1,3,1,11);

	free_matrix(_jRFlbp_6x18, 1,6,1,18);
	free_matrix(_jLFlbp_6x18, 1,6,1,18);
	free_matrix(_jCOMlbp_3x18, 1,3,1,18);
	free_matrix(_jCOMubp_3x11, 1,3,1,11);

	free_matrix(_jAUG_18x18, 1,18,1,18);
	free_matrix(_jAUGi_18x18, 1,18,1,18);
	free_matrix(_jCONST_DSPi_18x14, 1,18,1,14);
	free_matrix(_NCONST_DSP_18x18, 1,18, 1,18);
	free_matrix(_jTASK_18x18, 1,18,1,18);
	free_matrix(_jTASKN_18x18, 1,18,1,18);
	free_matrix(_jTASKNi_18x18, 1,18,1,18);

	free_matrix(_Rz_RHY_3x3,1,3,1,3);
	free_matrix(_Rx_RHR_3x3,1,3,1,3);
	free_matrix(_Ry_RHP_3x3,1,3,1,3);
	free_matrix(_Ry_RKN_3x3,1,3,1,3);
	free_matrix(_Ry_RAP_3x3,1,3,1,3);
	free_matrix(_Rx_RAR_3x3,1,3,1,3);

	free_matrix(_Rz_LHY_3x3,1,3,1,3);
	free_matrix(_Rx_LHR_3x3,1,3,1,3);
	free_matrix(_Ry_LHP_3x3,1,3,1,3);
	free_matrix(_Ry_LKN_3x3,1,3,1,3);
	free_matrix(_Ry_LAP_3x3,1,3,1,3);
	free_matrix(_Rx_LAR_3x3,1,3,1,3);

	free_matrix(_Rz_WST_3x3,1,3,1,3);
	free_matrix(_Ry_RSP_3x3,1,3,1,3);
	free_matrix(_Rx_RSR_3x3,1,3,1,3);
	free_matrix(_Rz_RSY_3x3,1,3,1,3);
	free_matrix(_Ry_REP_3x3,1,3,1,3);
	free_matrix(_Rz_REY_3x3,1,3,1,3);

	free_matrix(_Ry_LSP_3x3,1,3,1,3);
	free_matrix(_Rx_LSR_3x3,1,3,1,3);
	free_matrix(_Rz_LSY_3x3,1,3,1,3);
	free_matrix(_Ry_LEP_3x3,1,3,1,3);
	free_matrix(_Rz_LEY_3x3,1,3,1,3);

	free_matrix(_dcPEL_3x3,1,3,1,3);
	free_matrix(_dcRUL_3x3,1,3,1,3);
	free_matrix(_dcRLL_3x3,1,3,1,3);
	free_matrix(_dcRF_3x3,1,3,1,3);
	free_matrix(_dcLUL_3x3,1,3,1,3);
	free_matrix(_dcLLL_3x3,1,3,1,3);
	free_matrix(_dcLF_3x3,1,3,1,3);

	free_matrix(_dcTOR_3x3,1,3,1,3);
	free_matrix(_dcRUA_3x3,1,3,1,3);
	free_matrix(_dcRLA_3x3,1,3,1,3);
	free_matrix(_dcLUA_3x3,1,3,1,3);
	free_matrix(_dcLLA_3x3,1,3,1,3);

	free_matrix(_TEMP1_18x18,1,18,1,18);
	free_matrix(_TEMP2_18x18,1,18,1,18);
	free_matrix(_TEMP3_18x18,1,18,1,18);
	free_matrix(_TEMP4_18x18,1,18,1,18);
	free_matrix(_EYE_18, 1,18,1,18);
	free_matrix(_Wmat_4x4,1,4,1,4);
	free_matrix(_Wpmat_4x4,1,4,1,4);
	
	return 0;
}


int LoadMocapData(int MocapNo)
{
	unsigned int k;
	float ftemp;
	FILE *fp;
	float s_wst, s_rsp, s_rsr, s_rsy, s_rep, s_rey, s_lsp, s_lsr, s_lsy, s_lep, s_ley;

	switch(MocapNo)
	{
	case 1:
		if ( (fp =fopen( "C:\\Rainbow\\MotionCapture\\mocap_converted1.txt","r")) == NULL )
		{
			wberror("Data file mocap_converted1.txt not found\n");		
			return 1;
		}
		break;
	case 2:
		if ( (fp =fopen( "C:\\Rainbow\\MotionCapture\\mocap_converted2.txt","r")) == NULL )
		{
			wberror("Data file mocap_converted2.txt not found\n");		
			return 1;
		}
		break;
	default:
		wberror("\nThe selected motion is not exist!\n");		
		return 1;
	}	

	while (fgetc(fp) != '\n');

	fscanf(fp,"%d",&_MOTION_LENGTH);
	fgetc(fp);	// move to the next line from the scalar
	fgetc(fp);	// move to the next line from the blank line	

	while (fgetc(fp) != '\n');

	fscanf(fp,"%f",&ftemp);	
	fgetc(fp);	// move to the next line from the scalar
	fgetc(fp);	// move to the next line from the blank line
	
	while (fgetc(fp) != '\n');	
	
	_des_PCz = vector(1,_MOTION_LENGTH);
	_des_qPEL = matrix(1,_MOTION_LENGTH,1,4);
	_des_WST = vector(1,_MOTION_LENGTH);
	_des_RSP = vector(1,_MOTION_LENGTH);
	_des_RSR = vector(1,_MOTION_LENGTH);
	_des_RSY = vector(1,_MOTION_LENGTH);
	_des_REP = vector(1,_MOTION_LENGTH);
	_des_REY = vector(1,_MOTION_LENGTH);
	_des_LSP = vector(1,_MOTION_LENGTH);
	_des_LSR = vector(1,_MOTION_LENGTH);
	_des_LSY = vector(1,_MOTION_LENGTH);
	_des_LEP = vector(1,_MOTION_LENGTH);
	_des_LEY = vector(1,_MOTION_LENGTH);

	_des_PCzp = vector(1,_MOTION_LENGTH);
	_des_wPEL = matrix(1,_MOTION_LENGTH,1,3);
	_des_WSTp = vector(1,_MOTION_LENGTH);
	_des_RSPp = vector(1,_MOTION_LENGTH);
	_des_RSRp = vector(1,_MOTION_LENGTH);
	_des_RSYp = vector(1,_MOTION_LENGTH);
	_des_REPp = vector(1,_MOTION_LENGTH);
	_des_REYp = vector(1,_MOTION_LENGTH);
	_des_LSPp = vector(1,_MOTION_LENGTH);
	_des_LSRp = vector(1,_MOTION_LENGTH);
	_des_LSYp = vector(1,_MOTION_LENGTH);
	_des_LEPp = vector(1,_MOTION_LENGTH);
	_des_LEYp = vector(1,_MOTION_LENGTH);

	_des_PCzpp = vector(1,_MOTION_LENGTH);		
	_des_WSTpp = vector(1,_MOTION_LENGTH);
	_des_RSPpp = vector(1,_MOTION_LENGTH);
	_des_RSRpp = vector(1,_MOTION_LENGTH);
	_des_RSYpp = vector(1,_MOTION_LENGTH);
	_des_REPpp = vector(1,_MOTION_LENGTH);
	_des_REYpp = vector(1,_MOTION_LENGTH);
	_des_LSPpp = vector(1,_MOTION_LENGTH);
	_des_LSRpp = vector(1,_MOTION_LENGTH);
	_des_LSYpp = vector(1,_MOTION_LENGTH);
	_des_LEPpp = vector(1,_MOTION_LENGTH);
	_des_LEYpp = vector(1,_MOTION_LENGTH);	

	for (k=1; k<=_MOTION_LENGTH; k++)
	{
		fscanf(fp,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f "
			,&_des_PCz[k], &_des_qPEL[k][1], &_des_qPEL[k][2], &_des_qPEL[k][3], &_des_qPEL[k][4], &_des_WST[k], &_des_RSP[k], &_des_RSR[k], &_des_RSY[k], &_des_REP[k], &_des_REY[k], &_des_LSP[k], &_des_LSR[k], &_des_LSY[k], &_des_LEP[k], &_des_LEY[k]
			,&_des_PCzp[k], &_des_wPEL[k][1], &_des_wPEL[k][2], &_des_wPEL[k][3], &_des_WSTp[k], &_des_RSPp[k], &_des_RSRp[k], &_des_RSYp[k], &_des_REPp[k], &_des_REYp[k], &_des_LSPp[k], &_des_LSRp[k], &_des_LSYp[k], &_des_LEPp[k], &_des_LEYp[k]
			,&_des_PCzpp[k], &_des_WSTpp[k], &_des_RSPpp[k], &_des_RSRpp[k], &_des_RSYpp[k], &_des_REPpp[k], &_des_REYpp[k], &_des_LSPpp[k], &_des_LSRpp[k], &_des_LSYpp[k], &_des_LEPpp[k], &_des_LEYpp[k]);		
	}

	s_wst = 1.f;
	s_rsp = 0.8f;
	s_rsr = 0.6f;
	s_rsy = 0.8f;
	s_rep = 0.8f;
	s_rey = 1.f;
	s_lsp = 0.8f;
	s_lsr = 0.6f;
	s_lsy = 0.8f;
	s_lep = 0.8f;
	s_ley = 1.f;

	for(k=1; k<= _MOTION_LENGTH; k++)
	{
		_des_WST[k] = s_wst*_des_WST[k];
		_des_RSP[k] = s_rsp*_des_RSP[k];
		_des_RSR[k] = s_rsr*_des_RSR[k];
		_des_RSY[k] = s_rsy*_des_RSY[k];
		_des_REP[k] = s_rep*_des_REP[k];
		_des_REY[k] = s_rey*_des_REY[k];
		_des_LSP[k] = s_lsp*_des_LSP[k];
		_des_LSR[k] = s_lsr*_des_LSR[k];
		_des_LSY[k] = s_lsy*_des_LSY[k];
		_des_LEP[k] = s_lep*_des_LEP[k];
		_des_LEY[k] = s_ley*_des_LEY[k];


		_des_WSTp[k] = s_wst*_des_WSTp[k];
		_des_RSPp[k] = s_rsp*_des_RSPp[k];
		_des_RSRp[k] = s_rsr*_des_RSRp[k];
		_des_RSYp[k] = s_rsy*_des_RSYp[k];
		_des_REPp[k] = s_rep*_des_REPp[k];
		_des_REYp[k] = s_rey*_des_REYp[k];
		_des_LSPp[k] = s_lsp*_des_LSPp[k];
		_des_LSRp[k] = s_lsr*_des_LSRp[k];
		_des_LSYp[k] = s_lsy*_des_LSYp[k];
		_des_LEPp[k] = s_lep*_des_LEPp[k];
		_des_LEYp[k] = s_ley*_des_LEYp[k];

		_des_WSTpp[k] = s_wst*_des_WSTpp[k];
		_des_RSPpp[k] = s_rsp*_des_RSPpp[k];
		_des_RSRpp[k] = s_rsr*_des_RSRpp[k];
		_des_RSYpp[k] = s_rsy*_des_RSYpp[k];
		_des_REPpp[k] = s_rep*_des_REPpp[k];
		_des_REYpp[k] = s_rey*_des_REYpp[k];
		_des_LSPpp[k] = s_lsp*_des_LSPpp[k];
		_des_LSRpp[k] = s_lsr*_des_LSRpp[k];
		_des_LSYpp[k] = s_lsy*_des_LSYpp[k];
		_des_LEPpp[k] = s_lep*_des_LEPpp[k];
		_des_LEYpp[k] = s_ley*_des_LEYpp[k];
	}

	fclose(fp);

	return 0;
}

int ClearMocapData(void)
{
	if(_MOTION_LENGTH != 0)
	{
		free_vector(_des_PCz, 1, _MOTION_LENGTH);
		free_matrix(_des_qPEL, 1, _MOTION_LENGTH, 1, 4);
		free_vector(_des_WST, 1, _MOTION_LENGTH);
		free_vector(_des_RSP, 1, _MOTION_LENGTH);
		free_vector(_des_RSR, 1, _MOTION_LENGTH);
		free_vector(_des_RSY, 1, _MOTION_LENGTH);
		free_vector(_des_REP, 1, _MOTION_LENGTH);
		free_vector(_des_REY, 1, _MOTION_LENGTH);
		free_vector(_des_LSP, 1, _MOTION_LENGTH);
		free_vector(_des_LSR, 1, _MOTION_LENGTH);
		free_vector(_des_LSY, 1, _MOTION_LENGTH);
		free_vector(_des_LEP, 1, _MOTION_LENGTH);
		free_vector(_des_LEY, 1, _MOTION_LENGTH);

		free_vector(_des_PCzp, 1, _MOTION_LENGTH);
		free_matrix(_des_wPEL, 1, _MOTION_LENGTH, 1, 3);
		free_vector(_des_WSTp, 1, _MOTION_LENGTH);
		free_vector(_des_RSPp, 1, _MOTION_LENGTH);
		free_vector(_des_RSRp, 1, _MOTION_LENGTH);
		free_vector(_des_RSYp, 1, _MOTION_LENGTH);
		free_vector(_des_REPp, 1, _MOTION_LENGTH);
		free_vector(_des_REYp, 1, _MOTION_LENGTH);
		free_vector(_des_LSPp, 1, _MOTION_LENGTH);
		free_vector(_des_LSRp, 1, _MOTION_LENGTH);
		free_vector(_des_LSYp, 1, _MOTION_LENGTH);
		free_vector(_des_LEPp, 1, _MOTION_LENGTH);
		free_vector(_des_LEYp, 1, _MOTION_LENGTH);

		free_vector(_des_PCzpp, 1, _MOTION_LENGTH);
		free_vector(_des_WSTpp, 1, _MOTION_LENGTH);
		free_vector(_des_RSPpp, 1, _MOTION_LENGTH);
		free_vector(_des_RSRpp, 1, _MOTION_LENGTH);
		free_vector(_des_RSYpp, 1, _MOTION_LENGTH);
		free_vector(_des_REPpp, 1, _MOTION_LENGTH);
		free_vector(_des_REYpp, 1, _MOTION_LENGTH);
		free_vector(_des_LSPpp, 1, _MOTION_LENGTH);
		free_vector(_des_LSRpp, 1, _MOTION_LENGTH);
		free_vector(_des_LSYpp, 1, _MOTION_LENGTH);
		free_vector(_des_LEPpp, 1, _MOTION_LENGTH);
		free_vector(_des_LEYpp, 1, _MOTION_LENGTH);

		_MOTION_LENGTH = 0;

		return 0;
	}
	else
		return 1;
	
}


int Pos_PEL(const float *Qlb_19x1, float *pPC_3x1)
{
	pPC_3x1[1] = Qlb_19x1[1];
	pPC_3x1[2] = Qlb_19x1[2];
	pPC_3x1[3] = Qlb_19x1[3];

	return 0;
}


int Att_PEL(const float *Qlb_19x1, float *qPEL_4x1)
{
	qPEL_4x1[1] = Qlb_19x1[4];
	qPEL_4x1[2] = Qlb_19x1[5];
	qPEL_4x1[3] = Qlb_19x1[6];
	qPEL_4x1[4] = Qlb_19x1[7];

	return 0;
}

int Jac_RF(const float *Qlb_19x1, float **jRFlb_6x18)
{
	float rhy, rhr, rhp, rkn, rap, rar;		
	
	float axis_rhy[4], axis_rhr[4], axis_rhp[4], axis_rkn[4], axis_rap[4], axis_rar[4];	 
	float pRHIP[4], pRKN[4], pRANK[4], pRFC[4];

	float temp3_3x1[4];
	float temp4_3x1[4];	

	float qPEL[5] = {0., Qlb_19x1[4], Qlb_19x1[5], Qlb_19x1[6], Qlb_19x1[7]};
	float pPC[4] = {0., Qlb_19x1[1], Qlb_19x1[2], Qlb_19x1[3]};

	int i, j;

	for(i=1; i<=6; i++)
		for(j=1; j<=18; j++)
			jRFlb_6x18[i][j] = 0.;

	rhy = Qlb_19x1[8];
	rhr = Qlb_19x1[9];
	rhp = Qlb_19x1[10];
	rkn = Qlb_19x1[11];
	rap = Qlb_19x1[12];
	rar = Qlb_19x1[13];

	QT2DC(qPEL, _dcPEL_3x3);
	RZ(rhy, _Rz_RHY_3x3);
	RX(rhr, _Rx_RHR_3x3);
	RY(rhp, _Ry_RHP_3x3);
	RY(rkn, _Ry_RKN_3x3);
	RY(rap, _Ry_RAP_3x3);
	RX(rar, _Rx_RAR_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcRUL_3x3);

	mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
	
	mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcRF_3x3);

	mult_mv((const float**)(const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRHIP);

	mult_mv((const float**)(const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRHIP,3, temp3_3x1, pRKN);

	mult_mv((const float**)(const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, pRANK);

	mult_mv((const float**)_dcRF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(pRANK,3, temp3_3x1, pRFC);

	mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_rhy);
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_18x18);
	mult_mv((const float**)_TEMP1_18x18,3,3, _AXIS_X, axis_rhr);
	mult_mv((const float**)_dcRUL_3x3,3,3, _AXIS_Y, axis_rhp);
	axis_rkn[1] = axis_rhp[1];
	axis_rkn[2] = axis_rhp[2];
	axis_rkn[3] = axis_rhp[3];
	axis_rap[1] = axis_rhp[1];
	axis_rap[2] = axis_rhp[2];
	axis_rap[3] = axis_rhp[3];
	mult_mv((const float**)_dcRF_3x3,3,3, _AXIS_X, axis_rar);

	jRFlb_6x18[1][1] = 1.f;
	jRFlb_6x18[2][2] = 1.f;
	jRFlb_6x18[3][3] = 1.f;
	jRFlb_6x18[4][4] = 1.f;
	jRFlb_6x18[5][5] = 1.f;
	jRFlb_6x18[6][6] = 1.f;

	diff_vv(pRFC,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][4] = temp4_3x1[1];
	jRFlb_6x18[2][4] = temp4_3x1[2];
	jRFlb_6x18[3][4] = temp4_3x1[3];

	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][5] = temp4_3x1[1];
	jRFlb_6x18[2][5] = temp4_3x1[2];
	jRFlb_6x18[3][5] = temp4_3x1[3];

	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][6] = temp4_3x1[1];
	jRFlb_6x18[2][6] = temp4_3x1[2];
	jRFlb_6x18[3][6] = temp4_3x1[3];

	diff_vv(pRFC,3,pRHIP, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][7] = temp4_3x1[1];
	jRFlb_6x18[2][7] = temp4_3x1[2];
	jRFlb_6x18[3][7] = temp4_3x1[3];
	jRFlb_6x18[4][7] = axis_rhy[1];
	jRFlb_6x18[5][7] = axis_rhy[2];
	jRFlb_6x18[6][7] = axis_rhy[3];

	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][8] = temp4_3x1[1];
	jRFlb_6x18[2][8] = temp4_3x1[2];
	jRFlb_6x18[3][8] = temp4_3x1[3];
	jRFlb_6x18[4][8] = axis_rhr[1];
	jRFlb_6x18[5][8] = axis_rhr[2];
	jRFlb_6x18[6][8] = axis_rhr[3];

	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][9] = temp4_3x1[1];
	jRFlb_6x18[2][9] = temp4_3x1[2];
	jRFlb_6x18[3][9] = temp4_3x1[3];
	jRFlb_6x18[4][9] = axis_rhp[1];
	jRFlb_6x18[5][9] = axis_rhp[2];
	jRFlb_6x18[6][9] = axis_rhp[3];

	diff_vv(pRFC,3,pRKN, temp3_3x1);
	cross(1.f,axis_rkn, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][10] = temp4_3x1[1];
	jRFlb_6x18[2][10] = temp4_3x1[2];
	jRFlb_6x18[3][10] = temp4_3x1[3];
	jRFlb_6x18[4][10] = axis_rkn[1];
	jRFlb_6x18[5][10] = axis_rkn[2];
	jRFlb_6x18[6][10] = axis_rkn[3];

	diff_vv(pRFC,3,pRANK, temp3_3x1);
	cross(1.f,axis_rap, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][11] = temp4_3x1[1];
	jRFlb_6x18[2][11] = temp4_3x1[2];
	jRFlb_6x18[3][11] = temp4_3x1[3];
	jRFlb_6x18[4][11] = axis_rap[1];
	jRFlb_6x18[5][11] = axis_rap[2];
	jRFlb_6x18[6][11] = axis_rap[3];

	cross(1.f,axis_rar, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][12] = temp4_3x1[1];
	jRFlb_6x18[2][12] = temp4_3x1[2];
	jRFlb_6x18[3][12] = temp4_3x1[3];
	jRFlb_6x18[4][12] = axis_rar[1];
	jRFlb_6x18[5][12] = axis_rar[2];
	jRFlb_6x18[6][12] = axis_rar[3];

	return 0;
}


int Pos_RF(const float *Qlb_19x1, float *pRFC_3x1)
{
	float rhy, rhr, rhp, rkn, rap, rar;
	
	float pRHIP[4], pRKN[4], pRANK[4];

	float temp3_3x1[4];

	float qPEL[5] = {0., Qlb_19x1[4], Qlb_19x1[5], Qlb_19x1[6], Qlb_19x1[7]};
	float pPC[4] = {0., Qlb_19x1[1], Qlb_19x1[2], Qlb_19x1[3]};

	rhy = Qlb_19x1[8];
	rhr = Qlb_19x1[9];
	rhp = Qlb_19x1[10];
	rkn = Qlb_19x1[11];
	rap = Qlb_19x1[12];
	rar = Qlb_19x1[13];

	QT2DC(qPEL, _dcPEL_3x3);
	RZ(rhy, _Rz_RHY_3x3);
	RX(rhr, _Rx_RHR_3x3);
	RY(rhp, _Ry_RHP_3x3);
	RY(rkn, _Ry_RKN_3x3);
	RY(rap, _Ry_RAP_3x3);
	RX(rar, _Rx_RAR_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcRUL_3x3);

	mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
	
	mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcRF_3x3);

	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRHIP);

	mult_mv((const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRHIP,3, temp3_3x1, pRKN);

	mult_mv((const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, pRANK);

	mult_mv((const float**)_dcRF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(pRANK,3, temp3_3x1, pRFC_3x1);

	return 0;
}


int Att_RF(const float *Qlb_19x1, float *qRF_4x1)
{
	float rhy, rhr, rhp, rkn, rap, rar;			
	float qPEL[5] = {0., Qlb_19x1[4], Qlb_19x1[5], Qlb_19x1[6], Qlb_19x1[7]};

	rhy = Qlb_19x1[8];
	rhr = Qlb_19x1[9];
	rhp = Qlb_19x1[10];
	rkn = Qlb_19x1[11];
	rap = Qlb_19x1[12];
	rar = Qlb_19x1[13];

	QT2DC(qPEL, _dcPEL_3x3);
	RZ(rhy, _Rz_RHY_3x3);
	RX(rhr, _Rx_RHR_3x3);
	RY(rhp, _Ry_RHP_3x3);
	RY(rkn, _Ry_RKN_3x3);
	RY(rap, _Ry_RAP_3x3);
	RX(rar, _Rx_RAR_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcRUL_3x3);

	mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
	
	mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcRF_3x3);

	DC2QT((const float**)_dcRF_3x3, qRF_4x1);

	return 0;
}


int JacPosAtt_RF(const float *Qlb_19x1, float **jRFlb_6x18, float *pRFC_3x1, float *qRF_4x1)
{
	float rhy, rhr, rhp, rkn, rap, rar;		
	
	float axis_rhy[4], axis_rhr[4], axis_rhp[4], axis_rkn[4], axis_rap[4], axis_rar[4];	 
	float pRHIP[4], pRKN[4], pRANK[4];

	float temp3_3x1[4];
	float temp4_3x1[4];	

	float qPEL[5] = {0., Qlb_19x1[4], Qlb_19x1[5], Qlb_19x1[6], Qlb_19x1[7]};
	float pPC[4] = {0., Qlb_19x1[1], Qlb_19x1[2], Qlb_19x1[3]};


	int i, j;

	for(i=1; i<=6; i++)
		for(j=1; j<=18; j++)
			jRFlb_6x18[i][j] = 0.;

	rhy = Qlb_19x1[8];
	rhr = Qlb_19x1[9];
	rhp = Qlb_19x1[10];
	rkn = Qlb_19x1[11];
	rap = Qlb_19x1[12];
	rar = Qlb_19x1[13];

	QT2DC(qPEL, _dcPEL_3x3);
	RZ(rhy, _Rz_RHY_3x3);
	RX(rhr, _Rx_RHR_3x3);
	RY(rhp, _Ry_RHP_3x3);
	RY(rkn, _Ry_RKN_3x3);
	RY(rap, _Ry_RAP_3x3);
	RX(rar, _Rx_RAR_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcRUL_3x3);

	mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
	
	mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcRF_3x3);
	DC2QT((const float**)_dcRF_3x3, qRF_4x1);

	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRHIP);

	mult_mv((const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRHIP,3, temp3_3x1, pRKN);

	mult_mv((const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, pRANK);

	mult_mv((const float**)_dcRF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(pRANK,3, temp3_3x1, pRFC_3x1);

	mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_rhy);
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_18x18);
	mult_mv((const float**)_TEMP1_18x18,3,3, _AXIS_X, axis_rhr);
	mult_mv((const float**)_dcRUL_3x3,3,3, _AXIS_Y, axis_rhp);
	axis_rkn[1] = axis_rhp[1];
	axis_rkn[2] = axis_rhp[2];
	axis_rkn[3] = axis_rhp[3];
	axis_rap[1] = axis_rhp[1];
	axis_rap[2] = axis_rhp[2];
	axis_rap[3] = axis_rhp[3];
	mult_mv((const float**)_dcRF_3x3,3,3, _AXIS_X, axis_rar);

	jRFlb_6x18[1][1] = 1.f;
	jRFlb_6x18[2][2] = 1.f;
	jRFlb_6x18[3][3] = 1.f;
	jRFlb_6x18[4][4] = 1.f;
	jRFlb_6x18[5][5] = 1.f;
	jRFlb_6x18[6][6] = 1.f;

	diff_vv(pRFC_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][4] = temp4_3x1[1];
	jRFlb_6x18[2][4] = temp4_3x1[2];
	jRFlb_6x18[3][4] = temp4_3x1[3];

	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][5] = temp4_3x1[1];
	jRFlb_6x18[2][5] = temp4_3x1[2];
	jRFlb_6x18[3][5] = temp4_3x1[3];

	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][6] = temp4_3x1[1];
	jRFlb_6x18[2][6] = temp4_3x1[2];
	jRFlb_6x18[3][6] = temp4_3x1[3];

	diff_vv(pRFC_3x1,3,pRHIP, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][7] = temp4_3x1[1];
	jRFlb_6x18[2][7] = temp4_3x1[2];
	jRFlb_6x18[3][7] = temp4_3x1[3];
	jRFlb_6x18[4][7] = axis_rhy[1];
	jRFlb_6x18[5][7] = axis_rhy[2];
	jRFlb_6x18[6][7] = axis_rhy[3];

	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][8] = temp4_3x1[1];
	jRFlb_6x18[2][8] = temp4_3x1[2];
	jRFlb_6x18[3][8] = temp4_3x1[3];
	jRFlb_6x18[4][8] = axis_rhr[1];
	jRFlb_6x18[5][8] = axis_rhr[2];
	jRFlb_6x18[6][8] = axis_rhr[3];

	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][9] = temp4_3x1[1];
	jRFlb_6x18[2][9] = temp4_3x1[2];
	jRFlb_6x18[3][9] = temp4_3x1[3];
	jRFlb_6x18[4][9] = axis_rhp[1];
	jRFlb_6x18[5][9] = axis_rhp[2];
	jRFlb_6x18[6][9] = axis_rhp[3];

	diff_vv(pRFC_3x1,3,pRKN, temp3_3x1);
	cross(1.f,axis_rkn, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][10] = temp4_3x1[1];
	jRFlb_6x18[2][10] = temp4_3x1[2];
	jRFlb_6x18[3][10] = temp4_3x1[3];
	jRFlb_6x18[4][10] = axis_rkn[1];
	jRFlb_6x18[5][10] = axis_rkn[2];
	jRFlb_6x18[6][10] = axis_rkn[3];

	diff_vv(pRFC_3x1,3,pRANK, temp3_3x1);
	cross(1.f,axis_rap, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][11] = temp4_3x1[1];
	jRFlb_6x18[2][11] = temp4_3x1[2];
	jRFlb_6x18[3][11] = temp4_3x1[3];
	jRFlb_6x18[4][11] = axis_rap[1];
	jRFlb_6x18[5][11] = axis_rap[2];
	jRFlb_6x18[6][11] = axis_rap[3];

	cross(1.f,axis_rar, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][12] = temp4_3x1[1];
	jRFlb_6x18[2][12] = temp4_3x1[2];
	jRFlb_6x18[3][12] = temp4_3x1[3];
	jRFlb_6x18[4][12] = axis_rar[1];
	jRFlb_6x18[5][12] = axis_rar[2];
	jRFlb_6x18[6][12] = axis_rar[3];

	return 0;
}


int Jac_LF(const float *Qlb_19x1, float **jLFlb_6x18)
{
	float lhy, lhr, lhp, lkn, lap, lar;		
	
	float axis_lhy[4], axis_lhr[4], axis_lhp[4], axis_lkn[4], axis_lap[4], axis_lar[4];	 
	float pLHIP[4], pLKN[4], pLANK[4], pLFC[4];

	float temp3_3x1[4];
	float temp4_3x1[4];	

	float qPEL[5] = {0., Qlb_19x1[4], Qlb_19x1[5], Qlb_19x1[6], Qlb_19x1[7]};
	float pPC[4] = {0., Qlb_19x1[1], Qlb_19x1[2], Qlb_19x1[3]};

	int i, j;

	for(i=1; i<=6; i++)
		for(j=1; j<=18; j++)
			jLFlb_6x18[i][j] = 0.;

	lhy = Qlb_19x1[14];
	lhr = Qlb_19x1[15];
	lhp = Qlb_19x1[16];
	lkn = Qlb_19x1[17];
	lap = Qlb_19x1[18];
	lar = Qlb_19x1[19];

	QT2DC(qPEL, _dcPEL_3x3);
	RZ(lhy, _Rz_LHY_3x3);
	RX(lhr, _Rx_LHR_3x3);
	RY(lhp, _Ry_LHP_3x3);
	RY(lkn, _Ry_LKN_3x3);
	RY(lap, _Ry_LAP_3x3);
	RX(lar, _Rx_LAR_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcLUL_3x3);

	mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
	
	mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcLF_3x3);

	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLHIP);

	mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLHIP,3, temp3_3x1, pLKN);

	mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, pLANK);

	mult_mv((const float**)_dcLF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(pLANK,3, temp3_3x1, pLFC);

	mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_lhy);
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_18x18);
	mult_mv((const float**)_TEMP1_18x18,3,3, _AXIS_X, axis_lhr);
	mult_mv((const float**)_dcLUL_3x3,3,3, _AXIS_Y, axis_lhp);
	axis_lkn[1] = axis_lhp[1];
	axis_lkn[2] = axis_lhp[2];
	axis_lkn[3] = axis_lhp[3];
	axis_lap[1] = axis_lhp[1];
	axis_lap[2] = axis_lhp[2];
	axis_lap[3] = axis_lhp[3];
	mult_mv((const float**)_dcLF_3x3,3,3, _AXIS_X, axis_lar);

	jLFlb_6x18[1][1] = 1.f;
	jLFlb_6x18[2][2] = 1.f;
	jLFlb_6x18[3][3] = 1.f;
	jLFlb_6x18[4][4] = 1.f;
	jLFlb_6x18[5][5] = 1.f;
	jLFlb_6x18[6][6] = 1.f;

	diff_vv(pLFC,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][4] = temp4_3x1[1];
	jLFlb_6x18[2][4] = temp4_3x1[2];
	jLFlb_6x18[3][4] = temp4_3x1[3];

	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][5] = temp4_3x1[1];
	jLFlb_6x18[2][5] = temp4_3x1[2];
	jLFlb_6x18[3][5] = temp4_3x1[3];

	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][6] = temp4_3x1[1];
	jLFlb_6x18[2][6] = temp4_3x1[2];
	jLFlb_6x18[3][6] = temp4_3x1[3];

	diff_vv(pLFC,3,pLHIP, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][13] = temp4_3x1[1];
	jLFlb_6x18[2][13] = temp4_3x1[2];
	jLFlb_6x18[3][13] = temp4_3x1[3];
	jLFlb_6x18[4][13] = axis_lhy[1];
	jLFlb_6x18[5][13] = axis_lhy[2];
	jLFlb_6x18[6][13] = axis_lhy[3];

	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][14] = temp4_3x1[1];
	jLFlb_6x18[2][14] = temp4_3x1[2];
	jLFlb_6x18[3][14] = temp4_3x1[3];
	jLFlb_6x18[4][14] = axis_lhr[1];
	jLFlb_6x18[5][14] = axis_lhr[2];
	jLFlb_6x18[6][14] = axis_lhr[3];

	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][15] = temp4_3x1[1];
	jLFlb_6x18[2][15] = temp4_3x1[2];
	jLFlb_6x18[3][15] = temp4_3x1[3];
	jLFlb_6x18[4][15] = axis_lhp[1];
	jLFlb_6x18[5][15] = axis_lhp[2];
	jLFlb_6x18[6][15] = axis_lhp[3];

	diff_vv(pLFC,3,pLKN, temp3_3x1);
	cross(1.f,axis_lkn, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][16] = temp4_3x1[1];
	jLFlb_6x18[2][16] = temp4_3x1[2];
	jLFlb_6x18[3][16] = temp4_3x1[3];
	jLFlb_6x18[4][16] = axis_lkn[1];
	jLFlb_6x18[5][16] = axis_lkn[2];
	jLFlb_6x18[6][16] = axis_lkn[3];

	diff_vv(pLFC,3,pLANK, temp3_3x1);
	cross(1.f,axis_lap, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][17] = temp4_3x1[1];
	jLFlb_6x18[2][17] = temp4_3x1[2];
	jLFlb_6x18[3][17] = temp4_3x1[3];
	jLFlb_6x18[4][17] = axis_lap[1];
	jLFlb_6x18[5][17] = axis_lap[2];
	jLFlb_6x18[6][17] = axis_lap[3];

	cross(1.f,axis_lar, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][18] = temp4_3x1[1];
	jLFlb_6x18[2][18] = temp4_3x1[2];
	jLFlb_6x18[3][18] = temp4_3x1[3];
	jLFlb_6x18[4][18] = axis_lar[1];
	jLFlb_6x18[5][18] = axis_lar[2];
	jLFlb_6x18[6][18] = axis_lar[3];

	return 0;
}


int Pos_LF(const float *Qlb_19x1, float *pLFC_3x1)
{
	float lhy, lhr, lhp, lkn, lap, lar;			
	float pLHIP[4], pLKN[4], pLANK[4];
	float temp3_3x1[4];

	float qPEL[5] = {0., Qlb_19x1[4], Qlb_19x1[5], Qlb_19x1[6], Qlb_19x1[7]};
	float pPC[4] = {0., Qlb_19x1[1], Qlb_19x1[2], Qlb_19x1[3]};


	lhy = Qlb_19x1[14];
	lhr = Qlb_19x1[15];
	lhp = Qlb_19x1[16];
	lkn = Qlb_19x1[17];
	lap = Qlb_19x1[18];
	lar = Qlb_19x1[19];

	QT2DC(qPEL, _dcPEL_3x3);
	RZ(lhy, _Rz_LHY_3x3);
	RX(lhr, _Rx_LHR_3x3);
	RY(lhp, _Ry_LHP_3x3);
	RY(lkn, _Ry_LKN_3x3);
	RY(lap, _Ry_LAP_3x3);
	RX(lar, _Rx_LAR_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcLUL_3x3);

	mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
	
	mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcLF_3x3);

	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLHIP);

	mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLHIP,3, temp3_3x1, pLKN);

	mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, pLANK);

	mult_mv((const float**)_dcLF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(pLANK,3, temp3_3x1, pLFC_3x1);

	return 0;
}


int Att_LF(const float *Qlb_19x1, float *qLF_4x1)
{
	float lhy, lhr, lhp, lkn, lap, lar;			

	float qPEL[5] = {0., Qlb_19x1[4], Qlb_19x1[5], Qlb_19x1[6], Qlb_19x1[7]};
	
	lhy = Qlb_19x1[14];
	lhr = Qlb_19x1[15];
	lhp = Qlb_19x1[16];
	lkn = Qlb_19x1[17];
	lap = Qlb_19x1[18];
	lar = Qlb_19x1[19];

	QT2DC(qPEL, _dcPEL_3x3);
	RZ(lhy, _Rz_LHY_3x3);
	RX(lhr, _Rx_LHR_3x3);
	RY(lhp, _Ry_LHP_3x3);
	RY(lkn, _Ry_LKN_3x3);
	RY(lap, _Ry_LAP_3x3);
	RX(lar, _Rx_LAR_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcLUL_3x3);

	mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
	
	mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcLF_3x3);

	DC2QT((const float**)_dcLF_3x3, qLF_4x1);

	return 0;
}


int JacPosAtt_LF(const float *Qlb_19x1, float **jLFlb_6x18, float *pLFC_3x1, float *qLF_4x1)
{
	float lhy, lhr, lhp, lkn, lap, lar;		
	
	float axis_lhy[4], axis_lhr[4], axis_lhp[4], axis_lkn[4], axis_lap[4], axis_lar[4];	 
	float pLHIP[4], pLKN[4], pLANK[4];

	float temp3_3x1[4];
	float temp4_3x1[4];	

	float qPEL[5] = {0., Qlb_19x1[4], Qlb_19x1[5], Qlb_19x1[6], Qlb_19x1[7]};
	float pPC[4] = {0., Qlb_19x1[1], Qlb_19x1[2], Qlb_19x1[3]};


	int i, j;

	for(i=1; i<=6; i++)
		for(j=1; j<=18; j++)
			jLFlb_6x18[i][j] = 0.;

	lhy = Qlb_19x1[14];
	lhr = Qlb_19x1[15];
	lhp = Qlb_19x1[16];
	lkn = Qlb_19x1[17];
	lap = Qlb_19x1[18];
	lar = Qlb_19x1[19];

	QT2DC(qPEL, _dcPEL_3x3);
	RZ(lhy, _Rz_LHY_3x3);
	RX(lhr, _Rx_LHR_3x3);
	RY(lhp, _Ry_LHP_3x3);
	RY(lkn, _Ry_LKN_3x3);
	RY(lap, _Ry_LAP_3x3);
	RX(lar, _Rx_LAR_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcLUL_3x3);

	mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
	
	mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcLF_3x3);
	DC2QT((const float**)_dcLF_3x3, qLF_4x1);

	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLHIP);

	mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLHIP,3, temp3_3x1, pLKN);

	mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, pLANK);

	mult_mv((const float**)_dcLF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(pLANK,3, temp3_3x1, pLFC_3x1);

	mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_lhy);
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_18x18);
	mult_mv((const float**)_TEMP1_18x18,3,3, _AXIS_X, axis_lhr);
	mult_mv((const float**)_dcLUL_3x3,3,3, _AXIS_Y, axis_lhp);
	axis_lkn[1] = axis_lhp[1];
	axis_lkn[2] = axis_lhp[2];
	axis_lkn[3] = axis_lhp[3];
	axis_lap[1] = axis_lhp[1];
	axis_lap[2] = axis_lhp[2];
	axis_lap[3] = axis_lhp[3];
	mult_mv((const float**)_dcLF_3x3,3,3, _AXIS_X, axis_lar);

	jLFlb_6x18[1][1] = 1.f;
	jLFlb_6x18[2][2] = 1.f;
	jLFlb_6x18[3][3] = 1.f;
	jLFlb_6x18[4][4] = 1.f;
	jLFlb_6x18[5][5] = 1.f;
	jLFlb_6x18[6][6] = 1.f;

	diff_vv(pLFC_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][4] = temp4_3x1[1];
	jLFlb_6x18[2][4] = temp4_3x1[2];
	jLFlb_6x18[3][4] = temp4_3x1[3];

	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][5] = temp4_3x1[1];
	jLFlb_6x18[2][5] = temp4_3x1[2];
	jLFlb_6x18[3][5] = temp4_3x1[3];

	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][6] = temp4_3x1[1];
	jLFlb_6x18[2][6] = temp4_3x1[2];
	jLFlb_6x18[3][6] = temp4_3x1[3];

	diff_vv(pLFC_3x1,3,pLHIP, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][13] = temp4_3x1[1];
	jLFlb_6x18[2][13] = temp4_3x1[2];
	jLFlb_6x18[3][13] = temp4_3x1[3];
	jLFlb_6x18[4][13] = axis_lhy[1];
	jLFlb_6x18[5][13] = axis_lhy[2];
	jLFlb_6x18[6][13] = axis_lhy[3];

	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][14] = temp4_3x1[1];
	jLFlb_6x18[2][14] = temp4_3x1[2];
	jLFlb_6x18[3][14] = temp4_3x1[3];
	jLFlb_6x18[4][14] = axis_lhr[1];
	jLFlb_6x18[5][14] = axis_lhr[2];
	jLFlb_6x18[6][14] = axis_lhr[3];

	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][15] = temp4_3x1[1];
	jLFlb_6x18[2][15] = temp4_3x1[2];
	jLFlb_6x18[3][15] = temp4_3x1[3];
	jLFlb_6x18[4][15] = axis_lhp[1];
	jLFlb_6x18[5][15] = axis_lhp[2];
	jLFlb_6x18[6][15] = axis_lhp[3];

	diff_vv(pLFC_3x1,3,pLKN, temp3_3x1);
	cross(1.f,axis_lkn, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][16] = temp4_3x1[1];
	jLFlb_6x18[2][16] = temp4_3x1[2];
	jLFlb_6x18[3][16] = temp4_3x1[3];
	jLFlb_6x18[4][16] = axis_lkn[1];
	jLFlb_6x18[5][16] = axis_lkn[2];
	jLFlb_6x18[6][16] = axis_lkn[3];

	diff_vv(pLFC_3x1,3,pLANK, temp3_3x1);
	cross(1.f,axis_lap, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][17] = temp4_3x1[1];
	jLFlb_6x18[2][17] = temp4_3x1[2];
	jLFlb_6x18[3][17] = temp4_3x1[3];
	jLFlb_6x18[4][17] = axis_lap[1];
	jLFlb_6x18[5][17] = axis_lap[2];
	jLFlb_6x18[6][17] = axis_lap[3];

	cross(1.f,axis_lar, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][18] = temp4_3x1[1];
	jLFlb_6x18[2][18] = temp4_3x1[2];
	jLFlb_6x18[3][18] = temp4_3x1[3];
	jLFlb_6x18[4][18] = axis_lar[1];
	jLFlb_6x18[5][18] = axis_lar[2];
	jLFlb_6x18[6][18] = axis_lar[3];

	return 0;
}

int Jac_COM(const float *Qlb_19x1, const float *Qub_11x1, float **jCOMlb_3x18, float **jCOMub_3x11)	// Jacobian for the lower body(jCOMlb), Jacobian for the upper body(jCOMub)
{
	float wst, rhy, rhp, rhr, rkn, rap, rar, lhy, lhp, lhr, lkn, lap, lar, rsp, rsr, rsy, rep, rey, lsp, lsr, lsy, lep, ley;
	
	float axis_rhy[4], axis_rhr[4], axis_rhp[4], axis_rkn[4], axis_rap[4], axis_rar[4];	 
	float axis_lhy[4], axis_lhr[4], axis_lhp[4], axis_lkn[4], axis_lap[4], axis_lar[4];	
	float axis_wst[4], axis_rsp[4], axis_rsr[4], axis_rsy[4], axis_rep[4], axis_rey[4];
	float axis_lsp[4], axis_lsr[4], axis_lsy[4], axis_lep[4], axis_ley[4];

	float pRHIP[4], pRKN[4], pRANK[4];
	float pLHIP[4], pLKN[4], pLANK[4];
	float pWST[4], pRSHLD[4], pRELB[4];
	float pLSHLD[4], pLELB[4];

	float cPEL[4], cTOR[4], cRUL[4], cRLL[4], cRF[4], cLUL[4], cLLL[4], cLF[4], cRUA[4], cRLA[4], cLUA[4], cLLA[4];

	float temp3_3x1[4], temp4_3x1[4];

	//--- Jacobian for [xp,yp,zp,wPELx,wPELy,wPELz,wstp,rhyp,rhrp,rhpp,rknp,rapp,rarp,lhyp,lhrp,lhpp,lknp,lapp,larp,rspp,rsrp,rsyp,repp,reyp,lspp,lsrp,lsyp,lepp,leyp]' : 29x1
	float jCOM_PEL_3x29[4][30], jCOM_RUL_3x29[4][30], jCOM_RLL_3x29[4][30], jCOM_RF_3x29[4][30], jCOM_LUL_3x29[4][30], jCOM_LLL_3x29[4][30], jCOM_LF_3x29[4][30];
	float jCOM_TOR_3x29[4][30], jCOM_RUA_3x29[4][30], jCOM_RLA_3x29[4][30], jCOM_LUA_3x29[4][30], jCOM_LLA_3x29[4][30];

	float qPEL[5] = {0., Qlb_19x1[4], Qlb_19x1[5], Qlb_19x1[6], Qlb_19x1[7]};
	float pPC[4] = {0., Qlb_19x1[1], Qlb_19x1[2], Qlb_19x1[3]};

	int i, j;
	float ftemp;
	
	for(i=1; i<=3; i++)
	{
		jCOMlb_3x18[i][1] = 0.;
		jCOMlb_3x18[i][2] = 0.;
		jCOMlb_3x18[i][3] = 0.;

		for(j=1; j<=29; j++)
		{
			jCOM_PEL_3x29[i][j] = 0.;
			jCOM_RUL_3x29[i][j] = 0.;
			jCOM_RLL_3x29[i][j] = 0.;
			jCOM_RF_3x29[i][j] = 0.;
			jCOM_LUL_3x29[i][j] = 0.;
			jCOM_LLL_3x29[i][j] = 0.;
			jCOM_LF_3x29[i][j] = 0.;
			jCOM_TOR_3x29[i][j] = 0.;
			jCOM_RUA_3x29[i][j] = 0.;
			jCOM_RLA_3x29[i][j] = 0.;
			jCOM_LUA_3x29[i][j] = 0.;
			jCOM_LLA_3x29[i][j] = 0.;
		}
	}



	rhy = Qlb_19x1[8];
	rhr = Qlb_19x1[9];
	rhp = Qlb_19x1[10];
	rkn = Qlb_19x1[11];
	rap = Qlb_19x1[12];
	rar = Qlb_19x1[13];

	lhy = Qlb_19x1[14];
	lhr = Qlb_19x1[15];
	lhp = Qlb_19x1[16];
	lkn = Qlb_19x1[17];
	lap = Qlb_19x1[18];
	lar = Qlb_19x1[19];

	wst = Qub_11x1[1];
	rsp = Qub_11x1[2];
	rsr = Qub_11x1[3];
	rsy = Qub_11x1[4];
	rep = Qub_11x1[5];
	rey = Qub_11x1[6];
	lsp = Qub_11x1[7];
	lsr = Qub_11x1[8];
	lsy = Qub_11x1[9];
	lep = Qub_11x1[10];
	ley = Qub_11x1[11];


	QT2DC(qPEL, _dcPEL_3x3);
	
	RZ(rhy, _Rz_RHY_3x3);
	RX(rhr, _Rx_RHR_3x3);
	RY(rhp, _Ry_RHP_3x3);
	RY(rkn, _Ry_RKN_3x3);
	RY(rap, _Ry_RAP_3x3);
	RX(rar, _Rx_RAR_3x3);

	RZ(lhy, _Rz_LHY_3x3);
	RX(lhr, _Rx_LHR_3x3);
	RY(lhp, _Ry_LHP_3x3);
	RY(lkn, _Ry_LKN_3x3);
	RY(lap, _Ry_LAP_3x3);
	RX(lar, _Rx_LAR_3x3);

	RZ(wst, _Rz_WST_3x3);
	
	RY(rsp, _Ry_RSP_3x3);
	RX(rsr, _Rx_RSR_3x3);
	RZ(rsy, _Rz_RSY_3x3);
	RY(rep, _Ry_REP_3x3);
	RZ(rey, _Rz_REY_3x3);

	RY(lsp, _Ry_LSP_3x3);
	RX(lsr, _Rx_LSR_3x3);
	RZ(lsy, _Rz_LSY_3x3);
	RY(lep, _Ry_LEP_3x3);
	RZ(ley, _Rz_LEY_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcRUL_3x3);

	mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
	
	mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcRF_3x3);


	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcLUL_3x3);

	mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
	
	mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcLF_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_WST_3x3,3, _dcTOR_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_RSR_3x3,3,3, (const float**)_Rz_RSY_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcRUA_3x3);

	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REP_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_Rz_REY_3x3,3, _dcRLA_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_LSR_3x3,3,3, (const float**)_Rz_LSY_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcLUA_3x3);

	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEP_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_Rz_LEY_3x3,3, _dcLLA_3x3);

	
	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRHIP);

	mult_mv((const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRHIP,3, temp3_3x1, pRKN);

	mult_mv((const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, pRANK);


	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLHIP);

	mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLHIP,3, temp3_3x1, pLKN);

	mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, pLANK);

	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_PEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pWST);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_RSHLD, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRSHLD);

	mult_mv((const float**)_dcRUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pRSHLD,3, temp3_3x1, pRELB);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_LSHLD, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLSHLD);

	mult_mv((const float**)_dcLUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pLSHLD,3, temp3_3x1, pLELB);

	mult_mv((const float**)_dcPEL_3x3,3,3, _C_PEL, temp3_3x1);
	sum_vv(pPC,3, temp3_3x1, cPEL);

	mult_mv((const float**)_dcTOR_3x3,3,3, _C_TOR, temp3_3x1);
	sum_vv(pWST,3, temp3_3x1, cTOR);

	mult_mv((const float**)_dcRUL_3x3,3,3, _C_RUL, temp3_3x1);
	sum_vv(pRHIP,3, temp3_3x1, cRUL);

	mult_mv((const float**)_dcRLL_3x3,3,3, _C_RLL, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, cRLL);

	mult_mv((const float**)_dcRF_3x3,3,3, _C_RF, temp3_3x1);
	sum_vv(pRANK,3, temp3_3x1, cRF);

	mult_mv((const float**)_dcLUL_3x3,3,3, _C_LUL, temp3_3x1);
	sum_vv(pLHIP,3, temp3_3x1, cLUL);

	mult_mv((const float**)_dcLLL_3x3,3,3, _C_LLL, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, cLLL);

	mult_mv((const float**)_dcLF_3x3,3,3, _C_LF, temp3_3x1);
	sum_vv(pLANK,3, temp3_3x1, cLF);

	mult_mv((const float**)_dcRUA_3x3,3,3, _C_RUA, temp3_3x1);
	sum_vv(pRSHLD,3, temp3_3x1, cRUA);

	mult_mv((const float**)_dcRLA_3x3,3,3, _C_RLA, temp3_3x1);
	sum_vv(pRELB,3, temp3_3x1, cRLA);

	mult_mv((const float**)_dcLUA_3x3,3,3, _C_LUA, temp3_3x1);
	sum_vv(pLSHLD,3, temp3_3x1, cLUA);

	mult_mv((const float**)_dcLLA_3x3,3,3, _C_LLA, temp3_3x1);
	sum_vv(pLELB,3, temp3_3x1, cLLA);
	
	
	mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_wst);

	mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_rhy);
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_18x18);
	mult_mv((const float**)_TEMP1_18x18,3,3, _AXIS_X, axis_rhr);
	mult_mv((const float**)_dcRUL_3x3,3,3, _AXIS_Y, axis_rhp);
	axis_rkn[1] = axis_rhp[1];
	axis_rkn[2] = axis_rhp[2];
	axis_rkn[3] = axis_rhp[3];
	axis_rap[1] = axis_rhp[1];
	axis_rap[2] = axis_rhp[2];
	axis_rap[3] = axis_rhp[3];
	mult_mv((const float**)_dcRF_3x3,3,3, _AXIS_X, axis_rar);

	axis_lhy[1] = axis_rhy[1];
	axis_lhy[2] = axis_rhy[2];
	axis_lhy[3] = axis_rhy[3];
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_18x18);
	mult_mv((const float**)_TEMP1_18x18,3,3, _AXIS_X, axis_lhr);
	mult_mv((const float**)_dcLUL_3x3,3,3, _AXIS_Y, axis_lhp);
	axis_lkn[1] = axis_lhp[1];
	axis_lkn[2] = axis_lhp[2];
	axis_lkn[3] = axis_lhp[3];
	axis_lap[1] = axis_lhp[1];
	axis_lap[2] = axis_lhp[2];
	axis_lap[3] = axis_lhp[3];
	mult_mv((const float**)_dcLF_3x3,3,3, _AXIS_X, axis_lar);

	mult_mv((const float**)_dcTOR_3x3,3,3, _AXIS_Y, axis_rsp);
	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_18x18);
	mult_mv((const float**)_TEMP1_18x18,3,3, _AXIS_X, axis_rsr);
	mult_mv((const float**)_dcRUA_3x3,3,3, _AXIS_Z, axis_rsy);
	mult_mv((const float**)_dcRUA_3x3,3,3, _AXIS_Y, axis_rep);
	mult_mv((const float**)_dcRLA_3x3,3,3, _AXIS_Z, axis_rey);

	axis_lsp[1] = axis_rsp[1];
	axis_lsp[2] = axis_rsp[2];
	axis_lsp[3] = axis_rsp[3];
	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_18x18);
	mult_mv((const float**)_TEMP1_18x18,3,3, _AXIS_X, axis_lsr);
	mult_mv((const float**)_dcLUA_3x3,3,3, _AXIS_Z, axis_lsy);
	mult_mv((const float**)_dcLUA_3x3,3,3, _AXIS_Y, axis_lep);
	mult_mv((const float**)_dcLLA_3x3,3,3, _AXIS_Z, axis_ley);


	ftemp = m_PEL/m_TOTAL;
	jCOM_PEL_3x29[1][1] = ftemp;
	jCOM_PEL_3x29[2][2] = ftemp;
	jCOM_PEL_3x29[3][3] = ftemp;
	diff_vv(cPEL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_PEL_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_PEL_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_PEL_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_PEL_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_PEL_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_PEL_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_PEL_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_PEL_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_PEL_3x29[3][6] = ftemp*temp4_3x1[3];


	ftemp = m_TOR/m_TOTAL;
	jCOM_TOR_3x29[1][1] = ftemp;
	jCOM_TOR_3x29[2][2] = ftemp;
	jCOM_TOR_3x29[3][3] = ftemp;
	diff_vv(cTOR,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cTOR,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x29[1][7] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x29[2][7] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x29[3][7] = ftemp*temp4_3x1[3];


	ftemp = m_ULEG/m_TOTAL;
	jCOM_RUL_3x29[1][1] = ftemp;
	jCOM_RUL_3x29[2][2] = ftemp;
	jCOM_RUL_3x29[3][3] = ftemp;
	diff_vv(cRUL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRUL,3,pRHIP, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x29[1][8] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x29[2][8] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x29[3][8] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x29[1][9] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x29[2][9] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x29[3][9] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x29[1][10] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x29[2][10] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x29[3][10] = ftemp*temp4_3x1[3];


	ftemp = m_LLEG/m_TOTAL;
	jCOM_RLL_3x29[1][1] = ftemp;
	jCOM_RLL_3x29[2][2] = ftemp;
	jCOM_RLL_3x29[3][3] = ftemp;
	diff_vv(cRLL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRLL,3,pRHIP, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][8] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][8] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][8] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][9] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][9] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][9] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][10] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][10] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][10] = ftemp*temp4_3x1[3];
	diff_vv(cRLL,3,pRKN, temp3_3x1);
	cross(1.f,axis_rkn, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][11] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][11] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][11] = ftemp*temp4_3x1[3];


	ftemp = m_FOOT/m_TOTAL;
	jCOM_RF_3x29[1][1] = ftemp;
	jCOM_RF_3x29[2][2] = ftemp;
	jCOM_RF_3x29[3][3] = ftemp;
	diff_vv(cRF,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRF,3,pRHIP, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][8] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][8] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][8] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][9] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][9] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][9] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][10] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][10] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][10] = ftemp*temp4_3x1[3];
	diff_vv(cRF,3,pRKN, temp3_3x1);
	cross(1.f,axis_rkn, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][11] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][11] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][11] = ftemp*temp4_3x1[3];
	diff_vv(cRF,3,pRANK, temp3_3x1);
	cross(1.f,axis_rap, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][12] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][12] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][12] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rar, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][13] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][13] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][13] = ftemp*temp4_3x1[3];



	ftemp = m_ULEG/m_TOTAL;
	jCOM_LUL_3x29[1][1] = ftemp;
	jCOM_LUL_3x29[2][2] = ftemp;
	jCOM_LUL_3x29[3][3] = ftemp;
	diff_vv(cLUL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLUL,3,pLHIP, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x29[1][14] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x29[2][14] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x29[3][14] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x29[1][15] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x29[2][15] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x29[3][15] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x29[1][16] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x29[2][16] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x29[3][16] = ftemp*temp4_3x1[3];


	ftemp = m_LLEG/m_TOTAL;
	jCOM_LLL_3x29[1][1] = ftemp;
	jCOM_LLL_3x29[2][2] = ftemp;
	jCOM_LLL_3x29[3][3] = ftemp;
	diff_vv(cLLL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLLL,3,pLHIP, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][14] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][14] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][14] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][15] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][15] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][15] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][16] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][16] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][16] = ftemp*temp4_3x1[3];
	diff_vv(cLLL,3,pLKN, temp3_3x1);
	cross(1.f,axis_lkn, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][17] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][17] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][17] = ftemp*temp4_3x1[3];


	ftemp = (float)(m_FOOT/m_TOTAL);
	jCOM_LF_3x29[1][1] = ftemp;
	jCOM_LF_3x29[2][2] = ftemp;
	jCOM_LF_3x29[3][3] = ftemp;
	diff_vv(cLF,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLF,3,pLHIP, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][14] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][14] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][14] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][15] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][15] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][15] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][16] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][16] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][16] = ftemp*temp4_3x1[3];
	diff_vv(cLF,3,pLKN, temp3_3x1);
	cross(1.f,axis_lkn, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][17] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][17] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][17] = ftemp*temp4_3x1[3];
	diff_vv(cLF,3,pLANK, temp3_3x1);
	cross(1.f,axis_lap, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][18] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][18] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][18] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lar, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][19] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][19] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][19] = ftemp*temp4_3x1[3];


	ftemp = m_UARM/m_TOTAL;
	jCOM_RUA_3x29[1][1] = ftemp;
	jCOM_RUA_3x29[2][2] = ftemp;
	jCOM_RUA_3x29[3][3] = ftemp;
	diff_vv(cRUA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRUA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][7] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][7] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cRUA,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][20] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][20] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][20] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][21] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][21] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][21] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][22] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][22] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][22] = ftemp*temp4_3x1[3];


	ftemp = m_LARM/m_TOTAL;
	jCOM_RLA_3x29[1][1] = ftemp;
	jCOM_RLA_3x29[2][2] = ftemp;
	jCOM_RLA_3x29[3][3] = ftemp;
	diff_vv(cRLA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][7] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][7] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][20] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][20] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][20] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][21] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][21] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][21] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][22] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][22] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][22] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pRELB, temp3_3x1);
	cross(1.f,axis_rep, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][23] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][23] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][23] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rey, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][24] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][24] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][24] = ftemp*temp4_3x1[3];


	ftemp = m_UARM/m_TOTAL;
	jCOM_LUA_3x29[1][1] = ftemp;
	jCOM_LUA_3x29[2][2] = ftemp;
	jCOM_LUA_3x29[3][3] = ftemp;
	diff_vv(cLUA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLUA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][7] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][7] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cLUA,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][25] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][25] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][25] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][26] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][26] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][26] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][27] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][27] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][27] = ftemp*temp4_3x1[3];


	ftemp = m_LARM/m_TOTAL;
	jCOM_LLA_3x29[1][1] = ftemp;
	jCOM_LLA_3x29[2][2] = ftemp;
	jCOM_LLA_3x29[3][3] = ftemp;
	diff_vv(cLLA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][7] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][7] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][25] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][25] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][25] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][26] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][26] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][26] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][27] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][27] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][27] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pLELB, temp3_3x1);
	cross(1.f,axis_lep, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][28] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][28] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][28] = ftemp*temp4_3x1[3];
	cross(1.f,axis_ley, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][29] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][29] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][29] = ftemp*temp4_3x1[3];
	
	
	jCOMlb_3x18[1][1] = 1.f;
	jCOMlb_3x18[2][2] = 1.f;
	jCOMlb_3x18[3][3] = 1.f;


	for(i=1; i<=3; i++)
	{
		jCOMlb_3x18[i][4] = jCOM_PEL_3x29[i][4] + jCOM_TOR_3x29[i][4] + jCOM_RUL_3x29[i][4] + jCOM_RLL_3x29[i][4] + jCOM_RF_3x29[i][4] + jCOM_LUL_3x29[i][4] + jCOM_LLL_3x29[i][4] + jCOM_LF_3x29[i][4] + jCOM_RUA_3x29[i][4] + jCOM_RLA_3x29[i][4] + jCOM_LUA_3x29[i][4] + jCOM_LLA_3x29[i][4];
		jCOMlb_3x18[i][5] = jCOM_PEL_3x29[i][5] + jCOM_TOR_3x29[i][5] + jCOM_RUL_3x29[i][5] + jCOM_RLL_3x29[i][5] + jCOM_RF_3x29[i][5] + jCOM_LUL_3x29[i][5] + jCOM_LLL_3x29[i][5] + jCOM_LF_3x29[i][5] + jCOM_RUA_3x29[i][5] + jCOM_RLA_3x29[i][5] + jCOM_LUA_3x29[i][5] + jCOM_LLA_3x29[i][5];
		jCOMlb_3x18[i][6] = jCOM_PEL_3x29[i][6] + jCOM_TOR_3x29[i][6] + jCOM_RUL_3x29[i][6] + jCOM_RLL_3x29[i][6] + jCOM_RF_3x29[i][6] + jCOM_LUL_3x29[i][6] + jCOM_LLL_3x29[i][6] + jCOM_LF_3x29[i][6] + jCOM_RUA_3x29[i][6] + jCOM_RLA_3x29[i][6] + jCOM_LUA_3x29[i][6] + jCOM_LLA_3x29[i][6];
		jCOMlb_3x18[i][7] = jCOM_RUL_3x29[i][8] + jCOM_RLL_3x29[i][8] + jCOM_RF_3x29[i][8];
		jCOMlb_3x18[i][8] = jCOM_RUL_3x29[i][9] + jCOM_RLL_3x29[i][9] + jCOM_RF_3x29[i][9];
		jCOMlb_3x18[i][9] = jCOM_RUL_3x29[i][10] + jCOM_RLL_3x29[i][10] + jCOM_RF_3x29[i][10];
		jCOMlb_3x18[i][10] = jCOM_RUL_3x29[i][11] + jCOM_RLL_3x29[i][11] + jCOM_RF_3x29[i][11];
		jCOMlb_3x18[i][11] = jCOM_RUL_3x29[i][12] + jCOM_RLL_3x29[i][12] + jCOM_RF_3x29[i][12];
		jCOMlb_3x18[i][12] = jCOM_RUL_3x29[i][13] + jCOM_RLL_3x29[i][13] + jCOM_RF_3x29[i][13];
		jCOMlb_3x18[i][13] = jCOM_LUL_3x29[i][14] + jCOM_LLL_3x29[i][14] + jCOM_LF_3x29[i][14];
		jCOMlb_3x18[i][14] = jCOM_LUL_3x29[i][15] + jCOM_LLL_3x29[i][15] + jCOM_LF_3x29[i][15];
		jCOMlb_3x18[i][15] = jCOM_LUL_3x29[i][16] + jCOM_LLL_3x29[i][16] + jCOM_LF_3x29[i][16];
		jCOMlb_3x18[i][16] = jCOM_LUL_3x29[i][17] + jCOM_LLL_3x29[i][17] + jCOM_LF_3x29[i][17];
		jCOMlb_3x18[i][17] = jCOM_LUL_3x29[i][18] + jCOM_LLL_3x29[i][18] + jCOM_LF_3x29[i][18];
		jCOMlb_3x18[i][18] = jCOM_LUL_3x29[i][19] + jCOM_LLL_3x29[i][19] + jCOM_LF_3x29[i][19];
		
		jCOMub_3x11[i][1] = jCOM_TOR_3x29[i][7] + jCOM_RUA_3x29[i][7] + jCOM_RLA_3x29[i][7] + jCOM_LUA_3x29[i][7] + jCOM_LLA_3x29[i][7];
		jCOMub_3x11[i][2] = jCOM_RUA_3x29[i][20] + jCOM_RLA_3x29[i][20];
		jCOMub_3x11[i][3] = jCOM_RUA_3x29[i][21] + jCOM_RLA_3x29[i][21];
		jCOMub_3x11[i][4] = jCOM_RUA_3x29[i][22] + jCOM_RLA_3x29[i][22];
		jCOMub_3x11[i][5] = jCOM_RUA_3x29[i][23] + jCOM_RLA_3x29[i][23];
		jCOMub_3x11[i][6] = jCOM_RUA_3x29[i][24] + jCOM_RLA_3x29[i][24];
		jCOMub_3x11[i][7] = jCOM_LUA_3x29[i][25] + jCOM_LLA_3x29[i][25];	
		jCOMub_3x11[i][8] = jCOM_LUA_3x29[i][26] + jCOM_LLA_3x29[i][26];	
		jCOMub_3x11[i][9] = jCOM_LUA_3x29[i][27] + jCOM_LLA_3x29[i][27];	
		jCOMub_3x11[i][10] = jCOM_LUA_3x29[i][28] + jCOM_LLA_3x29[i][28];	
		jCOMub_3x11[i][11] = jCOM_LUA_3x29[i][29] + jCOM_LLA_3x29[i][29];	
	
	}

	return 0;

}



int Pos_COM(const float *Qlb_19x1, const float *Qub_11x1, float *pCOM_3x1)
{
	float wst, rhy, rhp, rhr, rkn, rap, rar, lhy, lhp, lhr, lkn, lap, lar, rsp, rsr, rsy, rep, rey, lsp, lsr, lsy, lep, ley;
	
	float pRHIP[4], pRKN[4], pRANK[4];
	float pLHIP[4], pLKN[4], pLANK[4];
	float pWST[4], pRSHLD[4], pRELB[4];
	float pLSHLD[4], pLELB[4];

	float cPEL[4], cTOR[4], cRUL[4], cRLL[4], cRF[4], cLUL[4], cLLL[4], cLF[4], cRUA[4], cRLA[4], cLUA[4], cLLA[4];

	float temp3_3x1[4];

	float qPEL[5] = {0., Qlb_19x1[4], Qlb_19x1[5], Qlb_19x1[6], Qlb_19x1[7]};
	float pPC[4] = {0., Qlb_19x1[1], Qlb_19x1[2], Qlb_19x1[3]};

	rhy = Qlb_19x1[8];
	rhr = Qlb_19x1[9];
	rhp = Qlb_19x1[10];
	rkn = Qlb_19x1[11];
	rap = Qlb_19x1[12];
	rar = Qlb_19x1[13];

	lhy = Qlb_19x1[14];
	lhr = Qlb_19x1[15];
	lhp = Qlb_19x1[16];
	lkn = Qlb_19x1[17];
	lap = Qlb_19x1[18];
	lar = Qlb_19x1[19];

	wst = Qub_11x1[1];
	rsp = Qub_11x1[2];
	rsr = Qub_11x1[3];
	rsy = Qub_11x1[4];
	rep = Qub_11x1[5];
	rey = Qub_11x1[6];
	lsp = Qub_11x1[7];
	lsr = Qub_11x1[8];
	lsy = Qub_11x1[9];
	lep = Qub_11x1[10];
	ley = Qub_11x1[11];


	QT2DC(qPEL, _dcPEL_3x3);
	
	RZ(rhy, _Rz_RHY_3x3);
	RX(rhr, _Rx_RHR_3x3);
	RY(rhp, _Ry_RHP_3x3);
	RY(rkn, _Ry_RKN_3x3);
	RY(rap, _Ry_RAP_3x3);
	RX(rar, _Rx_RAR_3x3);

	RZ(lhy, _Rz_LHY_3x3);
	RX(lhr, _Rx_LHR_3x3);
	RY(lhp, _Ry_LHP_3x3);
	RY(lkn, _Ry_LKN_3x3);
	RY(lap, _Ry_LAP_3x3);
	RX(lar, _Rx_LAR_3x3);

	RZ(wst, _Rz_WST_3x3);
	
	RY(rsp, _Ry_RSP_3x3);
	RX(rsr, _Rx_RSR_3x3);
	RZ(rsy, _Rz_RSY_3x3);
	RY(rep, _Ry_REP_3x3);
	RZ(rey, _Rz_REY_3x3);

	RY(lsp, _Ry_LSP_3x3);
	RX(lsr, _Rx_LSR_3x3);
	RZ(lsy, _Rz_LSY_3x3);
	RY(lep, _Ry_LEP_3x3);
	RZ(ley, _Rz_LEY_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcRUL_3x3);

	mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
	
	mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcRF_3x3);


	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcLUL_3x3);

	mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
	
	mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcLF_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_WST_3x3,3, _dcTOR_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_RSR_3x3,3,3, (const float**)_Rz_RSY_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcRUA_3x3);

	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REP_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_Rz_REY_3x3,3, _dcRLA_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_LSR_3x3,3,3, (const float**)_Rz_LSY_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcLUA_3x3);

	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEP_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_Rz_LEY_3x3,3, _dcLLA_3x3);

	
	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRHIP);

	mult_mv((const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRHIP,3, temp3_3x1, pRKN);

	mult_mv((const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, pRANK);


	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLHIP);

	mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLHIP,3, temp3_3x1, pLKN);

	mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, pLANK);

	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_PEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pWST);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_RSHLD, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRSHLD);

	mult_mv((const float**)_dcRUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pRSHLD,3, temp3_3x1, pRELB);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_LSHLD, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLSHLD);

	mult_mv((const float**)_dcLUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pLSHLD,3, temp3_3x1, pLELB);

	mult_mv((const float**)_dcPEL_3x3,3,3, _C_PEL, temp3_3x1);
	sum_vv(pPC,3, temp3_3x1, cPEL);

	mult_mv((const float**)_dcTOR_3x3,3,3, _C_TOR, temp3_3x1);
	sum_vv(pWST,3, temp3_3x1, cTOR);

	mult_mv((const float**)_dcRUL_3x3,3,3, _C_RUL, temp3_3x1);
	sum_vv(pRHIP,3, temp3_3x1, cRUL);

	mult_mv((const float**)_dcRLL_3x3,3,3, _C_RLL, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, cRLL);

	mult_mv((const float**)_dcRF_3x3,3,3, _C_RF, temp3_3x1);
	sum_vv(pRANK,3, temp3_3x1, cRF);

	mult_mv((const float**)_dcLUL_3x3,3,3, _C_LUL, temp3_3x1);
	sum_vv(pLHIP,3, temp3_3x1, cLUL);

	mult_mv((const float**)_dcLLL_3x3,3,3, _C_LLL, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, cLLL);

	mult_mv((const float**)_dcLF_3x3,3,3, _C_LF, temp3_3x1);
	sum_vv(pLANK,3, temp3_3x1, cLF);

	mult_mv((const float**)_dcRUA_3x3,3,3, _C_RUA, temp3_3x1);
	sum_vv(pRSHLD,3, temp3_3x1, cRUA);

	mult_mv((const float**)_dcRLA_3x3,3,3, _C_RLA, temp3_3x1);
	sum_vv(pRELB,3, temp3_3x1, cRLA);

	mult_mv((const float**)_dcLUA_3x3,3,3, _C_LUA, temp3_3x1);
	sum_vv(pLSHLD,3, temp3_3x1, cLUA);

	mult_mv((const float**)_dcLLA_3x3,3,3, _C_LLA, temp3_3x1);
	sum_vv(pLELB,3, temp3_3x1, cLLA);
	
	pCOM_3x1[1] = ((m_PEL*cPEL[1] +m_TOR*cTOR[1] + m_ULEG*(cRUL[1]+cLUL[1]) + m_LLEG*(cRLL[1]+cLLL[1]) +m_FOOT*(cRF[1]+cLF[1]) + m_UARM*(cRUA[1]+cLUA[1]) + m_LARM*(cRLA[1]+cLLA[1]))/m_TOTAL);
	pCOM_3x1[2] = ((m_PEL*cPEL[2] +m_TOR*cTOR[2] + m_ULEG*(cRUL[2]+cLUL[2]) + m_LLEG*(cRLL[2]+cLLL[2]) +m_FOOT*(cRF[2]+cLF[2]) + m_UARM*(cRUA[2]+cLUA[2]) + m_LARM*(cRLA[2]+cLLA[2]))/m_TOTAL);
	pCOM_3x1[3] = ((m_PEL*cPEL[3] +m_TOR*cTOR[3] + m_ULEG*(cRUL[3]+cLUL[3]) + m_LLEG*(cRLL[3]+cLLL[3]) +m_FOOT*(cRF[3]+cLF[3]) + m_UARM*(cRUA[3]+cLUA[3]) + m_LARM*(cRLA[3]+cLLA[3]))/m_TOTAL);

	return 0;
}



int MocapKine(const float *Qlb_19x1, const float *Qub_11x1,	// inputs
				float *pPC_3x1, float *qPEL_4x1,	// outputs
				float **jRFlb_6x18, float *pRFC_3x1, float *qRF_4x1,// outputs
				float **jLFlb_6x18, float *pLFC_3x1, float *qLF_4x1,// outputs
				float **jCOMlb_3x18, float **jCOMub_3x11, float *pCOM_3x1)
{
	float wst, rhy, rhp, rhr, rkn, rap, rar, lhy, lhp, lhr, lkn, lap, lar, rsp, rsr, rsy, rep, rey, lsp, lsr, lsy, lep, ley;
	
	float axis_rhy[4], axis_rhr[4], axis_rhp[4], axis_rkn[4], axis_rap[4], axis_rar[4];	 
	float axis_lhy[4], axis_lhr[4], axis_lhp[4], axis_lkn[4], axis_lap[4], axis_lar[4];	
	float axis_wst[4], axis_rsp[4], axis_rsr[4], axis_rsy[4], axis_rep[4], axis_rey[4];
	float axis_lsp[4], axis_lsr[4], axis_lsy[4], axis_lep[4], axis_ley[4];

	float pRHIP[4], pRKN[4], pRANK[4], pRFC[4];
	float pLHIP[4], pLKN[4], pLANK[4], pLFC[4];
	float pWST[4], pRSHLD[4], pRELB[4];
	float pLSHLD[4], pLELB[4];

	float cPEL[4], cTOR[4], cRUL[4], cRLL[4], cRF[4], cLUL[4], cLLL[4], cLF[4], cRUA[4], cRLA[4], cLUA[4], cLLA[4];

	float temp3_3x1[4], temp4_3x1[4];

	//--- Jacobian for [xp,yp,zp,wPELx,wPELy,wPELz,wstp,rhyp,rhrp,rhpp,rknp,rapp,rarp,lhyp,lhrp,lhpp,lknp,lapp,larp,rspp,rsrp,rsyp,repp,reyp,lspp,lsrp,lsyp,lepp,leyp]' : 29x1
	float jCOM_PEL_3x29[4][30], jCOM_RUL_3x29[4][30], jCOM_RLL_3x29[4][30], jCOM_RF_3x29[4][30], jCOM_LUL_3x29[4][30], jCOM_LLL_3x29[4][30], jCOM_LF_3x29[4][30];
	float jCOM_TOR_3x29[4][30], jCOM_RUA_3x29[4][30], jCOM_RLA_3x29[4][30], jCOM_LUA_3x29[4][30], jCOM_LLA_3x29[4][30];

	float qPEL[5] = {0., Qlb_19x1[4], Qlb_19x1[5], Qlb_19x1[6], Qlb_19x1[7]};
	float pPC[4] = {0., Qlb_19x1[1], Qlb_19x1[2], Qlb_19x1[3]};

	int i, j;
	float ftemp;
	
	for(i=1; i<=3; i++)
	{
		jCOMlb_3x18[i][1] = 0.;
		jCOMlb_3x18[i][2] = 0.;
		jCOMlb_3x18[i][3] = 0.;

		for(j=1; j<=29; j++)
		{
			jCOM_PEL_3x29[i][j] = 0.;
			jCOM_RUL_3x29[i][j] = 0.;
			jCOM_RLL_3x29[i][j] = 0.;
			jCOM_RF_3x29[i][j] = 0.;
			jCOM_LUL_3x29[i][j] = 0.;
			jCOM_LLL_3x29[i][j] = 0.;
			jCOM_LF_3x29[i][j] = 0.;
			jCOM_TOR_3x29[i][j] = 0.;
			jCOM_RUA_3x29[i][j] = 0.;
			jCOM_RLA_3x29[i][j] = 0.;
			jCOM_LUA_3x29[i][j] = 0.;
			jCOM_LLA_3x29[i][j] = 0.;
		}
	}

	rhy = Qlb_19x1[8];
	rhr = Qlb_19x1[9];
	rhp = Qlb_19x1[10];
	rkn = Qlb_19x1[11];
	rap = Qlb_19x1[12];
	rar = Qlb_19x1[13];

	lhy = Qlb_19x1[14];
	lhr = Qlb_19x1[15];
	lhp = Qlb_19x1[16];
	lkn = Qlb_19x1[17];
	lap = Qlb_19x1[18];
	lar = Qlb_19x1[19];

	wst = Qub_11x1[1];
	rsp = Qub_11x1[2];
	rsr = Qub_11x1[3];
	rsy = Qub_11x1[4];
	rep = Qub_11x1[5];
	rey = Qub_11x1[6];
	lsp = Qub_11x1[7];
	lsr = Qub_11x1[8];
	lsy = Qub_11x1[9];
	lep = Qub_11x1[10];
	ley = Qub_11x1[11];


	QT2DC(qPEL, _dcPEL_3x3);
	
	RZ(rhy, _Rz_RHY_3x3);
	RX(rhr, _Rx_RHR_3x3);
	RY(rhp, _Ry_RHP_3x3);
	RY(rkn, _Ry_RKN_3x3);
	RY(rap, _Ry_RAP_3x3);
	RX(rar, _Rx_RAR_3x3);

	RZ(lhy, _Rz_LHY_3x3);
	RX(lhr, _Rx_LHR_3x3);
	RY(lhp, _Ry_LHP_3x3);
	RY(lkn, _Ry_LKN_3x3);
	RY(lap, _Ry_LAP_3x3);
	RX(lar, _Rx_LAR_3x3);

	RZ(wst, _Rz_WST_3x3);
	
	RY(rsp, _Ry_RSP_3x3);
	RX(rsr, _Rx_RSR_3x3);
	RZ(rsy, _Rz_RSY_3x3);
	RY(rep, _Ry_REP_3x3);
	RZ(rey, _Rz_REY_3x3);

	RY(lsp, _Ry_LSP_3x3);
	RX(lsr, _Rx_LSR_3x3);
	RZ(lsy, _Rz_LSY_3x3);
	RY(lep, _Ry_LEP_3x3);
	RZ(ley, _Rz_LEY_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcRUL_3x3);

	mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
	
	mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcRF_3x3);


	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcLUL_3x3);

	mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
	
	mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcLF_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_WST_3x3,3, _dcTOR_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_RSR_3x3,3,3, (const float**)_Rz_RSY_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcRUA_3x3);

	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REP_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_Rz_REY_3x3,3, _dcRLA_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_LSR_3x3,3,3, (const float**)_Rz_LSY_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcLUA_3x3);

	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEP_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_Rz_LEY_3x3,3, _dcLLA_3x3);

	
	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRHIP);

	mult_mv((const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRHIP,3, temp3_3x1, pRKN);

	mult_mv((const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, pRANK);

	mult_mv((const float**)_dcRF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(pRANK,3, temp3_3x1, pRFC);


	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLHIP);

	mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLHIP,3, temp3_3x1, pLKN);

	mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, pLANK);

	mult_mv((const float**)_dcLF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(pLANK,3, temp3_3x1, pLFC);


	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_PEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pWST);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_RSHLD, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRSHLD);

	mult_mv((const float**)_dcRUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pRSHLD,3, temp3_3x1, pRELB);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_LSHLD, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLSHLD);

	mult_mv((const float**)_dcLUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pLSHLD,3, temp3_3x1, pLELB);

	mult_mv((const float**)_dcPEL_3x3,3,3, _C_PEL, temp3_3x1);
	sum_vv(pPC,3, temp3_3x1, cPEL);

	mult_mv((const float**)_dcTOR_3x3,3,3, _C_TOR, temp3_3x1);
	sum_vv(pWST,3, temp3_3x1, cTOR);

	mult_mv((const float**)_dcRUL_3x3,3,3, _C_RUL, temp3_3x1);
	sum_vv(pRHIP,3, temp3_3x1, cRUL);

	mult_mv((const float**)_dcRLL_3x3,3,3, _C_RLL, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, cRLL);

	mult_mv((const float**)_dcRF_3x3,3,3, _C_RF, temp3_3x1);
	sum_vv(pRANK,3, temp3_3x1, cRF);

	mult_mv((const float**)_dcLUL_3x3,3,3, _C_LUL, temp3_3x1);
	sum_vv(pLHIP,3, temp3_3x1, cLUL);

	mult_mv((const float**)_dcLLL_3x3,3,3, _C_LLL, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, cLLL);

	mult_mv((const float**)_dcLF_3x3,3,3, _C_LF, temp3_3x1);
	sum_vv(pLANK,3, temp3_3x1, cLF);

	mult_mv((const float**)_dcRUA_3x3,3,3, _C_RUA, temp3_3x1);
	sum_vv(pRSHLD,3, temp3_3x1, cRUA);

	mult_mv((const float**)_dcRLA_3x3,3,3, _C_RLA, temp3_3x1);
	sum_vv(pRELB,3, temp3_3x1, cRLA);

	mult_mv((const float**)_dcLUA_3x3,3,3, _C_LUA, temp3_3x1);
	sum_vv(pLSHLD,3, temp3_3x1, cLUA);

	mult_mv((const float**)_dcLLA_3x3,3,3, _C_LLA, temp3_3x1);
	sum_vv(pLELB,3, temp3_3x1, cLLA);
	
	
	mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_wst);

	mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_rhy);
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_18x18);
	mult_mv((const float**)_TEMP1_18x18,3,3, _AXIS_X, axis_rhr);
	mult_mv((const float**)_dcRUL_3x3,3,3, _AXIS_Y, axis_rhp);
	axis_rkn[1] = axis_rhp[1];
	axis_rkn[2] = axis_rhp[2];
	axis_rkn[3] = axis_rhp[3];
	axis_rap[1] = axis_rhp[1];
	axis_rap[2] = axis_rhp[2];
	axis_rap[3] = axis_rhp[3];
	mult_mv((const float**)_dcRF_3x3,3,3, _AXIS_X, axis_rar);

	axis_lhy[1] = axis_rhy[1];
	axis_lhy[2] = axis_rhy[2];
	axis_lhy[3] = axis_rhy[3];
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_18x18);
	mult_mv((const float**)_TEMP1_18x18,3,3, _AXIS_X, axis_lhr);
	mult_mv((const float**)_dcLUL_3x3,3,3, _AXIS_Y, axis_lhp);
	axis_lkn[1] = axis_lhp[1];
	axis_lkn[2] = axis_lhp[2];
	axis_lkn[3] = axis_lhp[3];
	axis_lap[1] = axis_lhp[1];
	axis_lap[2] = axis_lhp[2];
	axis_lap[3] = axis_lhp[3];
	mult_mv((const float**)_dcLF_3x3,3,3, _AXIS_X, axis_lar);

	mult_mv((const float**)_dcTOR_3x3,3,3, _AXIS_Y, axis_rsp);
	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_18x18);
	mult_mv((const float**)_TEMP1_18x18,3,3, _AXIS_X, axis_rsr);
	mult_mv((const float**)_dcRUA_3x3,3,3, _AXIS_Z, axis_rsy);
	mult_mv((const float**)_dcRUA_3x3,3,3, _AXIS_Y, axis_rep);
	mult_mv((const float**)_dcRLA_3x3,3,3, _AXIS_Z, axis_rey);

	axis_lsp[1] = axis_rsp[1];
	axis_lsp[2] = axis_rsp[2];
	axis_lsp[3] = axis_rsp[3];
	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_18x18);
	mult_mv((const float**)_TEMP1_18x18,3,3, _AXIS_X, axis_lsr);
	mult_mv((const float**)_dcLUA_3x3,3,3, _AXIS_Z, axis_lsy);
	mult_mv((const float**)_dcLUA_3x3,3,3, _AXIS_Y, axis_lep);
	mult_mv((const float**)_dcLLA_3x3,3,3, _AXIS_Z, axis_ley);


	ftemp = (m_PEL/m_TOTAL);
	jCOM_PEL_3x29[1][1] = ftemp;
	jCOM_PEL_3x29[2][2] = ftemp;
	jCOM_PEL_3x29[3][3] = ftemp;
	diff_vv(cPEL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_PEL_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_PEL_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_PEL_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_PEL_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_PEL_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_PEL_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_PEL_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_PEL_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_PEL_3x29[3][6] = ftemp*temp4_3x1[3];


	ftemp = (m_TOR/m_TOTAL);
	jCOM_TOR_3x29[1][1] = ftemp;
	jCOM_TOR_3x29[2][2] = ftemp;
	jCOM_TOR_3x29[3][3] = ftemp;
	diff_vv(cTOR,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cTOR,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x29[1][7] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x29[2][7] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x29[3][7] = ftemp*temp4_3x1[3];


	ftemp = (m_ULEG/m_TOTAL);
	jCOM_RUL_3x29[1][1] = ftemp;
	jCOM_RUL_3x29[2][2] = ftemp;
	jCOM_RUL_3x29[3][3] = ftemp;
	diff_vv(cRUL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRUL,3,pRHIP, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x29[1][8] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x29[2][8] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x29[3][8] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x29[1][9] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x29[2][9] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x29[3][9] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x29[1][10] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x29[2][10] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x29[3][10] = ftemp*temp4_3x1[3];


	ftemp = (m_LLEG/m_TOTAL);
	jCOM_RLL_3x29[1][1] = ftemp;
	jCOM_RLL_3x29[2][2] = ftemp;
	jCOM_RLL_3x29[3][3] = ftemp;
	diff_vv(cRLL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRLL,3,pRHIP, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][8] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][8] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][8] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][9] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][9] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][9] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][10] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][10] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][10] = ftemp*temp4_3x1[3];
	diff_vv(cRLL,3,pRKN, temp3_3x1);
	cross(1.f,axis_rkn, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x29[1][11] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x29[2][11] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x29[3][11] = ftemp*temp4_3x1[3];


	ftemp = (m_FOOT/m_TOTAL);
	jCOM_RF_3x29[1][1] = ftemp;
	jCOM_RF_3x29[2][2] = ftemp;
	jCOM_RF_3x29[3][3] = ftemp;
	diff_vv(cRF,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRF,3,pRHIP, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][8] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][8] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][8] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][9] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][9] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][9] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][10] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][10] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][10] = ftemp*temp4_3x1[3];
	diff_vv(cRF,3,pRKN, temp3_3x1);
	cross(1.f,axis_rkn, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][11] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][11] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][11] = ftemp*temp4_3x1[3];
	diff_vv(cRF,3,pRANK, temp3_3x1);
	cross(1.f,axis_rap, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][12] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][12] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][12] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rar, temp3_3x1, temp4_3x1);
	jCOM_RF_3x29[1][13] = ftemp*temp4_3x1[1];
	jCOM_RF_3x29[2][13] = ftemp*temp4_3x1[2];
	jCOM_RF_3x29[3][13] = ftemp*temp4_3x1[3];



	ftemp = (m_ULEG/m_TOTAL);
	jCOM_LUL_3x29[1][1] = ftemp;
	jCOM_LUL_3x29[2][2] = ftemp;
	jCOM_LUL_3x29[3][3] = ftemp;
	diff_vv(cLUL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLUL,3,pLHIP, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x29[1][14] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x29[2][14] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x29[3][14] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x29[1][15] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x29[2][15] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x29[3][15] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x29[1][16] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x29[2][16] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x29[3][16] = ftemp*temp4_3x1[3];


	ftemp = (m_LLEG/m_TOTAL);
	jCOM_LLL_3x29[1][1] = ftemp;
	jCOM_LLL_3x29[2][2] = ftemp;
	jCOM_LLL_3x29[3][3] = ftemp;
	diff_vv(cLLL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLLL,3,pLHIP, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][14] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][14] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][14] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][15] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][15] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][15] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][16] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][16] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][16] = ftemp*temp4_3x1[3];
	diff_vv(cLLL,3,pLKN, temp3_3x1);
	cross(1.f,axis_lkn, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x29[1][17] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x29[2][17] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x29[3][17] = ftemp*temp4_3x1[3];


	ftemp = (m_FOOT/m_TOTAL);
	jCOM_LF_3x29[1][1] = ftemp;
	jCOM_LF_3x29[2][2] = ftemp;
	jCOM_LF_3x29[3][3] = ftemp;
	diff_vv(cLF,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLF,3,pLHIP, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][14] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][14] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][14] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][15] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][15] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][15] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][16] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][16] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][16] = ftemp*temp4_3x1[3];
	diff_vv(cLF,3,pLKN, temp3_3x1);
	cross(1.f,axis_lkn, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][17] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][17] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][17] = ftemp*temp4_3x1[3];
	diff_vv(cLF,3,pLANK, temp3_3x1);
	cross(1.f,axis_lap, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][18] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][18] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][18] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lar, temp3_3x1, temp4_3x1);
	jCOM_LF_3x29[1][19] = ftemp*temp4_3x1[1];
	jCOM_LF_3x29[2][19] = ftemp*temp4_3x1[2];
	jCOM_LF_3x29[3][19] = ftemp*temp4_3x1[3];


	ftemp = (m_UARM/m_TOTAL);
	jCOM_RUA_3x29[1][1] = ftemp;
	jCOM_RUA_3x29[2][2] = ftemp;
	jCOM_RUA_3x29[3][3] = ftemp;
	diff_vv(cRUA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRUA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][7] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][7] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cRUA,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][20] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][20] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][20] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][21] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][21] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][21] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x29[1][22] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x29[2][22] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x29[3][22] = ftemp*temp4_3x1[3];


	ftemp = (m_LARM/m_TOTAL);
	jCOM_RLA_3x29[1][1] = ftemp;
	jCOM_RLA_3x29[2][2] = ftemp;
	jCOM_RLA_3x29[3][3] = ftemp;
	diff_vv(cRLA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][7] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][7] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][20] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][20] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][20] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][21] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][21] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][21] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][22] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][22] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][22] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pRELB, temp3_3x1);
	cross(1.f,axis_rep, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][23] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][23] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][23] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rey, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x29[1][24] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x29[2][24] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x29[3][24] = ftemp*temp4_3x1[3];


	ftemp = (m_UARM/m_TOTAL);
	jCOM_LUA_3x29[1][1] = ftemp;
	jCOM_LUA_3x29[2][2] = ftemp;
	jCOM_LUA_3x29[3][3] = ftemp;
	diff_vv(cLUA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLUA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][7] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][7] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cLUA,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][25] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][25] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][25] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][26] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][26] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][26] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x29[1][27] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x29[2][27] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x29[3][27] = ftemp*temp4_3x1[3];


	ftemp = (m_LARM/m_TOTAL);
	jCOM_LLA_3x29[1][1] = ftemp;
	jCOM_LLA_3x29[2][2] = ftemp;
	jCOM_LLA_3x29[3][3] = ftemp;
	diff_vv(cLLA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][4] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][4] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][5] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][5] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][6] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][6] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][7] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][7] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][25] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][25] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][25] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][26] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][26] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][26] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][27] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][27] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][27] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pLELB, temp3_3x1);
	cross(1.f,axis_lep, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][28] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][28] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][28] = ftemp*temp4_3x1[3];
	cross(1.f,axis_ley, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x29[1][29] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x29[2][29] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x29[3][29] = ftemp*temp4_3x1[3];
	
	
	jCOMlb_3x18[1][1] = 1.f;
	jCOMlb_3x18[2][2] = 1.f;
	jCOMlb_3x18[3][3] = 1.f;
	for(i=1; i<=3; i++)
	{
		jCOMlb_3x18[i][4] = jCOM_PEL_3x29[i][4] + jCOM_TOR_3x29[i][4] + jCOM_RUL_3x29[i][4] + jCOM_RLL_3x29[i][4] + jCOM_RF_3x29[i][4] + jCOM_LUL_3x29[i][4] + jCOM_LLL_3x29[i][4] + jCOM_LF_3x29[i][4] + jCOM_RUA_3x29[i][4] + jCOM_RLA_3x29[i][4] + jCOM_LUA_3x29[i][4] + jCOM_LLA_3x29[i][4];
		jCOMlb_3x18[i][5] = jCOM_PEL_3x29[i][5] + jCOM_TOR_3x29[i][5] + jCOM_RUL_3x29[i][5] + jCOM_RLL_3x29[i][5] + jCOM_RF_3x29[i][5] + jCOM_LUL_3x29[i][5] + jCOM_LLL_3x29[i][5] + jCOM_LF_3x29[i][5] + jCOM_RUA_3x29[i][5] + jCOM_RLA_3x29[i][5] + jCOM_LUA_3x29[i][5] + jCOM_LLA_3x29[i][5];
		jCOMlb_3x18[i][6] = jCOM_PEL_3x29[i][6] + jCOM_TOR_3x29[i][6] + jCOM_RUL_3x29[i][6] + jCOM_RLL_3x29[i][6] + jCOM_RF_3x29[i][6] + jCOM_LUL_3x29[i][6] + jCOM_LLL_3x29[i][6] + jCOM_LF_3x29[i][6] + jCOM_RUA_3x29[i][6] + jCOM_RLA_3x29[i][6] + jCOM_LUA_3x29[i][6] + jCOM_LLA_3x29[i][6];
		jCOMlb_3x18[i][7] = jCOM_RUL_3x29[i][8] + jCOM_RLL_3x29[i][8] + jCOM_RF_3x29[i][8];
		jCOMlb_3x18[i][8] = jCOM_RUL_3x29[i][9] + jCOM_RLL_3x29[i][9] + jCOM_RF_3x29[i][9];
		jCOMlb_3x18[i][9] = jCOM_RUL_3x29[i][10] + jCOM_RLL_3x29[i][10] + jCOM_RF_3x29[i][10];
		jCOMlb_3x18[i][10] = jCOM_RUL_3x29[i][11] + jCOM_RLL_3x29[i][11] + jCOM_RF_3x29[i][11];
		jCOMlb_3x18[i][11] = jCOM_RUL_3x29[i][12] + jCOM_RLL_3x29[i][12] + jCOM_RF_3x29[i][12];
		jCOMlb_3x18[i][12] = jCOM_RUL_3x29[i][13] + jCOM_RLL_3x29[i][13] + jCOM_RF_3x29[i][13];
		jCOMlb_3x18[i][13] = jCOM_LUL_3x29[i][14] + jCOM_LLL_3x29[i][14] + jCOM_LF_3x29[i][14];
		jCOMlb_3x18[i][14] = jCOM_LUL_3x29[i][15] + jCOM_LLL_3x29[i][15] + jCOM_LF_3x29[i][15];
		jCOMlb_3x18[i][15] = jCOM_LUL_3x29[i][16] + jCOM_LLL_3x29[i][16] + jCOM_LF_3x29[i][16];
		jCOMlb_3x18[i][16] = jCOM_LUL_3x29[i][17] + jCOM_LLL_3x29[i][17] + jCOM_LF_3x29[i][17];
		jCOMlb_3x18[i][17] = jCOM_LUL_3x29[i][18] + jCOM_LLL_3x29[i][18] + jCOM_LF_3x29[i][18];
		jCOMlb_3x18[i][18] = jCOM_LUL_3x29[i][19] + jCOM_LLL_3x29[i][19] + jCOM_LF_3x29[i][19];
		
		jCOMub_3x11[i][1] = jCOM_TOR_3x29[i][7] + jCOM_RUA_3x29[i][7] + jCOM_RLA_3x29[i][7] + jCOM_LUA_3x29[i][7] + jCOM_LLA_3x29[i][7];
		jCOMub_3x11[i][2] = jCOM_RUA_3x29[i][20] + jCOM_RLA_3x29[i][20];
		jCOMub_3x11[i][3] = jCOM_RUA_3x29[i][21] + jCOM_RLA_3x29[i][21];
		jCOMub_3x11[i][4] = jCOM_RUA_3x29[i][22] + jCOM_RLA_3x29[i][22];
		jCOMub_3x11[i][5] = jCOM_RUA_3x29[i][23] + jCOM_RLA_3x29[i][23];
		jCOMub_3x11[i][6] = jCOM_RUA_3x29[i][24] + jCOM_RLA_3x29[i][24];
		jCOMub_3x11[i][7] = jCOM_LUA_3x29[i][25] + jCOM_LLA_3x29[i][25];	
		jCOMub_3x11[i][8] = jCOM_LUA_3x29[i][26] + jCOM_LLA_3x29[i][26];	
		jCOMub_3x11[i][9] = jCOM_LUA_3x29[i][27] + jCOM_LLA_3x29[i][27];	
		jCOMub_3x11[i][10] = jCOM_LUA_3x29[i][28] + jCOM_LLA_3x29[i][28];	
		jCOMub_3x11[i][11] = jCOM_LUA_3x29[i][29] + jCOM_LLA_3x29[i][29];	
	
	}


	jRFlb_6x18[1][1] = 1.f;
	jRFlb_6x18[2][2] = 1.f;
	jRFlb_6x18[3][3] = 1.f;
	jRFlb_6x18[4][4] = 1.f;
	jRFlb_6x18[5][5] = 1.f;
	jRFlb_6x18[6][6] = 1.f;

	diff_vv(pRFC,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][4] = temp4_3x1[1];
	jRFlb_6x18[2][4] = temp4_3x1[2];
	jRFlb_6x18[3][4] = temp4_3x1[3];

	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][5] = temp4_3x1[1];
	jRFlb_6x18[2][5] = temp4_3x1[2];
	jRFlb_6x18[3][5] = temp4_3x1[3];

	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][6] = temp4_3x1[1];
	jRFlb_6x18[2][6] = temp4_3x1[2];
	jRFlb_6x18[3][6] = temp4_3x1[3];

	diff_vv(pRFC,3,pRHIP, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][7] = temp4_3x1[1];
	jRFlb_6x18[2][7] = temp4_3x1[2];
	jRFlb_6x18[3][7] = temp4_3x1[3];
	jRFlb_6x18[4][7] = axis_rhy[1];
	jRFlb_6x18[5][7] = axis_rhy[2];
	jRFlb_6x18[6][7] = axis_rhy[3];

	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][8] = temp4_3x1[1];
	jRFlb_6x18[2][8] = temp4_3x1[2];
	jRFlb_6x18[3][8] = temp4_3x1[3];
	jRFlb_6x18[4][8] = axis_rhr[1];
	jRFlb_6x18[5][8] = axis_rhr[2];
	jRFlb_6x18[6][8] = axis_rhr[3];

	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][9] = temp4_3x1[1];
	jRFlb_6x18[2][9] = temp4_3x1[2];
	jRFlb_6x18[3][9] = temp4_3x1[3];
	jRFlb_6x18[4][9] = axis_rhp[1];
	jRFlb_6x18[5][9] = axis_rhp[2];
	jRFlb_6x18[6][9] = axis_rhp[3];

	diff_vv(pRFC,3,pRKN, temp3_3x1);
	cross(1.f,axis_rkn, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][10] = temp4_3x1[1];
	jRFlb_6x18[2][10] = temp4_3x1[2];
	jRFlb_6x18[3][10] = temp4_3x1[3];
	jRFlb_6x18[4][10] = axis_rkn[1];
	jRFlb_6x18[5][10] = axis_rkn[2];
	jRFlb_6x18[6][10] = axis_rkn[3];

	diff_vv(pRFC,3,pRANK, temp3_3x1);
	cross(1.f,axis_rap, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][11] = temp4_3x1[1];
	jRFlb_6x18[2][11] = temp4_3x1[2];
	jRFlb_6x18[3][11] = temp4_3x1[3];
	jRFlb_6x18[4][11] = axis_rap[1];
	jRFlb_6x18[5][11] = axis_rap[2];
	jRFlb_6x18[6][11] = axis_rap[3];

	cross(1.f,axis_rar, temp3_3x1, temp4_3x1);
	jRFlb_6x18[1][12] = temp4_3x1[1];
	jRFlb_6x18[2][12] = temp4_3x1[2];
	jRFlb_6x18[3][12] = temp4_3x1[3];
	jRFlb_6x18[4][12] = axis_rar[1];
	jRFlb_6x18[5][12] = axis_rar[2];
	jRFlb_6x18[6][12] = axis_rar[3];


	jLFlb_6x18[1][1] = 1.f;
	jLFlb_6x18[2][2] = 1.f;
	jLFlb_6x18[3][3] = 1.f;
	jLFlb_6x18[4][4] = 1.f;
	jLFlb_6x18[5][5] = 1.f;
	jLFlb_6x18[6][6] = 1.f;

	diff_vv(pLFC,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][4] = temp4_3x1[1];
	jLFlb_6x18[2][4] = temp4_3x1[2];
	jLFlb_6x18[3][4] = temp4_3x1[3];

	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][5] = temp4_3x1[1];
	jLFlb_6x18[2][5] = temp4_3x1[2];
	jLFlb_6x18[3][5] = temp4_3x1[3];

	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][6] = temp4_3x1[1];
	jLFlb_6x18[2][6] = temp4_3x1[2];
	jLFlb_6x18[3][6] = temp4_3x1[3];

	diff_vv(pLFC,3,pLHIP, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][13] = temp4_3x1[1];
	jLFlb_6x18[2][13] = temp4_3x1[2];
	jLFlb_6x18[3][13] = temp4_3x1[3];
	jLFlb_6x18[4][13] = axis_lhy[1];
	jLFlb_6x18[5][13] = axis_lhy[2];
	jLFlb_6x18[6][13] = axis_lhy[3];

	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][14] = temp4_3x1[1];
	jLFlb_6x18[2][14] = temp4_3x1[2];
	jLFlb_6x18[3][14] = temp4_3x1[3];
	jLFlb_6x18[4][14] = axis_lhr[1];
	jLFlb_6x18[5][14] = axis_lhr[2];
	jLFlb_6x18[6][14] = axis_lhr[3];

	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][15] = temp4_3x1[1];
	jLFlb_6x18[2][15] = temp4_3x1[2];
	jLFlb_6x18[3][15] = temp4_3x1[3];
	jLFlb_6x18[4][15] = axis_lhp[1];
	jLFlb_6x18[5][15] = axis_lhp[2];
	jLFlb_6x18[6][15] = axis_lhp[3];

	diff_vv(pLFC,3,pLKN, temp3_3x1);
	cross(1.f,axis_lkn, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][16] = temp4_3x1[1];
	jLFlb_6x18[2][16] = temp4_3x1[2];
	jLFlb_6x18[3][16] = temp4_3x1[3];
	jLFlb_6x18[4][16] = axis_lkn[1];
	jLFlb_6x18[5][16] = axis_lkn[2];
	jLFlb_6x18[6][16] = axis_lkn[3];

	diff_vv(pLFC,3,pLANK, temp3_3x1);
	cross(1.f,axis_lap, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][17] = temp4_3x1[1];
	jLFlb_6x18[2][17] = temp4_3x1[2];
	jLFlb_6x18[3][17] = temp4_3x1[3];
	jLFlb_6x18[4][17] = axis_lap[1];
	jLFlb_6x18[5][17] = axis_lap[2];
	jLFlb_6x18[6][17] = axis_lap[3];

	cross(1.f,axis_lar, temp3_3x1, temp4_3x1);
	jLFlb_6x18[1][18] = temp4_3x1[1];
	jLFlb_6x18[2][18] = temp4_3x1[2];
	jLFlb_6x18[3][18] = temp4_3x1[3];
	jLFlb_6x18[4][18] = axis_lar[1];
	jLFlb_6x18[5][18] = axis_lar[2];
	jLFlb_6x18[6][18] = axis_lar[3];


	pCOM_3x1[1] = ((m_PEL*cPEL[1] +m_TOR*cTOR[1] + m_ULEG*(cRUL[1]+cLUL[1]) + m_LLEG*(cRLL[1]+cLLL[1]) +m_FOOT*(cRF[1]+cLF[1]) + m_UARM*(cRUA[1]+cLUA[1]) + m_LARM*(cRLA[1]+cLLA[1]))/m_TOTAL);
	pCOM_3x1[2] = ((m_PEL*cPEL[2] +m_TOR*cTOR[2] + m_ULEG*(cRUL[2]+cLUL[2]) + m_LLEG*(cRLL[2]+cLLL[2]) +m_FOOT*(cRF[2]+cLF[2]) + m_UARM*(cRUA[2]+cLUA[2]) + m_LARM*(cRLA[2]+cLLA[2]))/m_TOTAL);
	pCOM_3x1[3] = ((m_PEL*cPEL[3] +m_TOR*cTOR[3] + m_ULEG*(cRUL[3]+cLUL[3]) + m_LLEG*(cRLL[3]+cLLL[3]) +m_FOOT*(cRF[3]+cLF[3]) + m_UARM*(cRUA[3]+cLUA[3]) + m_LARM*(cRLA[3]+cLLA[3]))/m_TOTAL);

	pPC_3x1[1] = Qlb_19x1[1];
	pPC_3x1[2] = Qlb_19x1[2];
	pPC_3x1[3] = Qlb_19x1[3];

	qPEL_4x1[1] = Qlb_19x1[4];
	qPEL_4x1[2] = Qlb_19x1[5];
	qPEL_4x1[3] = Qlb_19x1[6];
	qPEL_4x1[4] = Qlb_19x1[7];

	pRFC_3x1[1] = pRFC[1];
	pRFC_3x1[2] = pRFC[2];
	pRFC_3x1[3] = pRFC[3];

	pLFC_3x1[1] = pLFC[1];
	pLFC_3x1[2] = pLFC[2];
	pLFC_3x1[3] = pLFC[3];

	DC2QT((const float**)_dcRF_3x3, qRF_4x1);
	DC2QT((const float**)_dcLF_3x3, qLF_4x1);

	return 0;
}



int QT2DC(const float QT_4x1[5], float **DC_3x3)		// convert a quaternion to a direction cosine matrix
{
	float q0 = QT_4x1[1];
	float q1 = QT_4x1[2];
	float q2 = QT_4x1[3];
	float q3 = QT_4x1[4];

	DC_3x3[1][1] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
	DC_3x3[1][2] = 2*(q1*q2-q0*q3);
	DC_3x3[1][3] = 2*(q1*q3+q0*q2);
	DC_3x3[2][1] = 2*(q1*q2+q0*q3);
	DC_3x3[2][2] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
	DC_3x3[2][3] = 2*(q2*q3-q0*q1);
	DC_3x3[3][1] = 2*(q1*q3-q0*q2);
	DC_3x3[3][2] = 2*(q2*q3+q0*q1);
	DC_3x3[3][3] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	return 0;
}



int DC2QT(const float **DC_3x3, float QT_4x1[5])
{
	unsigned int index;
	float q_sq[4], temp, max;

	q_sq[0] = 0.25f*(1.f + DC_3x3[1][1] + DC_3x3[2][2] + DC_3x3[3][3]);
	q_sq[1] = 0.25f*(1.f + DC_3x3[1][1] - DC_3x3[2][2] - DC_3x3[3][3]);
	q_sq[2] = 0.25f*(1.f - DC_3x3[1][1] + DC_3x3[2][2] - DC_3x3[3][3]);
	q_sq[3] = 0.25f*(1.f - DC_3x3[1][1] - DC_3x3[2][2] + DC_3x3[3][3]);

	findmax(q_sq, 4, &max, &index);

	switch(index)
	{
	case 0:
		QT_4x1[1] = (float)sqrt(max);
		temp = 4.f*QT_4x1[1];
		QT_4x1[2] = (DC_3x3[3][2]-DC_3x3[2][3])/temp;
		QT_4x1[3] = (DC_3x3[1][3]-DC_3x3[3][1])/temp;
		QT_4x1[4] = (DC_3x3[2][1]-DC_3x3[1][2])/temp;
		break;
	case 1:
		QT_4x1[2] = (float)sqrt(max);
		temp = 4.f*QT_4x1[2];
		QT_4x1[1] = (DC_3x3[3][2]-DC_3x3[2][3])/temp;
		QT_4x1[3] = (DC_3x3[2][1]+DC_3x3[1][2])/temp;
		QT_4x1[4] = (DC_3x3[1][3]+DC_3x3[3][1])/temp;
		break;
	case 2:
		QT_4x1[3] = (float)sqrt(max);
		temp = 4.f*QT_4x1[3];
		QT_4x1[1] = (DC_3x3[1][3]-DC_3x3[3][1])/temp;
		QT_4x1[2] = (DC_3x3[2][1]+DC_3x3[1][2])/temp;
		QT_4x1[4] = (DC_3x3[3][2]+DC_3x3[2][3])/temp;
		break;
	case 3:
		QT_4x1[4] = (float)sqrt(max);
		temp = 4.f*QT_4x1[4];
		QT_4x1[1] = (DC_3x3[2][1]-DC_3x3[1][2])/temp;
		QT_4x1[2] = (DC_3x3[1][3]-DC_3x3[3][1])/temp;
		QT_4x1[3] = (DC_3x3[3][2]+DC_3x3[2][3])/temp;
		break;
	}

	return 0;
}


int QTdel(const float *des_QT_4x1, const float *QT_4x1, float *result_3x1) // delta quaternion
{
	float q0;
	float des_q0 = des_QT_4x1[1];
	float q[4], des_q[4];
	float temp[4];
	float qt_4x1[5];

	QT2DC((const float*)QT_4x1, _TEMP1_18x18);
	DC2QT((const float**)_TEMP1_18x18, qt_4x1);

	q0 = qt_4x1[1];
	q[1] = qt_4x1[2];
	q[2] = qt_4x1[3];
	q[3] = qt_4x1[4];

	des_q[1] = des_QT_4x1[2];
	des_q[2] = des_QT_4x1[3];
	des_q[3] = des_QT_4x1[4];

	cross(1.f, q, des_q, temp);

	result_3x1[1] = q0*des_q[1] - des_q0*q[1] - temp[1];
	result_3x1[2] = q0*des_q[2] - des_q0*q[2] - temp[2];
	result_3x1[3] = q0*des_q[3] - des_q0*q[3] - temp[3];

	return 0;
}

int Wmatrix(const float *w_3x1, float **wmatrix_4x4)
{
	wmatrix_4x4[1][1] = 0.;
	wmatrix_4x4[1][2] = -w_3x1[1];
	wmatrix_4x4[1][3] = -w_3x1[2];
	wmatrix_4x4[1][4] = -w_3x1[3];

	wmatrix_4x4[2][1] = w_3x1[1];
	wmatrix_4x4[2][2] = 0.;
	wmatrix_4x4[2][3] = -w_3x1[3];
	wmatrix_4x4[2][4] = w_3x1[2];

	wmatrix_4x4[3][1] = w_3x1[2];
	wmatrix_4x4[3][2] = w_3x1[3];
	wmatrix_4x4[3][3] = 0.;
	wmatrix_4x4[3][4] = -w_3x1[1];

	wmatrix_4x4[4][1] = w_3x1[3];
	wmatrix_4x4[4][2] = -w_3x1[2];
	wmatrix_4x4[4][3] = w_3x1[1];
	wmatrix_4x4[4][4] = 0.;
	
	return 0;
}


int QThat(const float *QT_4x1, float **qthat_4x3)
{
	qthat_4x3[1][1] = -QT_4x1[2];
	qthat_4x3[1][2] = -QT_4x1[3];
	qthat_4x3[1][3] = -QT_4x1[4];

	qthat_4x3[2][1] = QT_4x1[1];
	qthat_4x3[2][2] = QT_4x1[4];
	qthat_4x3[2][3] = -QT_4x1[3];

	qthat_4x3[3][1] = -QT_4x1[4];
	qthat_4x3[3][2] = QT_4x1[1];
	qthat_4x3[3][3] = QT_4x1[2];

	qthat_4x3[4][1] = QT_4x1[3];
	qthat_4x3[4][2] = -QT_4x1[2];
	qthat_4x3[4][3] = QT_4x1[1];

	return 0;
}


int Crsmatrix(const float *vector_3x1, float **matrix_3x3)	// skew-symmetric cross product matrix
{
	matrix_3x3[1][1] = 0.;
	matrix_3x3[1][2] = -vector_3x1[3];
	matrix_3x3[1][3] = vector_3x1[2];

	matrix_3x3[2][1] = vector_3x1[3];
	matrix_3x3[2][2] = 0.;
	matrix_3x3[2][3] = -vector_3x1[1];

	matrix_3x3[3][1] = -vector_3x1[2];
	matrix_3x3[3][2] = vector_3x1[1];
	matrix_3x3[3][3] = 0.;

	return 0;
}

int RX(float _theta, float **R_3x3)
{
	R_3x3[1][1] = 1.f;
	R_3x3[1][2] = 0;
	R_3x3[1][3] = 0;

	R_3x3[2][1] = 0;
	R_3x3[2][2] = (float)cos(_theta);
	R_3x3[2][3] = -(float)sin(_theta);

	R_3x3[3][1] = 0;
	R_3x3[3][2] = -R_3x3[2][3];
	R_3x3[3][3] = R_3x3[2][2];

	return 0;
}


int RY(float _theta, float **R_3x3)
{
	R_3x3[1][1] = (float)cos(_theta);
	R_3x3[1][2] = 0;
	R_3x3[1][3] = (float)sin(_theta);

	R_3x3[2][1] = 0;
	R_3x3[2][2] = 1.f;
	R_3x3[2][3] = 0;

	R_3x3[3][1] = -R_3x3[1][3];
	R_3x3[3][2] = 0;
	R_3x3[3][3] = R_3x3[1][1];

	return 0;
}


int RZ(float _theta, float **R_3x3)
{
	R_3x3[1][1] = (float)cos(_theta);
	R_3x3[1][2] = -(float)sin(_theta);
	R_3x3[1][3] = 0;

	R_3x3[2][1] = -R_3x3[1][2];
	R_3x3[2][2] = R_3x3[1][1];
	R_3x3[2][3] = 0;

	R_3x3[3][1] = 0;
	R_3x3[3][2] = 0;
	R_3x3[3][3] = 1.f;

	return 0;
}




int JAVLC(float x, float xp, float xpp_ref, float xpmax, float xppmax, float xmin, float xmax, float margin, float *result_)	// Joint-range, acc and vel limit controller
{
	
	float Klv = 1.f/DT;
	
	float temp = x + xp*DT;
	float temp2 = DT*DT;

	float Amin = 2.f*(xmin+margin-temp)/temp2; // (x+xp*dt+0.5*a*dt^2)-xmin > margin
	float Amax = 2.f*(xmax-margin-temp)/temp2; // xmax-(x+xp*dt+0.5*a*dt^2) > margin

	float A, V;
	float xnext, xpnext;
	float r_max, r_min;
	float Vmax, Vmin;

	if(xpp_ref < Amin)
		A = Amin;
	else if(xpp_ref > Amax)
		A = Amax;
	else
		A = xpp_ref;

	A = FMIN(1.f, (float)(xppmax/fabs(A)))*A;
	

	xnext = temp + 0.5f*A*temp2;
	xpnext = xp + A*DT;

	r_max = xmax - xnext;
	r_min = xnext - xmin;

	if(r_max > margin)
		Vmax = (float)sqrt(2.f*xppmax*(r_max-margin));
	else
		Vmax = 0.;

	if(r_min > margin)
		Vmin = -(float)sqrt(2.f*xppmax*(r_min-margin));
	else
		Vmin = 0.;

	Vmax = FMIN(1.f, (float)(xpmax/fabs(Vmax)))*Vmax;
	Vmin = FMIN(1.f, (float)(xpmax/fabs(Vmin)))*Vmin;

	V = xpnext; //V = xpp/Klv - xpref + Klp/Klv*(xref-x);

	if(V > Vmax)
		V = Vmax;
	else if(V < Vmin)
		V = Vmin;

	A = Klv*(V-xp);
	*result_ = FMIN(1.f, (float)(xppmax/fabs(A)))*A; // *result_ = xpp
	
	if(fabs(*result_-xpp_ref) < 5e-5)
		return 0;	// not limited
	else
		return 1;	// limited

}


int ZLC(float Cx, float Cxp, float Cxpp_ref, float Cz, float Czpp, float ZMPxmin, float ZMPxmax, float ZMPz, float margin, float *result_)	// ZMP-lmited COM controller
{
	float Kv = 1.f/DT;
	float g = 9.81f;
	float temp = (Czpp+g)/(ZMPz-Cz);
	float temp2 = (float)sqrt(-temp);

	float Amax = (ZMPxmin-Cx+margin)*temp;
	float Amin = (ZMPxmax-Cx-margin)*temp;

	float A, V;
	float Vmax, Vmin;
	float Cx_next, Cxp_next;
	float r_max, r_min;

	if(Cxpp_ref > Amax)
		A = Amax;
	else if(Cxpp_ref < Amin)
		A = Amin;
	else
		A = Cxpp_ref;

	Cx_next = Cx + Cxp*DT + 0.5f*A*DT*DT;
	Cxp_next = Cxp + A*DT;

	r_max = ZMPxmax - Cx_next;
	r_min = Cx_next - ZMPxmin;

	if(r_max > margin)
		Vmax = temp2*(r_max-margin);
	else
		Vmax = 0.;

	if(r_min > margin)
		Vmin = -temp2*(r_min-margin);
	else
		Vmin = 0;

	V = Cxp_next;

	if(V > Vmax)
		V = Vmax;
	else if(V < Vmin)
		V= Vmin;


	A = Kv*(V-Cxp);

	if(A > Amax)
		A = Amax;
	else if(A < Amin)
		A = Amin;


	*result_ = A;		// Cxpp = A

	if(fabs(A-Cxpp_ref) < 5e-5)
		return 0;		// not limited
	else
		return 1;		// limited

}



int MocapOnestep(unsigned int n)
{
	unsigned int i, j;

	float Qlbpp_18x1[19], Qubpp_11x1[12], Qlbpp_const_18x1[19], Qlbpp_task_18x1[19];
	float pPC_3x1[4], qPEL_4x1[5];
	float pRFC_3x1[4], qRF_4x1[5];
	float pLFC_3x1[4], qLF_4x1[5];
	float pCOM_3x1[4];

	float Xaug_18x1[19];
	float Xsupp_12x1[13];
	float Xcom_2x1[3];
	float Xpel_3x1[4];
	float Xpcz_1x1;
	
	float vCOM_3x1[4], ref_aCOM_3x1[4];
	float temp1_6x1[7], temp2_6x1[7], temp3_3x1[4], temp4_3x1[4], temp5_4x1[5], temp6_4x1[5], temp7_4x1[5], temp8_18x1[19];
	float wPELnorm, temp;
	
	BOOL isLimited = FALSE;

	const float des_pRFC_3x1[4] = {0., 0., -L_PEL*0.5f, 0.};
	const float des_pLFC_3x1[4] = {0., 0., L_PEL*0.5f, 0.};
	const float des_qF[5] = {0., 1., 0., 0., 0.};
	const float des_pCOM_3x1[4] = {0., _pCOM0_3x1[1], _pCOM0_3x1[2], _pCOM0_3x1[3]};
	const float jPCZlb_1x18[19] = {0., 0., 0., 1.f, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};

/*
	_Qlb_19x1[8] = Joint[RHY].RefAngleCurrent*D2R;
	_Qlb_19x1[9] = Joint[RHR].RefAngleCurrent*D2R;
	_Qlb_19x1[10] = Joint[RHP].RefAngleCurrent*D2R;
	_Qlb_19x1[11] = Joint[RKN].RefAngleCurrent*D2R;
	_Qlb_19x1[12] = Joint[RAP].RefAngleCurrent*D2R;
	_Qlb_19x1[13] = Joint[RAR].RefAngleCurrent*D2R;
	_Qlb_19x1[14] = Joint[LHY].RefAngleCurrent*D2R;
	_Qlb_19x1[15] = Joint[LHR].RefAngleCurrent*D2R;
	_Qlb_19x1[16] = Joint[LHP].RefAngleCurrent*D2R;
	_Qlb_19x1[17] = Joint[LKN].RefAngleCurrent*D2R;
	_Qlb_19x1[18] = Joint[LAP].RefAngleCurrent*D2R;
	_Qlb_19x1[19] = Joint[LAR].RefAngleCurrent*D2R;

	_Qub_11x1[1] = Joint[WST].RefAngleCurrent*D2R;
	_Qub_11x1[2] = Joint[RSP].RefAngleCurrent*D2R;
	_Qub_11x1[3] = Joint[RSR].RefAngleCurrent*D2R;
	_Qub_11x1[4] = Joint[RSY].RefAngleCurrent*D2R;
	_Qub_11x1[5] = Joint[REB].RefAngleCurrent*D2R;
	_Qub_11x1[6] = Joint[RWY].RefAngleCurrent*D2R;
	_Qub_11x1[7] = Joint[LSP].RefAngleCurrent*D2R;
	_Qub_11x1[8] = Joint[LSR].RefAngleCurrent*D2R;
	_Qub_11x1[9] = Joint[LSY].RefAngleCurrent*D2R;
	_Qub_11x1[10] = Joint[LEB].RefAngleCurrent*D2R;
	_Qub_11x1[11] = Joint[LWY].RefAngleCurrent*D2R;
*/

	if(n == 0)
		return 0;

	MocapKine(_Qlb_19x1, _Qub_11x1, pPC_3x1, qPEL_4x1, _jRFlb_6x18, pRFC_3x1, qRF_4x1, _jLFlb_6x18, pLFC_3x1, qLF_4x1, _jCOMlb_3x18, _jCOMub_3x11, pCOM_3x1);
	
	diff_mm((const float**)_jRFlb_6x18,6,18, (const float**)_jRFlb_old_6x18, _jRFlbp_6x18);
	mult_sm((const float**)_jRFlbp_6x18,6,18, (1.f/DT), _jRFlbp_6x18);

	diff_mm((const float**)_jLFlb_6x18,6,18, (const float**)_jLFlb_old_6x18, _jLFlbp_6x18);
	mult_sm((const float**)_jLFlbp_6x18,6,18, (1.f/DT), _jLFlbp_6x18);

	diff_mm((const float**)_jCOMlb_3x18,3,18, (const float**)_jCOMlb_old_3x18, _jCOMlbp_3x18);
	mult_sm((const float**)_jCOMlbp_3x18,3,18, (1.f/DT), _jCOMlbp_3x18);

	diff_mm((const float**)_jCOMub_3x11,3,11, (const float**)_jCOMub_old_3x11, _jCOMubp_3x11);
	mult_sm((const float**)_jCOMubp_3x11,3,11, (1.f/DT), _jCOMubp_3x11);

	subs_m((const float**)_jRFlb_6x18,6,18, _jRFlb_old_6x18);
	subs_m((const float**)_jLFlb_6x18,6,18, _jLFlb_old_6x18);
	subs_m((const float**)_jCOMlb_3x18,3,18, _jCOMlb_old_3x18);
	subs_m((const float**)_jCOMub_3x11,3,11, _jCOMub_old_3x11);
	
	for(i=1; i<=18; i++)
	{
		_jAUG_18x18[1][i] = _jRFlb_6x18[1][i];
		_jAUG_18x18[2][i] = _jRFlb_6x18[2][i];
		_jAUG_18x18[3][i] = _jRFlb_6x18[3][i];
		_jAUG_18x18[4][i] = _jRFlb_6x18[4][i];
		_jAUG_18x18[5][i] = _jRFlb_6x18[5][i];
		_jAUG_18x18[6][i] = _jRFlb_6x18[6][i];			

		_jAUG_18x18[7][i] = _jLFlb_6x18[1][i];
		_jAUG_18x18[8][i] = _jLFlb_6x18[2][i];
		_jAUG_18x18[9][i] = _jLFlb_6x18[3][i];
		_jAUG_18x18[10][i] = _jLFlb_6x18[4][i];
		_jAUG_18x18[11][i] = _jLFlb_6x18[5][i];
		_jAUG_18x18[12][i] = _jLFlb_6x18[6][i];

		_jAUG_18x18[13][i] = _jCOMlb_3x18[1][i];
		_jAUG_18x18[14][i] = _jCOMlb_3x18[2][i];

		_jAUG_18x18[15][i] = _jPELlb_att_3x18[1][i];
		_jAUG_18x18[16][i] = _jPELlb_att_3x18[2][i];
		_jAUG_18x18[17][i] = _jPELlb_att_3x18[3][i];
		
		_jAUG_18x18[18][i] = jPCZlb_1x18[i];
	}

	if(n > _MOTION_LENGTH)
	{
		JAVLC(_Qub_11x1[1], _Qubp_11x1[1],  KV_MOCAP*(-_Qubp_11x1[1])+KP_MOCAP*(_Qub0_11x1[1]-_Qub_11x1[1]),   WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &Qubpp_11x1[1]);
		JAVLC(_Qub_11x1[2], _Qubp_11x1[2],  KV_MOCAP*(-_Qubp_11x1[2])+KP_MOCAP*(_Qub0_11x1[2]-_Qub_11x1[2]),   RSPpmax, RSPppmax, RSPmin, RSPmax, D2R, &Qubpp_11x1[2]);
		JAVLC(_Qub_11x1[3], _Qubp_11x1[3],  KV_MOCAP*(-_Qubp_11x1[3])+KP_MOCAP*(_Qub0_11x1[3]-_Qub_11x1[3]),   RSRpmax, RSRppmax, RSRmin, RSRmax, D2R, &Qubpp_11x1[3]);
		JAVLC(_Qub_11x1[4], _Qubp_11x1[4],  KV_MOCAP*(-_Qubp_11x1[4])+KP_MOCAP*(_Qub0_11x1[4]-_Qub_11x1[4]),   RSYpmax, RSYppmax, RSYmin, RSYmax, D2R, &Qubpp_11x1[4]);
		JAVLC(_Qub_11x1[5], _Qubp_11x1[5],  KV_MOCAP*(-_Qubp_11x1[5])+KP_MOCAP*(_Qub0_11x1[5]-_Qub_11x1[5]),   REPpmax, REPppmax, REPmin, REPmax, D2R, &Qubpp_11x1[5]);
		JAVLC(_Qub_11x1[6], _Qubp_11x1[6],  KV_MOCAP*(-_Qubp_11x1[6])+KP_MOCAP*(_Qub0_11x1[6]-_Qub_11x1[6]),   REYpmax, REYppmax, REYmin, REYmax, D2R, &Qubpp_11x1[6]);
		JAVLC(_Qub_11x1[7], _Qubp_11x1[7],  KV_MOCAP*(-_Qubp_11x1[7])+KP_MOCAP*(_Qub0_11x1[7]-_Qub_11x1[7]),   LSPpmax, LSPppmax, LSPmin, LSPmax, D2R, &Qubpp_11x1[7]);
		JAVLC(_Qub_11x1[8], _Qubp_11x1[8],  KV_MOCAP*(-_Qubp_11x1[8])+KP_MOCAP*(_Qub0_11x1[8]-_Qub_11x1[8]),   LSRpmax, LSRppmax, LSRmin, LSRmax, D2R, &Qubpp_11x1[8]);
		JAVLC(_Qub_11x1[9], _Qubp_11x1[9],  KV_MOCAP*(-_Qubp_11x1[9])+KP_MOCAP*(_Qub0_11x1[9]-_Qub_11x1[9]),   LSYpmax, LSYppmax, LSYmin, LSYmax, D2R, &Qubpp_11x1[9]);
		JAVLC(_Qub_11x1[10],_Qubp_11x1[10], KV_MOCAP*(-_Qubp_11x1[10])+KP_MOCAP*(_Qub0_11x1[10]-_Qub_11x1[10]), LEPpmax, LEPppmax, LEPmin, LEPmax, D2R, &Qubpp_11x1[10]);
		JAVLC(_Qub_11x1[11],_Qubp_11x1[11], KV_MOCAP*(-_Qubp_11x1[11])+KP_MOCAP*(_Qub0_11x1[11]-_Qub_11x1[11]), LEYpmax, LEYppmax, LEYmin, LEYmax, D2R, &Qubpp_11x1[11]);
	}
	else
	{
		JAVLC(_Qub_11x1[1], _Qubp_11x1[1],  _des_WSTpp[n]+KV_MOCAP*(_des_WSTp[n]-_Qubp_11x1[1])+KP_MOCAP*(_des_WST[n]-_Qub_11x1[1]),   WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &Qubpp_11x1[1]);
		JAVLC(_Qub_11x1[2], _Qubp_11x1[2],  _des_RSPpp[n]+KV_MOCAP*(_des_RSPp[n]-_Qubp_11x1[2])+KP_MOCAP*(_des_RSP[n]-_Qub_11x1[2]),   RSPpmax, RSPppmax, RSPmin, RSPmax, D2R, &Qubpp_11x1[2]);
		JAVLC(_Qub_11x1[3], _Qubp_11x1[3],  _des_RSRpp[n]+KV_MOCAP*(_des_RSRp[n]-_Qubp_11x1[3])+KP_MOCAP*(_des_RSR[n]-_Qub_11x1[3]),   RSRpmax, RSRppmax, RSRmin, RSRmax, D2R, &Qubpp_11x1[3]);
		JAVLC(_Qub_11x1[4], _Qubp_11x1[4],  _des_RSYpp[n]+KV_MOCAP*(_des_RSYp[n]-_Qubp_11x1[4])+KP_MOCAP*(_des_RSY[n]-_Qub_11x1[4]),   RSYpmax, RSYppmax, RSYmin, RSYmax, D2R, &Qubpp_11x1[4]);
		JAVLC(_Qub_11x1[5], _Qubp_11x1[5],  _des_REPpp[n]+KV_MOCAP*(_des_REPp[n]-_Qubp_11x1[5])+KP_MOCAP*(_des_REP[n]-_Qub_11x1[5]),   REPpmax, REPppmax, REPmin, REPmax, D2R, &Qubpp_11x1[5]);
		JAVLC(_Qub_11x1[6], _Qubp_11x1[6],  _des_REYpp[n]+KV_MOCAP*(_des_REYp[n]-_Qubp_11x1[6])+KP_MOCAP*(_des_REY[n]-_Qub_11x1[6]),   REYpmax, REYppmax, REYmin, REYmax, D2R, &Qubpp_11x1[6]);
		JAVLC(_Qub_11x1[7], _Qubp_11x1[7],  _des_LSPpp[n]+KV_MOCAP*(_des_LSPp[n]-_Qubp_11x1[7])+KP_MOCAP*(_des_LSP[n]-_Qub_11x1[7]),   LSPpmax, LSPppmax, LSPmin, LSPmax, D2R, &Qubpp_11x1[7]);
		JAVLC(_Qub_11x1[8], _Qubp_11x1[8],  _des_LSRpp[n]+KV_MOCAP*(_des_LSRp[n]-_Qubp_11x1[8])+KP_MOCAP*(_des_LSR[n]-_Qub_11x1[8]),   LSRpmax, LSRppmax, LSRmin, LSRmax, D2R, &Qubpp_11x1[8]);
		JAVLC(_Qub_11x1[9], _Qubp_11x1[9],  _des_LSYpp[n]+KV_MOCAP*(_des_LSYp[n]-_Qubp_11x1[9])+KP_MOCAP*(_des_LSY[n]-_Qub_11x1[9]),   LSYpmax, LSYppmax, LSYmin, LSYmax, D2R, &Qubpp_11x1[9]);
		JAVLC(_Qub_11x1[10],_Qubp_11x1[10], _des_LEPpp[n]+KV_MOCAP*(_des_LEPp[n]-_Qubp_11x1[10])+KP_MOCAP*(_des_LEP[n]-_Qub_11x1[10]), LEPpmax, LEPppmax, LEPmin, LEPmax, D2R, &Qubpp_11x1[10]);
		JAVLC(_Qub_11x1[11],_Qubp_11x1[11], _des_LEYpp[n]+KV_MOCAP*(_des_LEYp[n]-_Qubp_11x1[11])+KP_MOCAP*(_des_LEY[n]-_Qub_11x1[11]), LEYpmax, LEYppmax, LEYmin, LEYmax, D2R, &Qubpp_11x1[11]);
	}

	
	//------------ Supporting
	mult_smv(-KV_SUPP, (const float**)_jRFlb_6x18,6,18, _Qlbp_18x1, temp1_6x1);
	diff_vv(des_pRFC_3x1,3, pRFC_3x1, temp3_3x1);
	QTdel(des_qF, qRF_4x1, temp4_3x1);		
	mult_mv((const float**)_jRFlbp_6x18,6,18, _Qlbp_18x1, temp2_6x1);

	Xsupp_12x1[1] = temp1_6x1[1] + KP_SUPP*temp3_3x1[1] - temp2_6x1[1];
	Xsupp_12x1[2] = temp1_6x1[2] + KP_SUPP*temp3_3x1[2] - temp2_6x1[2];
	Xsupp_12x1[3] = temp1_6x1[3] + KP_SUPP*temp3_3x1[3] - temp2_6x1[3];
	Xsupp_12x1[4] = temp1_6x1[4] + KP_SUPP*temp4_3x1[1] - temp2_6x1[4];
	Xsupp_12x1[5] = temp1_6x1[5] + KP_SUPP*temp4_3x1[2] - temp2_6x1[5];
	Xsupp_12x1[6] = temp1_6x1[6] + KP_SUPP*temp4_3x1[3] - temp2_6x1[6];

	mult_smv(-KV_SUPP, (const float**)_jLFlb_6x18,6,18, _Qlbp_18x1, temp1_6x1);
	diff_vv(des_pLFC_3x1,3, pLFC_3x1, temp3_3x1);
	QTdel(des_qF, qLF_4x1, temp4_3x1);
	mult_mv((const float**)_jLFlbp_6x18,6,18, _Qlbp_18x1, temp2_6x1);

	Xsupp_12x1[7] = temp1_6x1[1] + KP_SUPP*temp3_3x1[1] - temp2_6x1[1];
	Xsupp_12x1[8] = temp1_6x1[2] + KP_SUPP*temp3_3x1[2] - temp2_6x1[2];
	Xsupp_12x1[9] = temp1_6x1[3] + KP_SUPP*temp3_3x1[3] - temp2_6x1[3];
	Xsupp_12x1[10] = temp1_6x1[4] + KP_SUPP*temp4_3x1[1] - temp2_6x1[4];
	Xsupp_12x1[11] = temp1_6x1[5] + KP_SUPP*temp4_3x1[2] - temp2_6x1[5];
	Xsupp_12x1[12] = temp1_6x1[6] + KP_SUPP*temp4_3x1[3] - temp2_6x1[6];
	//-----------------

	if(n > _MOTION_LENGTH)
	{
		//------------ COM				
		mult_mv((const float**)_jCOMlb_3x18,3,18, _Qlbp_18x1, temp3_3x1);
		mult_mv((const float**)_jCOMub_3x11,3,11, _Qubp_11x1, temp4_3x1);
		sum_vv(temp3_3x1,3, temp4_3x1, vCOM_3x1);

		diff_vv(_pCOM0_3x1,3, pCOM_3x1, temp3_3x1);
		sum_svsv(-KV_COM_RECOV,vCOM_3x1,3, KP_COM_RECOV,temp3_3x1, ref_aCOM_3x1);

		ZLC(pCOM_3x1[1], vCOM_3x1[1], ref_aCOM_3x1[1], pCOM_3x1[3], 0., ZMPXMIN, ZMPXMAX, 0., 0.01f, &ref_aCOM_3x1[1]);
		ZLC(pCOM_3x1[2], vCOM_3x1[2], ref_aCOM_3x1[2], pCOM_3x1[3], 0., ZMPYMIN, ZMPYMAX, 0., 0.01f, &ref_aCOM_3x1[2]);

		mult_mv((const float**)_jCOMlbp_3x18,3,18, _Qlbp_18x1, temp3_3x1);
		mult_mv((const float**)_jCOMubp_3x11,3,11, _Qubp_11x1, temp4_3x1);
		sum_vv(temp3_3x1,3,temp4_3x1, temp3_3x1);
		mult_mv((const float**)_jCOMub_3x11, 3,11, Qubpp_11x1, temp4_3x1);
		Xcom_2x1[1] = ref_aCOM_3x1[1] - temp3_3x1[1] - temp4_3x1[1];
		Xcom_2x1[2] = ref_aCOM_3x1[2] - temp3_3x1[2] - temp4_3x1[2];
		//-----------------

		//--------- PEL
		temp5_4x1[1] = 1.f;
		temp5_4x1[2] = 0.;
		temp5_4x1[3] = 0.;
		temp5_4x1[4] = 0.;
		QTdel(temp5_4x1, &_Qlb_19x1[3], temp3_3x1);
		Xpel_3x1[1] = KV_PEL_RECOV*(0.f-_Qlbp_18x1[4]) + KP_PEL_RECOV*temp3_3x1[1];
		Xpel_3x1[2] = KV_PEL_RECOV*(0.f-_Qlbp_18x1[5]) + KP_PEL_RECOV*temp3_3x1[2];
		Xpel_3x1[3] = KV_PEL_RECOV*(0.f-_Qlbp_18x1[6]) + KP_PEL_RECOV*temp3_3x1[3];
		//-----------------

		//--------- PCz
		Xpcz_1x1 = KV_PCZ_RECOV*(-_Qlbp_18x1[3]) + KP_PCZ_RECOV*(_pPCz0 - _Qlb_19x1[3]);
		//-----------------
	}
	else
	{
		//------------ COM				
		mult_mv((const float**)_jCOMlb_3x18,3,18, _Qlbp_18x1, temp3_3x1);
		mult_mv((const float**)_jCOMub_3x11,3,11, _Qubp_11x1, temp4_3x1);
		sum_vv(temp3_3x1,3, temp4_3x1, vCOM_3x1);

		diff_vv(des_pCOM_3x1,3, pCOM_3x1, temp3_3x1);
		sum_svsv(-KV_COM,vCOM_3x1,3, KP_COM,temp3_3x1, ref_aCOM_3x1);

		ZLC(pCOM_3x1[1], vCOM_3x1[1], ref_aCOM_3x1[1], pCOM_3x1[3], 0., ZMPXMIN, ZMPXMAX, 0., 0.01f, &ref_aCOM_3x1[1]);
		ZLC(pCOM_3x1[2], vCOM_3x1[2], ref_aCOM_3x1[2], pCOM_3x1[3], 0., ZMPYMIN, ZMPYMAX, 0., 0.01f, &ref_aCOM_3x1[2]);

		mult_mv((const float**)_jCOMlbp_3x18,3,18, _Qlbp_18x1, temp3_3x1);
		mult_mv((const float**)_jCOMubp_3x11,3,11, _Qubp_11x1, temp4_3x1);
		sum_vv(temp3_3x1,3,temp4_3x1, temp3_3x1);
		mult_mv((const float**)_jCOMub_3x11, 3,11, Qubpp_11x1, temp4_3x1);
		Xcom_2x1[1] = ref_aCOM_3x1[1] - temp3_3x1[1] - temp4_3x1[1];
		Xcom_2x1[2] = ref_aCOM_3x1[2] - temp3_3x1[2] - temp4_3x1[2];
		//-----------------

		//--------- PEL
		QTdel(_des_qPEL[n], &_Qlb_19x1[3], temp3_3x1);
		Xpel_3x1[1] = KV_PEL*(_des_wPEL[n][1]-_Qlbp_18x1[4]) + KP_PEL*temp3_3x1[1];
		Xpel_3x1[2] = KV_PEL*(_des_wPEL[n][2]-_Qlbp_18x1[5]) + KP_PEL*temp3_3x1[2];
		Xpel_3x1[3] = KV_PEL*(_des_wPEL[n][3]-_Qlbp_18x1[6]) + KP_PEL*temp3_3x1[3];
		//-----------------

		//--------- PCz
		Xpcz_1x1 = _des_PCzpp[n] + KV_PCZ*(_des_PCzp[n]-_Qlbp_18x1[3]) + KP_PCZ*(_des_PCz[n]-_Qlb_19x1[3]);
		//-----------------
	}
	

	Xaug_18x1[1] = Xsupp_12x1[1];
	Xaug_18x1[2] = Xsupp_12x1[2];
	Xaug_18x1[3] = Xsupp_12x1[3];
	Xaug_18x1[4] = Xsupp_12x1[4];
	Xaug_18x1[5] = Xsupp_12x1[5];
	Xaug_18x1[6] = Xsupp_12x1[6];
	Xaug_18x1[7] = Xsupp_12x1[7];
	Xaug_18x1[8] = Xsupp_12x1[8];
	Xaug_18x1[9] = Xsupp_12x1[9];
	Xaug_18x1[10] = Xsupp_12x1[10];
	Xaug_18x1[11] = Xsupp_12x1[11];
	Xaug_18x1[12] = Xsupp_12x1[12];
	Xaug_18x1[13] = Xcom_2x1[1];
	Xaug_18x1[14] = Xcom_2x1[2];
	Xaug_18x1[15] = Xpel_3x1[1];
	Xaug_18x1[16] = Xpel_3x1[2];
	Xaug_18x1[17] = Xpel_3x1[3];
	Xaug_18x1[18] = Xpcz_1x1;

	subs_m((const float**)_jAUG_18x18,18,18, _jAUGi_18x18);
	subs_v(Xaug_18x1, 18, Qlbpp_18x1);

	gaussj_mod(_jAUGi_18x18,18,Qlbpp_18x1);
	
	isLimited = FALSE;
	if( JAVLC(_Qlb_19x1[8], _Qlbp_18x1[7], Qlbpp_18x1[7], RHYpmax, RHYppmax, RHYmin, RHYmax, D2R, &Qlbpp_18x1[7]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[9], _Qlbp_18x1[8], Qlbpp_18x1[8], RHRpmax, RHRppmax, RHRmin, RHRmax, D2R, &Qlbpp_18x1[8]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[10], _Qlbp_18x1[9], Qlbpp_18x1[9], RHPpmax, RHPppmax, RHPmin, RHPmax, D2R, &Qlbpp_18x1[9]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[11], _Qlbp_18x1[10], Qlbpp_18x1[10], RKNpmax, RKNppmax, RKNmin, RKNmax, D2R, &Qlbpp_18x1[10]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[12], _Qlbp_18x1[11], Qlbpp_18x1[11], RAPpmax, RAPppmax, RAPmin, RAPmax, D2R, &Qlbpp_18x1[11]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[13], _Qlbp_18x1[12], Qlbpp_18x1[12], RARpmax, RARppmax, RARmin, RARmax, D2R, &Qlbpp_18x1[12]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[14], _Qlbp_18x1[13], Qlbpp_18x1[13], LHYpmax, LHYppmax, LHYmin, LHYmax, D2R, &Qlbpp_18x1[13]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[15], _Qlbp_18x1[14], Qlbpp_18x1[14], LHRpmax, LHRppmax, LHRmin, LHRmax, D2R, &Qlbpp_18x1[14]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[16], _Qlbp_18x1[15], Qlbpp_18x1[15], LHPpmax, LHPppmax, LHPmin, LHPmax, D2R, &Qlbpp_18x1[15]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[17], _Qlbp_18x1[16], Qlbpp_18x1[16], LKNpmax, LKNppmax, LKNmin, LKNmax, D2R, &Qlbpp_18x1[16]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[18], _Qlbp_18x1[17], Qlbpp_18x1[17], LAPpmax, LAPppmax, LAPmin, LAPmax, D2R, &Qlbpp_18x1[17]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[19], _Qlbp_18x1[18], Qlbpp_18x1[18], LARpmax, LARppmax, LARmin, LARmax, D2R, &Qlbpp_18x1[18]) )
		isLimited = TRUE;


	if(isLimited)
	{
		subs_m((const float**)_jAUG_18x18,18,18, _jAUGi_18x18);
		//subs_v(Xaug_18x1, 18, Qlbpp_18x1);

		trans(1.f, (const float**)_jAUG_18x18,14,18, _TEMP3_18x18);
		mult_mm((const float**)_jAUG_18x18,14,18, (const float**)_TEMP3_18x18,14, _TEMP4_18x18);
		inv(_TEMP4_18x18,14);
		mult_mm((const float**)_TEMP3_18x18,18,14, (const float**)_TEMP4_18x18,14, _jCONST_DSPi_18x14);
		mult_mm((const float**)_jCONST_DSPi_18x14,18,14, (const float**)_jAUG_18x18,18, _TEMP3_18x18);
		diff_mm((const float**)_EYE_18,18,18, (const float**)_TEMP3_18x18, _NCONST_DSP_18x18);


		for(j=1; j<=18; j++)
		{
			_jTASK_18x18[1][j] = _jPELlb_att_3x18[1][j];
			_jTASK_18x18[2][j] = _jPELlb_att_3x18[2][j];
			_jTASK_18x18[3][j] = _jPELlb_att_3x18[3][j];
			_jTASK_18x18[4][j] = jPCZlb_1x18[j];
		}
		mult_mm((const float**)_jTASK_18x18,4,18, (const float**)_NCONST_DSP_18x18,18, _jTASKN_18x18);
		trans(1.f, (const float**)_jTASKN_18x18,4,18, _TEMP3_18x18);
		mult_mm((const float**)_jTASKN_18x18,4,18, (const float**)_TEMP3_18x18,4, _TEMP4_18x18);
		inv(_TEMP4_18x18,4);
		mult_mm((const float**)_TEMP3_18x18,18,4, (const float**)_TEMP4_18x18,4, _jTASKNi_18x18);

		//subs_v(Xaug_18x1,14, temp8_18x1);
		mult_mv((const float**)_jCONST_DSPi_18x14,18,14, Xaug_18x1, Qlbpp_const_18x1);

		temp7_4x1[1] = Xpel_3x1[1];
		temp7_4x1[2] = Xpel_3x1[2];
		temp7_4x1[3] = Xpel_3x1[3];
		temp7_4x1[4] = Xpcz_1x1;

		mult_mv((const float**)_jTASK_18x18,4,18, Qlbpp_const_18x1, temp8_18x1);
		diff_vv(temp7_4x1,4, temp8_18x1, temp7_4x1);
		mult_mv((const float**)_jTASKNi_18x18,18,4, temp7_4x1, Qlbpp_task_18x1);

		sum_vv(Qlbpp_const_18x1,18, Qlbpp_task_18x1, Qlbpp_18x1);

		i=0;
		while(isLimited)
		{
			isLimited = FALSE;
			if( JAVLC(_Qlb_19x1[8], _Qlbp_18x1[7], Qlbpp_18x1[7], RHYpmax, RHYppmax, RHYmin, RHYmax, D2R, &Qlbpp_18x1[7]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[9], _Qlbp_18x1[8], Qlbpp_18x1[8], RHRpmax, RHRppmax, RHRmin, RHRmax, D2R, &Qlbpp_18x1[8]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[10], _Qlbp_18x1[9], Qlbpp_18x1[9], RHPpmax, RHPppmax, RHPmin, RHPmax, D2R, &Qlbpp_18x1[9]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[11], _Qlbp_18x1[10], Qlbpp_18x1[10], RKNpmax, RKNppmax, RKNmin, RKNmax, D2R, &Qlbpp_18x1[10]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[12], _Qlbp_18x1[11], Qlbpp_18x1[11], RAPpmax, RAPppmax, RAPmin, RAPmax, D2R, &Qlbpp_18x1[11]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[13], _Qlbp_18x1[12], Qlbpp_18x1[12], RARpmax, RARppmax, RARmin, RARmax, D2R, &Qlbpp_18x1[12]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[14], _Qlbp_18x1[13], Qlbpp_18x1[13], LHYpmax, LHYppmax, LHYmin, LHYmax, D2R, &Qlbpp_18x1[13]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[15], _Qlbp_18x1[14], Qlbpp_18x1[14], LHRpmax, LHRppmax, LHRmin, LHRmax, D2R, &Qlbpp_18x1[14]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[16], _Qlbp_18x1[15], Qlbpp_18x1[15], LHPpmax, LHPppmax, LHPmin, LHPmax, D2R, &Qlbpp_18x1[15]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[17], _Qlbp_18x1[16], Qlbpp_18x1[16], LKNpmax, LKNppmax, LKNmin, LKNmax, D2R, &Qlbpp_18x1[16]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[18], _Qlbp_18x1[17], Qlbpp_18x1[17], LAPpmax, LAPppmax, LAPmin, LAPmax, D2R, &Qlbpp_18x1[17]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[19], _Qlbp_18x1[18], Qlbpp_18x1[18], LARpmax, LARppmax, LARmin, LARmax, D2R, &Qlbpp_18x1[18]) )
				isLimited = TRUE;

			if(isLimited)
			{
				mult_mv((const float**)_NCONST_DSP_18x18,18,18, Qlbpp_18x1, temp8_18x1);
				sum_vv(Qlbpp_const_18x1,18, temp8_18x1, Qlbpp_18x1);
				
				i++;
				if(i>500)
				{
					isLimited = FALSE;
					wberror("Whole-body motion convergence failure!!\n");
				}
			}			
		}
	}
			
	_Qlb_19x1[1] += _Qlbp_18x1[1]*DT + 0.5f*Qlbpp_18x1[1]*DT*DT;
	_Qlb_19x1[2] += _Qlbp_18x1[2]*DT + 0.5f*Qlbpp_18x1[2]*DT*DT;
	_Qlb_19x1[3] += _Qlbp_18x1[3]*DT + 0.5f*Qlbpp_18x1[3]*DT*DT;

	wPELnorm  = (float)sqrt(_Qlbp_18x1[4]*_Qlbp_18x1[4] + _Qlbp_18x1[5]*_Qlbp_18x1[5] + _Qlbp_18x1[6]*_Qlbp_18x1[6]);

	if(wPELnorm < 1e-6)
	{
		Wmatrix(&Qlbpp_18x1[3], _Wpmat_4x4);
		mult_smv(0.25f*DT*DT, (const float**)_Wpmat_4x4,4,4, &_Qlb_19x1[3], temp5_4x1);
		_Qlb_19x1[4] += temp5_4x1[1];
		_Qlb_19x1[5] += temp5_4x1[2];
		_Qlb_19x1[6] += temp5_4x1[3];
		_Qlb_19x1[7] += temp5_4x1[4];
	}
	else
	{
		Wmatrix(&_Qlbp_18x1[3], _Wmat_4x4);
		Wmatrix(&Qlbpp_18x1[3], _Wpmat_4x4);
		mult_mv((const float**)_Wmat_4x4,4,4, &_Qlb_19x1[3], temp5_4x1);
		mult_mv((const float**)_Wpmat_4x4,4,4, &_Qlb_19x1[3], temp6_4x1);
		mult_mv((const float**)_Wmat_4x4,4,4, temp6_4x1, temp7_4x1);

		sum_svsv((float)cos(wPELnorm*DT*0.5f), &_Qlb_19x1[3],4, 1.f/wPELnorm*(float)sin(wPELnorm*DT*0.5f),temp5_4x1, &_Qlb_19x1[3]);
		sum_svsv(1.f, &_Qlb_19x1[3],4, 2.f/(wPELnorm*wPELnorm)*(1.f-(float)cos(wPELnorm*DT*0.5f)),temp6_4x1, &_Qlb_19x1[3]);
		sum_svsv(1.f, &_Qlb_19x1[3],4, 1.f/(wPELnorm*wPELnorm)*(DT-2.f/wPELnorm*(float)sin(wPELnorm*DT*0.5f)), temp7_4x1, &_Qlb_19x1[3]);
	}

	temp = (float)sqrt(_Qlb_19x1[4]*_Qlb_19x1[4] + _Qlb_19x1[5]*_Qlb_19x1[5] + _Qlb_19x1[6]*_Qlb_19x1[6] + _Qlb_19x1[7]*_Qlb_19x1[7]);
	_Qlb_19x1[4] /= temp;
	_Qlb_19x1[5] /= temp;
	_Qlb_19x1[6] /= temp;
	_Qlb_19x1[7] /= temp;

	_Qlb_19x1[8] += _Qlbp_18x1[7]*DT + 0.5f*Qlbpp_18x1[7]*DT*DT;
	_Qlb_19x1[9] += _Qlbp_18x1[8]*DT + 0.5f*Qlbpp_18x1[8]*DT*DT;
	_Qlb_19x1[10] += _Qlbp_18x1[9]*DT + 0.5f*Qlbpp_18x1[9]*DT*DT;
	_Qlb_19x1[11] += _Qlbp_18x1[10]*DT + 0.5f*Qlbpp_18x1[10]*DT*DT;
	_Qlb_19x1[12] += _Qlbp_18x1[11]*DT + 0.5f*Qlbpp_18x1[11]*DT*DT;
	_Qlb_19x1[13] += _Qlbp_18x1[12]*DT + 0.5f*Qlbpp_18x1[12]*DT*DT;
	_Qlb_19x1[14] += _Qlbp_18x1[13]*DT + 0.5f*Qlbpp_18x1[13]*DT*DT;
	_Qlb_19x1[15] += _Qlbp_18x1[14]*DT + 0.5f*Qlbpp_18x1[14]*DT*DT;
	_Qlb_19x1[16] += _Qlbp_18x1[15]*DT + 0.5f*Qlbpp_18x1[15]*DT*DT;
	_Qlb_19x1[17] += _Qlbp_18x1[16]*DT + 0.5f*Qlbpp_18x1[16]*DT*DT;
	_Qlb_19x1[18] += _Qlbp_18x1[17]*DT + 0.5f*Qlbpp_18x1[17]*DT*DT;
	_Qlb_19x1[19] += _Qlbp_18x1[18]*DT + 0.5f*Qlbpp_18x1[18]*DT*DT;

	_Qlbp_18x1[1] += DT*Qlbpp_18x1[1];
	_Qlbp_18x1[2] += DT*Qlbpp_18x1[2];
	_Qlbp_18x1[3] += DT*Qlbpp_18x1[3];
	_Qlbp_18x1[4] += DT*Qlbpp_18x1[4];
	_Qlbp_18x1[5] += DT*Qlbpp_18x1[5];
	_Qlbp_18x1[6] += DT*Qlbpp_18x1[6];
	_Qlbp_18x1[7] += DT*Qlbpp_18x1[7];
	_Qlbp_18x1[8] += DT*Qlbpp_18x1[8];
	_Qlbp_18x1[9] += DT*Qlbpp_18x1[9];
	_Qlbp_18x1[10] += DT*Qlbpp_18x1[10];
	_Qlbp_18x1[11] += DT*Qlbpp_18x1[11];
	_Qlbp_18x1[12] += DT*Qlbpp_18x1[12];
	_Qlbp_18x1[13] += DT*Qlbpp_18x1[13];
	_Qlbp_18x1[14] += DT*Qlbpp_18x1[14];
	_Qlbp_18x1[15] += DT*Qlbpp_18x1[15];
	_Qlbp_18x1[16] += DT*Qlbpp_18x1[16];
	_Qlbp_18x1[17] += DT*Qlbpp_18x1[17];
	_Qlbp_18x1[18] += DT*Qlbpp_18x1[18];

	_Qub_11x1[1] += _Qubp_11x1[1]*DT + 0.5f*Qubpp_11x1[1]*DT*DT;
	_Qub_11x1[2] += _Qubp_11x1[2]*DT + 0.5f*Qubpp_11x1[2]*DT*DT;
	_Qub_11x1[3] += _Qubp_11x1[3]*DT + 0.5f*Qubpp_11x1[3]*DT*DT;
	_Qub_11x1[4] += _Qubp_11x1[4]*DT + 0.5f*Qubpp_11x1[4]*DT*DT;
	_Qub_11x1[5] += _Qubp_11x1[5]*DT + 0.5f*Qubpp_11x1[5]*DT*DT;
	_Qub_11x1[6] += _Qubp_11x1[6]*DT + 0.5f*Qubpp_11x1[6]*DT*DT;
	_Qub_11x1[7] += _Qubp_11x1[7]*DT + 0.5f*Qubpp_11x1[7]*DT*DT;
	_Qub_11x1[8] += _Qubp_11x1[8]*DT + 0.5f*Qubpp_11x1[8]*DT*DT;
	_Qub_11x1[9] += _Qubp_11x1[9]*DT + 0.5f*Qubpp_11x1[9]*DT*DT;
	_Qub_11x1[10] += _Qubp_11x1[10]*DT + 0.5f*Qubpp_11x1[10]*DT*DT;
	_Qub_11x1[11] += _Qubp_11x1[11]*DT + 0.5f*Qubpp_11x1[11]*DT*DT;

	_Qubp_11x1[1] += DT*Qubpp_11x1[1];
	_Qubp_11x1[2] += DT*Qubpp_11x1[2];
	_Qubp_11x1[3] += DT*Qubpp_11x1[3];
	_Qubp_11x1[4] += DT*Qubpp_11x1[4];
	_Qubp_11x1[5] += DT*Qubpp_11x1[5];
	_Qubp_11x1[6] += DT*Qubpp_11x1[6];
	_Qubp_11x1[7] += DT*Qubpp_11x1[7];
	_Qubp_11x1[8] += DT*Qubpp_11x1[8];
	_Qubp_11x1[9] += DT*Qubpp_11x1[9];
	_Qubp_11x1[10] += DT*Qubpp_11x1[10];
	_Qubp_11x1[11] += DT*Qubpp_11x1[11];
	
	Joint[RHY].RefAngleCurrent = _Qlb_19x1[8]*R2D;
	Joint[RHR].RefAngleCurrent = _Qlb_19x1[9]*R2D;
	Joint[RHP].RefAngleCurrent = _Qlb_19x1[10]*R2D;
	Joint[RKN].RefAngleCurrent = _Qlb_19x1[11]*R2D;
	Joint[RAP].RefAngleCurrent = _Qlb_19x1[12]*R2D;
	Joint[RAR].RefAngleCurrent = _Qlb_19x1[13]*R2D;
	Joint[LHY].RefAngleCurrent = _Qlb_19x1[14]*R2D;
	Joint[LHR].RefAngleCurrent = _Qlb_19x1[15]*R2D;
	Joint[LHP].RefAngleCurrent = _Qlb_19x1[16]*R2D;
	Joint[LKN].RefAngleCurrent = _Qlb_19x1[17]*R2D;
	Joint[LAP].RefAngleCurrent = _Qlb_19x1[18]*R2D;
	Joint[LAR].RefAngleCurrent = _Qlb_19x1[19]*R2D;

	Joint[WST].RefAngleCurrent = _Qub_11x1[1]*R2D;
	Joint[RSP].RefAngleCurrent = _Qub_11x1[2]*R2D;
	Joint[RSR].RefAngleCurrent = _Qub_11x1[3]*R2D - OFFSET_RSR;
	Joint[RSY].RefAngleCurrent = _Qub_11x1[4]*R2D;
	Joint[REB].RefAngleCurrent = _Qub_11x1[5]*R2D;
	Joint[RWY].RefAngleCurrent = _Qub_11x1[6]*R2D;
	Joint[LSP].RefAngleCurrent = _Qub_11x1[7]*R2D;
	Joint[LSR].RefAngleCurrent = _Qub_11x1[8]*R2D - OFFSET_LSR;
	Joint[LSY].RefAngleCurrent = _Qub_11x1[9]*R2D;
	Joint[LEB].RefAngleCurrent = _Qub_11x1[10]*R2D;
	Joint[LWY].RefAngleCurrent = _Qub_11x1[11]*R2D;

	return 0;
}


int UpdatePassiveCoord_DSP(void)
{
	unsigned int trial=0;
	
	const float pRFC_ref[4] = {0., 0., -0.5f*L_PEL, 0.};
	const float qRF_ref[5] = {0., 1.f, 0., 0., 0.};
	const float pLFC_ref[4] = {0., 0., 0.5f*L_PEL, 0.};
	const float qLF_ref[5] = {0., 1.f, 0., 0., 0.};

	float qPEL[5] = {0., 1.f, 0., 0., 0.};
	float pPC[4] = {0., 0., 0., PCZ0};
	float pRHIP[4], pRKN[4], pRANK[4], pRFC[4];
	float pLHIP[4], pLKN[4], pLANK[4], pLFC[4];
	float dX_DSP_12x1[13];
	float qRF[5], qLF[5];

	float temp1_3x1[4], temp2_12x1[13], temp3_3x1[4];
	float temp;

	RZ(_Qlb_19x1[8], _Rz_RHY_3x3);
	RX(_Qlb_19x1[9], _Rx_RHR_3x3);
	RY(_Qlb_19x1[10], _Ry_RHP_3x3);
	RY(_Qlb_19x1[11], _Ry_RKN_3x3);
	RY(_Qlb_19x1[12], _Ry_RAP_3x3);
	RX(_Qlb_19x1[13], _Rx_RAR_3x3);

	RZ(_Qlb_19x1[14], _Rz_LHY_3x3);
	RX(_Qlb_19x1[15], _Rx_LHR_3x3);
	RY(_Qlb_19x1[16], _Ry_LHP_3x3);
	RY(_Qlb_19x1[17], _Ry_LKN_3x3);
	RY(_Qlb_19x1[18], _Ry_LAP_3x3);
	RX(_Qlb_19x1[19], _Rx_LAR_3x3);

	QT2DC(qPEL, _dcPEL_3x3);		

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcRUL_3x3);

	mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
	
	mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcRF_3x3);


	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_18x18);
	mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcLUL_3x3);

	mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
	
	mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_18x18);
	mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcLF_3x3);
	
	
	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRHIP);

	mult_mv((const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRHIP,3, temp3_3x1, pRKN);

	mult_mv((const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, pRANK);

	mult_mv((const float**)_dcRF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(pRANK,3, temp3_3x1, pRFC);


	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLHIP);

	mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLHIP,3, temp3_3x1, pLKN);

	mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, pLANK);

	mult_mv((const float**)_dcLF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(pLANK,3, temp3_3x1, pLFC);
	
	dX_DSP_12x1[1] = pRFC_ref[1]-pRFC[1];
	dX_DSP_12x1[2] = pRFC_ref[2]-pRFC[2];
	dX_DSP_12x1[3] = pRFC_ref[3]-pRFC[3];

	dX_DSP_12x1[7] = pLFC_ref[1]-pLFC[1];
	dX_DSP_12x1[8] = pLFC_ref[2]-pLFC[2];
	dX_DSP_12x1[9] = pLFC_ref[3]-pLFC[3];
	
	DC2QT((const float**)_dcRF_3x3, qRF);
	DC2QT((const float**)_dcLF_3x3, qLF);
	QTdel(qRF_ref, qRF, &dX_DSP_12x1[3]);
	QTdel(qLF_ref, qLF, &dX_DSP_12x1[9]);

	while(norm_v(dX_DSP_12x1,12) > 1e-5)
	{
		QThat(qPEL, _TEMP3_18x18);
		trans2(1.f,_TEMP3_18x18,4,3);

		diff_vv(pRFC,3, pPC,temp1_3x1);
		Crsmatrix(temp1_3x1, _TEMP4_18x18);
		mult_smm(-2.f, (const float**)_TEMP4_18x18,3,3, (const float**)_TEMP3_18x18,4, _TEMP1_18x18);
		aug_mm(1.f, (const float**)_EYE_18,3,3, (const float**)_TEMP1_18x18,4, _TEMP2_18x18);

		_TEMP2_18x18[4][1] = 0.;
		_TEMP2_18x18[4][2] = 0.;
		_TEMP2_18x18[4][3] = 0.;
		_TEMP2_18x18[4][4] = 2.f*_TEMP3_18x18[1][1];
		_TEMP2_18x18[4][5] = 2.f*_TEMP3_18x18[1][2];
		_TEMP2_18x18[4][6] = 2.f*_TEMP3_18x18[1][3];
		_TEMP2_18x18[4][7] = 2.f*_TEMP3_18x18[1][4];

		_TEMP2_18x18[5][1] = 0.;
		_TEMP2_18x18[5][2] = 0.;
		_TEMP2_18x18[5][3] = 0.;
		_TEMP2_18x18[5][4] = 2.f*_TEMP3_18x18[2][1];
		_TEMP2_18x18[5][5] = 2.f*_TEMP3_18x18[2][2];
		_TEMP2_18x18[5][6] = 2.f*_TEMP3_18x18[2][3];
		_TEMP2_18x18[5][7] = 2.f*_TEMP3_18x18[2][4];

		_TEMP2_18x18[6][1] = 0.;
		_TEMP2_18x18[6][2] = 0.;
		_TEMP2_18x18[6][3] = 0.;
		_TEMP2_18x18[6][4] = 2.f*_TEMP3_18x18[3][1];
		_TEMP2_18x18[6][5] = 2.f*_TEMP3_18x18[3][2];
		_TEMP2_18x18[6][6] = 2.f*_TEMP3_18x18[3][3];
		_TEMP2_18x18[6][7] = 2.f*_TEMP3_18x18[3][4];


		diff_vv(pLFC,3, pPC,temp1_3x1);
		Crsmatrix(temp1_3x1, _TEMP4_18x18);
		mult_smm(-2.f, (const float**)_TEMP4_18x18,3,3, (const float**)_TEMP3_18x18,4, _TEMP1_18x18);
		aug_mm(1.f, (const float**)_EYE_18,3,3, (const float**)_TEMP1_18x18,4, _TEMP4_18x18);

		_TEMP4_18x18[4][1] = 0.;
		_TEMP4_18x18[4][2] = 0.;
		_TEMP4_18x18[4][3] = 0.;
		_TEMP4_18x18[4][4] = 2.f*_TEMP3_18x18[1][1];
		_TEMP4_18x18[4][5] = 2.f*_TEMP3_18x18[1][2];
		_TEMP4_18x18[4][6] = 2.f*_TEMP3_18x18[1][3];
		_TEMP4_18x18[4][7] = 2.f*_TEMP3_18x18[1][4];

		_TEMP4_18x18[5][1] = 0.;
		_TEMP4_18x18[5][2] = 0.;
		_TEMP4_18x18[5][3] = 0.;
		_TEMP4_18x18[5][4] = 2.f*_TEMP3_18x18[2][1];
		_TEMP4_18x18[5][5] = 2.f*_TEMP3_18x18[2][2];
		_TEMP4_18x18[5][6] = 2.f*_TEMP3_18x18[2][3];
		_TEMP4_18x18[5][7] = 2.f*_TEMP3_18x18[2][4];

		_TEMP4_18x18[6][1] = 0.;
		_TEMP4_18x18[6][2] = 0.;
		_TEMP4_18x18[6][3] = 0.;
		_TEMP4_18x18[6][4] = 2.f*_TEMP3_18x18[3][1];
		_TEMP4_18x18[6][5] = 2.f*_TEMP3_18x18[3][2];
		_TEMP4_18x18[6][6] = 2.f*_TEMP3_18x18[3][3];
		_TEMP4_18x18[6][7] = 2.f*_TEMP3_18x18[3][4];
		
		accu_mm(1.f, (const float**)_TEMP2_18x18,6,7, (const float**)_TEMP4_18x18,6, _TEMP1_18x18);
		
		trans(1.f, (const float**)_TEMP1_18x18,12,7, _TEMP2_18x18);
		mult_mm((const float**)_TEMP2_18x18,7,12, (const float**)_TEMP1_18x18,7, _TEMP3_18x18);
		inv(_TEMP3_18x18,7);
		mult_mm((const float**)_TEMP3_18x18,7,7, (const float**)_TEMP2_18x18,12, _TEMP1_18x18);

		mult_mv((const float**)_TEMP1_18x18,7,12, dX_DSP_12x1, temp2_12x1);

		pPC[1] += temp2_12x1[1];
		pPC[2] += temp2_12x1[2];
		pPC[3] += temp2_12x1[3];

		qPEL[1] += temp2_12x1[4];
		qPEL[2] += temp2_12x1[5];
		qPEL[3] += temp2_12x1[6];
		qPEL[4] += temp2_12x1[7];
		temp = norm_v(qPEL,4);
		qPEL[1] /= temp;
		qPEL[2] /= temp;
		qPEL[3] /= temp;
		qPEL[4] /= temp;

		QT2DC(qPEL, _dcPEL_3x3);		

		mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_18x18);
		mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_18x18);
		mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcRUL_3x3);

		mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
		
		mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_18x18);
		mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcRF_3x3);


		mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_18x18);
		mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_18x18);
		mult_mm((const float**)_TEMP1_18x18,3,3, (const float**)_TEMP2_18x18,3, _dcLUL_3x3);

		mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
		
		mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_18x18);
		mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_18x18,3, _dcLF_3x3);
		
		
		mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
		sum_vv(pPC,3, temp3_3x1, pRHIP);

		mult_mv((const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
		sum_vv(pRHIP,3, temp3_3x1, pRKN);

		mult_mv((const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
		sum_vv(pRKN,3, temp3_3x1, pRANK);

		mult_mv((const float**)_dcRF_3x3,3,3, _LINK_FOOT, temp3_3x1);
		sum_vv(pRANK,3, temp3_3x1, pRFC);


		mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
		sum_vv(pPC,3, temp3_3x1, pLHIP);

		mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
		sum_vv(pLHIP,3, temp3_3x1, pLKN);

		mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
		sum_vv(pLKN,3, temp3_3x1, pLANK);

		mult_mv((const float**)_dcLF_3x3,3,3, _LINK_FOOT, temp3_3x1);
		sum_vv(pLANK,3, temp3_3x1, pLFC);
		
		dX_DSP_12x1[1] = pRFC_ref[1]-pRFC[1];
		dX_DSP_12x1[2] = pRFC_ref[2]-pRFC[2];
		dX_DSP_12x1[3] = pRFC_ref[3]-pRFC[3];

		dX_DSP_12x1[7] = pLFC_ref[1]-pLFC[1];
		dX_DSP_12x1[8] = pLFC_ref[2]-pLFC[2];
		dX_DSP_12x1[9] = pLFC_ref[3]-pLFC[3];
		
		DC2QT((const float**)_dcRF_3x3, qRF);
		DC2QT((const float**)_dcLF_3x3, qLF);
		QTdel(qRF_ref, qRF, &dX_DSP_12x1[3]);
		QTdel(qLF_ref, qLF, &dX_DSP_12x1[9]);		

		trial++;

		if(trial > 300)
			return 1;

	}

	_Qlb_19x1[1] = pPC[1];
	_Qlb_19x1[2] = pPC[2];
	_Qlb_19x1[3] = pPC[3];
	_Qlb_19x1[4] = qPEL[1];
	_Qlb_19x1[5] = qPEL[2];
	_Qlb_19x1[6] = qPEL[3];
	_Qlb_19x1[7] = qPEL[4];

	_pPCz0 = _Qlb_19x1[3];

	Pos_COM((const float*)_Qlb_19x1, (const float*)_Qub_11x1, _pCOM0_3x1);

	
	return 0;
}


void wberror(char error_text[])
/* whole-body motion error handler */
{
	printf("\nWhole-body motion run-time error...\n");
	printf("%s\n",error_text);
}



int derive3(float x0_in, float x1_in, float x2_in, float *xd1_out, float *xdd1_out, float dt_in)
{
	*xd1_out = (x2_in-x0_in)/(2.f*dt_in);
	*xdd1_out = (x2_in-2.f*x1_in+x0_in)/(dt_in*dt_in);
	
	return 0;
}


int JW_motion_window(float wst, float rsp, float rsr, float rsy, float reb, float rwy, float rwp,
				     float lsp, float lsr, float lsy, float leb, float lwy, float lwp,
					 float dNKY, float dNK1, float dNK2,
					 float dRF1, float dRF2, float dRF3, float dRF4, float dRF5,
					 float dLF1, float dLF2, float dLF3, float dLF4, float dLF5,
					 float body_pitch)
{
	unsigned int head_new_WST = (_Qub_window.head_WST+1)%WIND_SIZE_WST;
	unsigned int head_new_RSP = (_Qub_window.head_RSP+1)%WIND_SIZE_RSP;
	unsigned int head_new_RSR = (_Qub_window.head_RSR+1)%WIND_SIZE_RSR;
	unsigned int head_new_RSY = (_Qub_window.head_RSY+1)%WIND_SIZE_RSY;
	unsigned int head_new_REB = (_Qub_window.head_REB+1)%WIND_SIZE_REB;
	unsigned int head_new_RWY = (_Qub_window.head_RWY+1)%WIND_SIZE_RWY;
	unsigned int head_new_RWP = (_Qub_window.head_RWP+1)%WIND_SIZE_RWP;
	unsigned int head_new_LSP = (_Qub_window.head_LSP+1)%WIND_SIZE_LSP;
	unsigned int head_new_LSR = (_Qub_window.head_LSR+1)%WIND_SIZE_LSR;
	unsigned int head_new_LSY = (_Qub_window.head_LSY+1)%WIND_SIZE_LSY;
	unsigned int head_new_LEB = (_Qub_window.head_LEB+1)%WIND_SIZE_LEB;
	unsigned int head_new_LWY = (_Qub_window.head_LWY+1)%WIND_SIZE_LWY;
	unsigned int head_new_LWP = (_Qub_window.head_LWP+1)%WIND_SIZE_LWP;
	unsigned int head_new_BP = (_Qub_window.head_BP+1)%WIND_SIZE_BP;

	
	_Qub_window.wind_WST[head_new_WST] = wst;
	_Qub_window.wind_RSP[head_new_RSP] = rsp;
	_Qub_window.wind_RSR[head_new_RSR] = (rsr + OFFSET_RSR*D2R);
	_Qub_window.wind_RSY[head_new_RSY] = rsy;
	_Qub_window.wind_REB[head_new_REB] = reb;
	_Qub_window.wind_RWY[head_new_RWY] = rwy;
	_Qub_window.wind_RWP[head_new_RWP] = rwp;
	_Qub_window.wind_LSP[head_new_LSP] = lsp;
	_Qub_window.wind_LSR[head_new_LSR] = (lsr + OFFSET_LSR*D2R);
	_Qub_window.wind_LSY[head_new_LSY] = lsy;
	_Qub_window.wind_LEB[head_new_LEB] = leb;
	_Qub_window.wind_LWY[head_new_LWY] = lwy;
	_Qub_window.wind_LWP[head_new_LWP] = lwp;

	_Qub_window.wind_BP[head_new_BP] = -body_pitch;
	
	_Qub_window.dNK_JW_3x1[1] = dNKY;
	_Qub_window.dNK_JW_3x1[2] = dNK1;
	_Qub_window.dNK_JW_3x1[3] = dNK2;

	_Qub_window.dRF_JW_5x1[1] = dRF1;
	_Qub_window.dRF_JW_5x1[2] = dRF2;
	_Qub_window.dRF_JW_5x1[3] = dRF3;
	_Qub_window.dRF_JW_5x1[4] = dRF4;
	_Qub_window.dRF_JW_5x1[5] = dRF5;

	_Qub_window.dLF_JW_5x1[1] = dLF1;
	_Qub_window.dLF_JW_5x1[2] = dLF2;
	_Qub_window.dLF_JW_5x1[3] = dLF3;
	_Qub_window.dLF_JW_5x1[4] = dLF4;
	_Qub_window.dLF_JW_5x1[5] = dLF5;

	_Qub_window.head_WST = head_new_WST;
	_Qub_window.head_RSP = head_new_RSP;
	_Qub_window.head_RSR = head_new_RSR;
	_Qub_window.head_RSY = head_new_RSY;
	_Qub_window.head_REB = head_new_REB;
	_Qub_window.head_RWY = head_new_RWY;
	_Qub_window.head_RWP = head_new_RWP;
	_Qub_window.head_LSP = head_new_LSP;
	_Qub_window.head_LSR = head_new_LSR;
	_Qub_window.head_LSY = head_new_LSY;
	_Qub_window.head_LEB = head_new_LEB;
	_Qub_window.head_LWY = head_new_LWY;
	_Qub_window.head_LWP = head_new_LWP;
	_Qub_window.head_BP = head_new_BP;

	if(head_new_WST == _Qub_window.index_maxWST)
		findmax(_Qub_window.wind_WST, WIND_SIZE_WST, &_Qub_window.maxWST, &_Qub_window.index_maxWST);
	else if(_Qub_window.maxWST < _Qub_window.wind_WST[head_new_WST])
	{
		_Qub_window.maxWST = _Qub_window.wind_WST[head_new_WST];
		_Qub_window.index_maxWST = head_new_WST;
	}	
	
	if(head_new_RSP == _Qub_window.index_maxRSP)
		findmax(_Qub_window.wind_RSP, WIND_SIZE_RSP, &_Qub_window.maxRSP, &_Qub_window.index_maxRSP);
	else if(_Qub_window.maxRSP < _Qub_window.wind_RSP[head_new_RSP])
	{
		_Qub_window.maxRSP = _Qub_window.wind_RSP[head_new_RSP];
		_Qub_window.index_maxRSP = head_new_RSP;
	}

	if(head_new_RSR == _Qub_window.index_maxRSR)
		findmax(_Qub_window.wind_RSR, WIND_SIZE_RSR, &_Qub_window.maxRSR, &_Qub_window.index_maxRSR);
	else if(_Qub_window.maxRSR < _Qub_window.wind_RSR[head_new_RSR])
	{
		_Qub_window.maxRSR = _Qub_window.wind_RSR[head_new_RSR];
		_Qub_window.index_maxRSR = head_new_RSR;
	}

	if(head_new_RSY == _Qub_window.index_maxRSY)
		findmax(_Qub_window.wind_RSY, WIND_SIZE_RSY, &_Qub_window.maxRSY, &_Qub_window.index_maxRSY);
	else if(_Qub_window.maxRSY < _Qub_window.wind_RSY[head_new_RSY])
	{
		_Qub_window.maxRSY = _Qub_window.wind_RSY[head_new_RSY];
		_Qub_window.index_maxRSY = head_new_RSY;
	}

	if(head_new_REB == _Qub_window.index_maxREB)
		findmax(_Qub_window.wind_REB, WIND_SIZE_REB, &_Qub_window.maxREB, &_Qub_window.index_maxREB);
	else if(_Qub_window.maxREB < _Qub_window.wind_REB[head_new_REB])
	{
		_Qub_window.maxREB = _Qub_window.wind_REB[head_new_REB];
		_Qub_window.index_maxREB = head_new_REB;
	}

	if(head_new_RWY == _Qub_window.index_maxRWY)
		findmax(_Qub_window.wind_RWY, WIND_SIZE_RWY, &_Qub_window.maxRWY, &_Qub_window.index_maxRWY);
	else if(_Qub_window.maxRWY < _Qub_window.wind_RWY[head_new_RWY])
	{
		_Qub_window.maxRWY = _Qub_window.wind_RWY[head_new_RWY];
		_Qub_window.index_maxRWY = head_new_RWY;
	}

	if(head_new_RWP == _Qub_window.index_maxRWP)
		findmax(_Qub_window.wind_RWP, WIND_SIZE_RWP, &_Qub_window.maxRWP, &_Qub_window.index_maxRWP);
	else if(_Qub_window.maxRWP < _Qub_window.wind_RWP[head_new_RWP])
	{
		_Qub_window.maxRWP = _Qub_window.wind_RWP[head_new_RWP];
		_Qub_window.index_maxRWP = head_new_RWP;
	}

	if(head_new_LSP == _Qub_window.index_maxLSP)
		findmax(_Qub_window.wind_LSP, WIND_SIZE_LSP, &_Qub_window.maxLSP, &_Qub_window.index_maxLSP);
	else if(_Qub_window.maxLSP < _Qub_window.wind_LSP[head_new_LSP])
	{
		_Qub_window.maxLSP = _Qub_window.wind_LSP[head_new_LSP];
		_Qub_window.index_maxLSP = head_new_LSP;
	}

	if(head_new_LSR == _Qub_window.index_maxLSR)
		findmax(_Qub_window.wind_LSR, WIND_SIZE_LSR, &_Qub_window.maxLSR, &_Qub_window.index_maxLSR);
	else if(_Qub_window.maxLSR < _Qub_window.wind_LSR[head_new_LSR])
	{
		_Qub_window.maxLSR = _Qub_window.wind_LSR[head_new_LSR];
		_Qub_window.index_maxLSR = head_new_LSR;
	}

	if(head_new_LSY == _Qub_window.index_maxLSY)
		findmax(_Qub_window.wind_LSY, WIND_SIZE_LSY, &_Qub_window.maxLSY, &_Qub_window.index_maxLSY);
	else if(_Qub_window.maxLSY < _Qub_window.wind_LSY[head_new_LSY])
	{
		_Qub_window.maxLSY = _Qub_window.wind_LSY[head_new_LSY];
		_Qub_window.index_maxLSY = head_new_LSY;
	}

	if(head_new_LEB == _Qub_window.index_maxLEB)
		findmax(_Qub_window.wind_LEB, WIND_SIZE_LEB, &_Qub_window.maxLEB, &_Qub_window.index_maxLEB);
	else if(_Qub_window.maxLEB < _Qub_window.wind_LEB[head_new_LEB])
	{
		_Qub_window.maxLEB = _Qub_window.wind_LEB[head_new_LEB];
		_Qub_window.index_maxLEB = head_new_LEB;
	}

	if(head_new_LWY == _Qub_window.index_maxLWY)
		findmax(_Qub_window.wind_LWY, WIND_SIZE_LWY, &_Qub_window.maxLWY, &_Qub_window.index_maxLWY);
	else if(_Qub_window.maxLWY < _Qub_window.wind_LWY[head_new_LWY])
	{
		_Qub_window.maxLWY = _Qub_window.wind_LWY[head_new_LWY];
		_Qub_window.index_maxLWY = head_new_LWY;
	}

	if(head_new_LWP == _Qub_window.index_maxLWP)
		findmax(_Qub_window.wind_LWP, WIND_SIZE_LWP, &_Qub_window.maxLWP, &_Qub_window.index_maxLWP);
	else if(_Qub_window.maxLWP < _Qub_window.wind_LWP[head_new_LWP])
	{
		_Qub_window.maxLWP = _Qub_window.wind_LWP[head_new_LWP];
		_Qub_window.index_maxLWP = head_new_LWP;
	}



	if(head_new_WST == _Qub_window.index_minWST)
		findmin(_Qub_window.wind_WST, WIND_SIZE_WST, &_Qub_window.minWST, &_Qub_window.index_minWST);
	else if(_Qub_window.minWST > _Qub_window.wind_WST[head_new_WST])
	{
		_Qub_window.minWST = _Qub_window.wind_WST[head_new_WST];
		_Qub_window.index_minWST = head_new_WST;
	}	
	
	if(head_new_RSP == _Qub_window.index_minRSP)
		findmin(_Qub_window.wind_RSP, WIND_SIZE_RSP, &_Qub_window.minRSP, &_Qub_window.index_minRSP);
	else if(_Qub_window.minRSP > _Qub_window.wind_RSP[head_new_RSP])
	{
		_Qub_window.minRSP = _Qub_window.wind_RSP[head_new_RSP];
		_Qub_window.index_minRSP = head_new_RSP;
	}

	if(head_new_RSR == _Qub_window.index_minRSR)
		findmin(_Qub_window.wind_RSR, WIND_SIZE_RSR, &_Qub_window.minRSR, &_Qub_window.index_minRSR);
	else if(_Qub_window.minRSR > _Qub_window.wind_RSR[head_new_RSR])
	{
		_Qub_window.minRSR = _Qub_window.wind_RSR[head_new_RSR];
		_Qub_window.index_minRSR = head_new_RSR;
	}

	if(head_new_RSY == _Qub_window.index_minRSY)
		findmin(_Qub_window.wind_RSY, WIND_SIZE_RSY, &_Qub_window.minRSY, &_Qub_window.index_minRSY);
	else if(_Qub_window.minRSY > _Qub_window.wind_RSY[head_new_RSY])
	{
		_Qub_window.minRSY = _Qub_window.wind_RSY[head_new_RSY];
		_Qub_window.index_minRSY = head_new_RSY;
	}

	if(head_new_REB == _Qub_window.index_minREB)
		findmin(_Qub_window.wind_REB, WIND_SIZE_REB, &_Qub_window.minREB, &_Qub_window.index_minREB);
	else if(_Qub_window.minREB > _Qub_window.wind_REB[head_new_REB])
	{
		_Qub_window.minREB = _Qub_window.wind_REB[head_new_REB];
		_Qub_window.index_minREB = head_new_REB;
	}

	if(head_new_RWY == _Qub_window.index_minRWY)
		findmin(_Qub_window.wind_RWY, WIND_SIZE_RWY, &_Qub_window.minRWY, &_Qub_window.index_minRWY);
	else if(_Qub_window.minRWY > _Qub_window.wind_RWY[head_new_RWY])
	{
		_Qub_window.minRWY = _Qub_window.wind_RWY[head_new_RWY];
		_Qub_window.index_minRWY = head_new_RWY;
	}

	if(head_new_RWP == _Qub_window.index_minRWP)
		findmin(_Qub_window.wind_RWP, WIND_SIZE_RWP, &_Qub_window.minRWP, &_Qub_window.index_minRWP);
	else if(_Qub_window.minRWP > _Qub_window.wind_RWP[head_new_RWP])
	{
		_Qub_window.minRWP = _Qub_window.wind_RWP[head_new_RWP];
		_Qub_window.index_minRWP = head_new_RWP;
	}

	if(head_new_LSP == _Qub_window.index_minLSP)
		findmin(_Qub_window.wind_LSP, WIND_SIZE_LSP, &_Qub_window.minLSP, &_Qub_window.index_minLSP);
	else if(_Qub_window.minLSP > _Qub_window.wind_LSP[head_new_LSP])
	{
		_Qub_window.minLSP = _Qub_window.wind_LSP[head_new_LSP];
		_Qub_window.index_minLSP = head_new_LSP;
	}

	if(head_new_LSR == _Qub_window.index_minLSR)
		findmin(_Qub_window.wind_LSR, WIND_SIZE_LSR, &_Qub_window.minLSR, &_Qub_window.index_minLSR);
	else if(_Qub_window.minLSR > _Qub_window.wind_LSR[head_new_LSR])
	{
		_Qub_window.minLSR = _Qub_window.wind_LSR[head_new_LSR];
		_Qub_window.index_minLSR = head_new_LSR;
	}

	if(head_new_LSY == _Qub_window.index_minLSY)
		findmin(_Qub_window.wind_LSY, WIND_SIZE_LSY, &_Qub_window.minLSY, &_Qub_window.index_minLSY);
	else if(_Qub_window.minLSY > _Qub_window.wind_LSY[head_new_LSY])
	{
		_Qub_window.minLSY = _Qub_window.wind_LSY[head_new_LSY];
		_Qub_window.index_minLSY = head_new_LSY;
	}

	if(head_new_LEB == _Qub_window.index_minLEB)
		findmin(_Qub_window.wind_LEB, WIND_SIZE_LEB, &_Qub_window.minLEB, &_Qub_window.index_minLEB);
	else if(_Qub_window.minLEB > _Qub_window.wind_LEB[head_new_LEB])
	{
		_Qub_window.minLEB = _Qub_window.wind_LEB[head_new_LEB];
		_Qub_window.index_minLEB = head_new_LEB;
	}

	if(head_new_LWY == _Qub_window.index_minLWY)
		findmin(_Qub_window.wind_LWY, WIND_SIZE_LWY, &_Qub_window.minLWY, &_Qub_window.index_minLWY);
	else if(_Qub_window.minLWY > _Qub_window.wind_LWY[head_new_LWY])
	{
		_Qub_window.minLWY = _Qub_window.wind_LWY[head_new_LWY];
		_Qub_window.index_minLWY = head_new_LWY;
	}

	if(head_new_LWP == _Qub_window.index_minLWP)
		findmin(_Qub_window.wind_LWP, WIND_SIZE_LWP, &_Qub_window.minLWP, &_Qub_window.index_minLWP);
	else if(_Qub_window.minLWP > _Qub_window.wind_LWP[head_new_LWP])
	{
		_Qub_window.minLWP = _Qub_window.wind_LWP[head_new_LWP];
		_Qub_window.index_minLWP = head_new_LWP;
	}


	return 0;
}



int MocapOnestep_JW(void)
{
	unsigned int i, j;

	float Qlbpp_18x1[19], Qubpp_11x1[12], Qlbpp_const_18x1[19], Qlbpp_task_18x1[19];
	float RWPpp, LWPpp;
	float pPC_3x1[4], qPEL_4x1[5];
	float pRFC_3x1[4], qRF_4x1[5];
	float pLFC_3x1[4], qLF_4x1[5];
	float pCOM_3x1[4];

	float Xaug_18x1[19];
	float Xsupp_12x1[13];
	float Xcom_2x1[3];
	float Xpel_3x1[4];
	float Xpcz_1x1;
	
	float vCOM_3x1[4], ref_aCOM_3x1[4];
	float temp1_6x1[7], temp2_6x1[7], temp3_3x1[4], temp4_3x1[4], temp5_4x1[5], temp6_4x1[5], temp7_4x1[5], temp8_18x1[19];
	float wPELnorm, temp;

	float des_WST, des_WSTp, des_WSTpp;
	float des_RSP, des_RSPp, des_RSPpp;
	float des_RSR, des_RSRp, des_RSRpp;
	float des_RSY, des_RSYp, des_RSYpp;
	float des_REB, des_REBp, des_REBpp;
	float des_RWY, des_RWYp, des_RWYpp;
	float des_RWP, des_RWPp, des_RWPpp;

	float des_LSP, des_LSPp, des_LSPpp;
	float des_LSR, des_LSRp, des_LSRpp;
	float des_LSY, des_LSYp, des_LSYpp;
	float des_LEB, des_LEBp, des_LEBpp;
	float des_LWY, des_LWYp, des_LWYpp;
	float des_LWP, des_LWPp, des_LWPpp;	

	float des_BP, des_BPp, des_BPpp;		// for body pitch

	BOOL isLimited = FALSE;

	const float des_pRFC_3x1[4] = {0., 0., -L_PEL*0.5f, 0.};
	const float des_pLFC_3x1[4] = {0., 0., L_PEL*0.5f, 0.};
	const float des_qF[5] = {0., 1., 0., 0., 0.};
	const float des_pCOM_3x1[4] = {0., _pCOM0_3x1[1], _pCOM0_3x1[2], _pCOM0_3x1[3]};
	const float jPCZlb_1x18[19] = {0., 0., 0., 1.f, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};

	unsigned int tail_up_WST, tail_down_WST;
	unsigned int tail_up_RSP, tail_down_RSP;
	unsigned int tail_up_RSR, tail_down_RSR;
	unsigned int tail_up_RSY, tail_down_RSY;
	unsigned int tail_up_REB, tail_down_REB;
	unsigned int tail_up_RWY, tail_down_RWY;
	unsigned int tail_up_RWP, tail_down_RWP;
	unsigned int tail_up_LSP, tail_down_LSP;
	unsigned int tail_up_LSR, tail_down_LSR;
	unsigned int tail_up_LSY, tail_down_LSY;
	unsigned int tail_up_LEB, tail_down_LEB;
	unsigned int tail_up_LWY, tail_down_LWY;
	unsigned int tail_up_LWP, tail_down_LWP;
	unsigned int tail_up_BP, tail_down_BP;

	float wstmax = WSTmax;
	float rspmax = RSPmax;
	float rsrmax = RSRmax;
	float rsymax = RSYmax;
	float rebmax = REPmax;
	float rwymax = REYmax;
	float rwpmax = RWPmax;
	float lspmax = LSPmax;
	float lsrmax = LSRmax;
	float lsymax = LSYmax;
	float lebmax = LEPmax;
	float lwymax = LEYmax;
	float lwpmax = LWPmax;

	float wstmin = WSTmin;
	float rspmin = RSPmin;
	float rsrmin = RSRmin;
	float rsymin = RSYmin;
	float rebmin = REPmin;
	float rwymin = REYmin;
	float rwpmin = RWPmin;
	float lspmin = LSPmin;
	float lsrmin = LSRmin;
	float lsymin = LSYmin;
	float lebmin = LEPmin;
	float lwymin = LEYmin;
	float lwpmin = LWPmin;

	
	if(_Qub_window.tail_WST == 0)
	{
		tail_up_WST = 1;
		tail_down_WST = WIND_SIZE_WST-1;
	}
	else 
	{
		tail_up_WST = (_Qub_window.tail_WST+1)%WIND_SIZE_WST;
		tail_down_WST = _Qub_window.tail_WST-1;
	}

	if(_Qub_window.tail_RSP == 0)
	{
		tail_up_RSP = 1;
		tail_down_RSP = WIND_SIZE_RSP-1;
	}
	else 
	{
		tail_up_RSP = (_Qub_window.tail_RSP+1)%WIND_SIZE_RSP;
		tail_down_RSP = _Qub_window.tail_RSP-1;
	}

	if(_Qub_window.tail_RSR == 0)
	{
		tail_up_RSR = 1;
		tail_down_RSR = WIND_SIZE_RSR-1;
	}
	else 
	{
		tail_up_RSR = (_Qub_window.tail_RSR+1)%WIND_SIZE_RSR;
		tail_down_RSR = _Qub_window.tail_RSR-1;
	}

	if(_Qub_window.tail_RSY == 0)
	{
		tail_up_RSY = 1;
		tail_down_RSY = WIND_SIZE_RSY-1;
	}
	else 
	{
		tail_up_RSY = (_Qub_window.tail_RSY+1)%WIND_SIZE_RSY;
		tail_down_RSY = _Qub_window.tail_RSY-1;
	}

	if(_Qub_window.tail_REB == 0)
	{
		tail_up_REB = 1;
		tail_down_REB = WIND_SIZE_REB-1;
	}
	else 
	{
		tail_up_REB = (_Qub_window.tail_REB+1)%WIND_SIZE_REB;
		tail_down_REB = _Qub_window.tail_REB-1;
	}

	if(_Qub_window.tail_RWY == 0)
	{
		tail_up_RWY = 1;
		tail_down_RWY = WIND_SIZE_RWY-1;
	}
	else 
	{
		tail_up_RWY = (_Qub_window.tail_RWY+1)%WIND_SIZE_RWY;
		tail_down_RWY = _Qub_window.tail_RWY-1;
	}

	if(_Qub_window.tail_RWP == 0)
	{
		tail_up_RWP = 1;
		tail_down_RWP = WIND_SIZE_RWP-1;
	}
	else 
	{
		tail_up_RWP = (_Qub_window.tail_RWP+1)%WIND_SIZE_RWP;
		tail_down_RWP = _Qub_window.tail_RWP-1;
	}

	if(_Qub_window.tail_LSP == 0)
	{
		tail_up_LSP = 1;
		tail_down_LSP = WIND_SIZE_LSP-1;
	}
	else 
	{
		tail_up_LSP = (_Qub_window.tail_LSP+1)%WIND_SIZE_LSP;
		tail_down_LSP = _Qub_window.tail_LSP-1;
	}

	if(_Qub_window.tail_LSR == 0)
	{
		tail_up_LSR = 1;
		tail_down_LSR = WIND_SIZE_LSR-1;
	}
	else 
	{
		tail_up_LSR = (_Qub_window.tail_LSR+1)%WIND_SIZE_LSR;
		tail_down_LSR = _Qub_window.tail_LSR-1;
	}

	if(_Qub_window.tail_LSY == 0)
	{
		tail_up_LSY = 1;
		tail_down_LSY = WIND_SIZE_LSY-1;
	}
	else 
	{
		tail_up_LSY = (_Qub_window.tail_LSY+1)%WIND_SIZE_LSY;
		tail_down_LSY = _Qub_window.tail_LSY-1;
	}


	if(_Qub_window.tail_LEB == 0)
	{
		tail_up_LEB = 1;
		tail_down_LEB = WIND_SIZE_LEB-1;
	}
	else 
	{
		tail_up_LEB = (_Qub_window.tail_LEB+1)%WIND_SIZE_LEB;
		tail_down_LEB = _Qub_window.tail_LEB-1;
	}

	if(_Qub_window.tail_LWY == 0)
	{
		tail_up_LWY = 1;
		tail_down_LWY = WIND_SIZE_LWY-1;
	}
	else 
	{
		tail_up_LWY = (_Qub_window.tail_LWY+1)%WIND_SIZE_LWY;
		tail_down_LWY = _Qub_window.tail_LWY-1;
	}

	if(_Qub_window.tail_LWP == 0)
	{
		tail_up_LWP = 1;
		tail_down_LWP = WIND_SIZE_LWP-1;
	}
	else 
	{
		tail_up_LWP = (_Qub_window.tail_LWP+1)%WIND_SIZE_LWP;
		tail_down_LWP = _Qub_window.tail_LWP-1;
	}

	if(_Qub_window.tail_BP == 0)
	{
		tail_up_BP = 1;
		tail_down_BP = WIND_SIZE_BP-1;
	}
	else 
	{
		tail_up_BP = (_Qub_window.tail_BP+1)%WIND_SIZE_BP;
		tail_down_BP = _Qub_window.tail_BP-1;
	}

	des_WST = _Qub_window.wind_WST[_Qub_window.tail_WST];
	des_RSP = _Qub_window.wind_RSP[_Qub_window.tail_RSP];
	des_RSR = _Qub_window.wind_RSR[_Qub_window.tail_RSR];
	des_RSY = _Qub_window.wind_RSY[_Qub_window.tail_RSY];
	des_REB = _Qub_window.wind_REB[_Qub_window.tail_REB];
	des_RWY = _Qub_window.wind_RWY[_Qub_window.tail_RWY];
	des_RWP = _Qub_window.wind_RWP[_Qub_window.tail_RWP];
	des_LSP = _Qub_window.wind_LSP[_Qub_window.tail_LSP];
	des_LSR = _Qub_window.wind_LSR[_Qub_window.tail_LSR];
	des_LSY = _Qub_window.wind_LSY[_Qub_window.tail_LSY];
	des_LEB = _Qub_window.wind_LEB[_Qub_window.tail_LEB];
	des_LWY = _Qub_window.wind_LWY[_Qub_window.tail_LWY];
	des_LWP = _Qub_window.wind_LWP[_Qub_window.tail_LWP];
	des_BP	= _Qub_window.wind_BP[_Qub_window.tail_BP];

	derive3(_Qub_window.wind_WST[tail_down_WST], des_WST, _Qub_window.wind_WST[tail_up_WST], &des_WSTp, &des_WSTpp, DT);
	derive3(_Qub_window.wind_RSP[tail_down_RSP], des_RSP, _Qub_window.wind_RSP[tail_up_RSP], &des_RSPp, &des_RSPpp, DT);
	derive3(_Qub_window.wind_RSR[tail_down_RSR], des_RSR, _Qub_window.wind_RSR[tail_up_RSR], &des_RSRp, &des_RSRpp, DT);
	derive3(_Qub_window.wind_RSY[tail_down_RSY], des_RSY, _Qub_window.wind_RSY[tail_up_RSY], &des_RSYp, &des_RSYpp, DT);
	derive3(_Qub_window.wind_REB[tail_down_REB], des_REB, _Qub_window.wind_REB[tail_up_REB], &des_REBp, &des_REBpp, DT);
	derive3(_Qub_window.wind_RWY[tail_down_RWY], des_RWY, _Qub_window.wind_RWY[tail_up_RWY], &des_RWYp, &des_RWYpp, DT);
	derive3(_Qub_window.wind_RWP[tail_down_RWP], des_RWP, _Qub_window.wind_RWP[tail_up_RWP], &des_RWPp, &des_RWPpp, DT);
	derive3(_Qub_window.wind_LSP[tail_down_LSP], des_LSP, _Qub_window.wind_LSP[tail_up_LSP], &des_LSPp, &des_LSPpp, DT);
	derive3(_Qub_window.wind_LSR[tail_down_LSR], des_LSR, _Qub_window.wind_LSR[tail_up_LSR], &des_LSRp, &des_LSRpp, DT);
	derive3(_Qub_window.wind_LSY[tail_down_LSY], des_LSY, _Qub_window.wind_LSY[tail_up_LSY], &des_LSYp, &des_LSYpp, DT);
	derive3(_Qub_window.wind_LEB[tail_down_LEB], des_LEB, _Qub_window.wind_LEB[tail_up_LEB], &des_LEBp, &des_LEBpp, DT);
	derive3(_Qub_window.wind_LWY[tail_down_LWY], des_LWY, _Qub_window.wind_LWY[tail_up_LWY], &des_LWYp, &des_LWYpp, DT);
	derive3(_Qub_window.wind_LWP[tail_down_LWP], des_LWP, _Qub_window.wind_LWP[tail_up_LWP], &des_LWPp, &des_LWPpp, DT);
	derive3(_Qub_window.wind_BP[tail_down_BP], des_BP, _Qub_window.wind_BP[tail_up_BP], &des_BPp, &des_BPpp, DT);

	_Qub_window.tail_WST = (_Qub_window.tail_WST+1)%WIND_SIZE_WST;
	_Qub_window.tail_RSP = (_Qub_window.tail_RSP+1)%WIND_SIZE_RSP;
	_Qub_window.tail_RSR = (_Qub_window.tail_RSR+1)%WIND_SIZE_RSR;
	_Qub_window.tail_RSY = (_Qub_window.tail_RSY+1)%WIND_SIZE_RSY;
	_Qub_window.tail_REB = (_Qub_window.tail_REB+1)%WIND_SIZE_REB;
	_Qub_window.tail_RWY = (_Qub_window.tail_RWY+1)%WIND_SIZE_RWY;
	_Qub_window.tail_RWP = (_Qub_window.tail_RWP+1)%WIND_SIZE_RWP;
	_Qub_window.tail_LSP = (_Qub_window.tail_LSP+1)%WIND_SIZE_LSP;
	_Qub_window.tail_LSR = (_Qub_window.tail_LSR+1)%WIND_SIZE_LSR;
	_Qub_window.tail_LSY = (_Qub_window.tail_LSY+1)%WIND_SIZE_LSY;
	_Qub_window.tail_LEB = (_Qub_window.tail_LEB+1)%WIND_SIZE_LEB;
	_Qub_window.tail_LWY = (_Qub_window.tail_LWY+1)%WIND_SIZE_LWY;
	_Qub_window.tail_LWP = (_Qub_window.tail_LWP+1)%WIND_SIZE_LWP;
	_Qub_window.tail_BP = (_Qub_window.tail_BP+1)%WIND_SIZE_BP;
	
	
	MocapKine(_Qlb_19x1, _Qub_11x1, pPC_3x1, qPEL_4x1, _jRFlb_6x18, pRFC_3x1, qRF_4x1, _jLFlb_6x18, pLFC_3x1, qLF_4x1, _jCOMlb_3x18, _jCOMub_3x11, pCOM_3x1);
	
	diff_mm((const float**)_jRFlb_6x18,6,18, (const float**)_jRFlb_old_6x18, _jRFlbp_6x18);
	mult_sm((const float**)_jRFlbp_6x18,6,18, (1.f/DT), _jRFlbp_6x18);

	diff_mm((const float**)_jLFlb_6x18,6,18, (const float**)_jLFlb_old_6x18, _jLFlbp_6x18);
	mult_sm((const float**)_jLFlbp_6x18,6,18, (1.f/DT), _jLFlbp_6x18);

	diff_mm((const float**)_jCOMlb_3x18,3,18, (const float**)_jCOMlb_old_3x18, _jCOMlbp_3x18);
	mult_sm((const float**)_jCOMlbp_3x18,3,18, (1.f/DT), _jCOMlbp_3x18);

	diff_mm((const float**)_jCOMub_3x11,3,11, (const float**)_jCOMub_old_3x11, _jCOMubp_3x11);
	mult_sm((const float**)_jCOMubp_3x11,3,11, (1.f/DT), _jCOMubp_3x11);

	subs_m((const float**)_jRFlb_6x18,6,18, _jRFlb_old_6x18);
	subs_m((const float**)_jLFlb_6x18,6,18, _jLFlb_old_6x18);
	subs_m((const float**)_jCOMlb_3x18,3,18, _jCOMlb_old_3x18);
	subs_m((const float**)_jCOMub_3x11,3,11, _jCOMub_old_3x11);
	
	for(i=1; i<=18; i++)
	{
		_jAUG_18x18[1][i] = _jRFlb_6x18[1][i];
		_jAUG_18x18[2][i] = _jRFlb_6x18[2][i];
		_jAUG_18x18[3][i] = _jRFlb_6x18[3][i];
		_jAUG_18x18[4][i] = _jRFlb_6x18[4][i];
		_jAUG_18x18[5][i] = _jRFlb_6x18[5][i];
		_jAUG_18x18[6][i] = _jRFlb_6x18[6][i];			

		_jAUG_18x18[7][i] = _jLFlb_6x18[1][i];
		_jAUG_18x18[8][i] = _jLFlb_6x18[2][i];
		_jAUG_18x18[9][i] = _jLFlb_6x18[3][i];
		_jAUG_18x18[10][i] = _jLFlb_6x18[4][i];
		_jAUG_18x18[11][i] = _jLFlb_6x18[5][i];
		_jAUG_18x18[12][i] = _jLFlb_6x18[6][i];

		_jAUG_18x18[13][i] = _jCOMlb_3x18[1][i];
		_jAUG_18x18[14][i] = _jCOMlb_3x18[2][i];

		_jAUG_18x18[15][i] = _jPELlb_att_3x18[1][i];
		_jAUG_18x18[16][i] = _jPELlb_att_3x18[2][i];
		_jAUG_18x18[17][i] = _jPELlb_att_3x18[3][i];
		
		_jAUG_18x18[18][i] = jPCZlb_1x18[i];
	}


	if(_Qubp_11x1[1] < 0.)
	{
		if(wstmin < _Qub_window.minWST)
			wstmin = _Qub_window.minWST - MARGIN;
	}
	else if(_Qubp_11x1[1] > 0.)
	{
		if(wstmax > _Qub_window.maxWST)
			wstmax = _Qub_window.maxWST + MARGIN;
	}

	if(_Qubp_11x1[2] < 0.)
	{
		if(rspmin < _Qub_window.minRSP)
			rspmin = _Qub_window.minRSP - MARGIN;
	}
	else if(_Qubp_11x1[2] > 0.)
	{
		if(rspmax > _Qub_window.maxRSP)
			rspmax = _Qub_window.maxRSP + MARGIN;
	}

	if(_Qubp_11x1[3] < 0.)
	{
		if(rsrmin < _Qub_window.minRSR)
			rsrmin = _Qub_window.minRSR - MARGIN;
	}
	else if(_Qubp_11x1[3] > 0.)
	{
		if(rsrmax > _Qub_window.maxRSR)
			rsrmax = _Qub_window.maxRSR + MARGIN;
	}

	if(_Qubp_11x1[4] < 0.)
	{
		if(rsymin < _Qub_window.minRSY)
			rsymin = _Qub_window.minRSY - MARGIN;
	}
	else if(_Qubp_11x1[4] > 0.)
	{
		if(rsymax > _Qub_window.maxRSY)
			rsymax = _Qub_window.maxRSY + MARGIN;
	}

	if(_Qubp_11x1[5] < 0.)
	{
		if(rebmin < _Qub_window.minREB)
			rebmin = _Qub_window.minREB - MARGIN;
	}
	else if(_Qubp_11x1[5] > 0.)
	{
		if(rebmax > _Qub_window.maxREB)
			rebmax = _Qub_window.maxREB + MARGIN;
	}

	if(_Qubp_11x1[6] < 0.)
	{
		if(rwymin < _Qub_window.minRWY)
			rwymin = _Qub_window.minRWY - MARGIN;
	}
	else if(_Qubp_11x1[6] > 0.)
	{
		if(rwymax > _Qub_window.maxRWY)
			rwymax = _Qub_window.maxRWY + MARGIN;
	}

	
	if(_Qubp_11x1[7] < 0.)
	{
		if(lspmin < _Qub_window.minLSP)
			lspmin = _Qub_window.minLSP - MARGIN;
	}
	else if(_Qubp_11x1[7] > 0.)
	{
		if(lspmax > _Qub_window.maxLSP)
			lspmax = _Qub_window.maxLSP + MARGIN;
	}

	if(_Qubp_11x1[8] < 0.)
	{
		if(lsrmin < _Qub_window.minLSR)
			lsrmin = _Qub_window.minLSR - MARGIN;
	}
	else if(_Qubp_11x1[8] > 0.)
	{
		if(lsrmax > _Qub_window.maxLSR)
			lsrmax = _Qub_window.maxLSR + MARGIN;
	}

	if(_Qubp_11x1[9] < 0.)
	{
		if(lsymin < _Qub_window.minLSY)
			lsymin = _Qub_window.minLSY - MARGIN;
	}
	else if(_Qubp_11x1[9] > 0.)
	{
		if(lsymax > _Qub_window.maxLSY)
			lsymax = _Qub_window.maxLSY + MARGIN;
	}

	if(_Qubp_11x1[10] < 0.)
	{
		if(lebmin < _Qub_window.minLEB)
			lebmin = _Qub_window.minLEB - MARGIN;
	}
	else if(_Qubp_11x1[10] > 0.)
	{
		if(lebmax > _Qub_window.maxLEB)
			lebmax = _Qub_window.maxLEB + MARGIN;
	}

	if(_Qubp_11x1[11] < 0.)
	{
		if(lwymin < _Qub_window.minLWY)
			lwymin = _Qub_window.minLWY - MARGIN;
	}
	else if(_Qubp_11x1[11] > 0.)
	{
		if(lwymax > _Qub_window.maxLWY)
			lwymax = _Qub_window.maxLWY + MARGIN;
	}


	if(_RWPp < 0.)
	{
		if(rwpmin < _Qub_window.minRWP)
			rwpmin = _Qub_window.minRWP - MARGIN;
	}
	else if(_RWPp > 0.)
	{
		if(rwpmax > _Qub_window.maxRWP)
			rwpmax = _Qub_window.maxRWP + MARGIN;
	}

	if(_LWPp < 0.)
	{
		if(lwpmin < _Qub_window.minLWP)
			lwpmin = _Qub_window.minLWP - MARGIN;
	}
	else if(_LWPp > 0.)
	{
		if(lwpmax > _Qub_window.maxLWP)
			lwpmax = _Qub_window.maxLWP + MARGIN;
	}

	
	JAVLC(_Qub_11x1[1], _Qubp_11x1[1],  des_WSTpp+KV_MOCAP*(des_WSTp-_Qubp_11x1[1])+KP_MOCAP*(des_WST-_Qub_11x1[1]),   WSTpmax, WSTppmax, wstmin, wstmax, MARGIN, &Qubpp_11x1[1]);
	JAVLC(_Qub_11x1[2], _Qubp_11x1[2],  des_RSPpp+KV_MOCAP*(des_RSPp-_Qubp_11x1[2])+KP_MOCAP*(des_RSP-_Qub_11x1[2]),   RSPpmax, RSPppmax, rspmin, rspmax, MARGIN, &Qubpp_11x1[2]);
	JAVLC(_Qub_11x1[3], _Qubp_11x1[3],  des_RSRpp+KV_MOCAP*(des_RSRp-_Qubp_11x1[3])+KP_MOCAP*(des_RSR-_Qub_11x1[3]),   RSRpmax, RSRppmax, rsrmin, rsrmax, MARGIN, &Qubpp_11x1[3]);
	JAVLC(_Qub_11x1[4], _Qubp_11x1[4],  des_RSYpp+KV_MOCAP*(des_RSYp-_Qubp_11x1[4])+KP_MOCAP*(des_RSY-_Qub_11x1[4]),   RSYpmax, RSYppmax, rsymin, rsymax, MARGIN, &Qubpp_11x1[4]);
	JAVLC(_Qub_11x1[5], _Qubp_11x1[5],  des_REBpp+KV_MOCAP*(des_REBp-_Qubp_11x1[5])+KP_MOCAP*(des_REB-_Qub_11x1[5]),   REPpmax, REPppmax, rebmin, rebmax, MARGIN, &Qubpp_11x1[5]);
	JAVLC(_Qub_11x1[6], _Qubp_11x1[6],  des_RWYpp+KV_MOCAP*(des_RWYp-_Qubp_11x1[6])+KP_MOCAP*(des_RWY-_Qub_11x1[6]),   REYpmax, REYppmax, rwymin, rwymax, MARGIN, &Qubpp_11x1[6]);
	JAVLC(_Qub_11x1[7], _Qubp_11x1[7],  des_LSPpp+KV_MOCAP*(des_LSPp-_Qubp_11x1[7])+KP_MOCAP*(des_LSP-_Qub_11x1[7]),   LSPpmax, LSPppmax, lspmin, lspmax, MARGIN, &Qubpp_11x1[7]);
	JAVLC(_Qub_11x1[8], _Qubp_11x1[8],  des_LSRpp+KV_MOCAP*(des_LSRp-_Qubp_11x1[8])+KP_MOCAP*(des_LSR-_Qub_11x1[8]),   LSRpmax, LSRppmax, lsrmin, lsrmax, MARGIN, &Qubpp_11x1[8]);
	JAVLC(_Qub_11x1[9], _Qubp_11x1[9],  des_LSYpp+KV_MOCAP*(des_LSYp-_Qubp_11x1[9])+KP_MOCAP*(des_LSY-_Qub_11x1[9]),   LSYpmax, LSYppmax, lsymin, lsymax, MARGIN, &Qubpp_11x1[9]);
	JAVLC(_Qub_11x1[10],_Qubp_11x1[10], des_LEBpp+KV_MOCAP*(des_LEBp-_Qubp_11x1[10])+KP_MOCAP*(des_LEB-_Qub_11x1[10]), LEPpmax, LEPppmax, lebmin, lebmax, MARGIN, &Qubpp_11x1[10]);
	JAVLC(_Qub_11x1[11],_Qubp_11x1[11], des_LWYpp+KV_MOCAP*(des_LWYp-_Qubp_11x1[11])+KP_MOCAP*(des_LWY-_Qub_11x1[11]), LEYpmax, LEYppmax, lwymin, lwymax, MARGIN, &Qubpp_11x1[11]);

	JAVLC(_RWP,_RWPp, des_RWPpp+KV_MOCAP*(des_RWPp-_RWPp)+KP_MOCAP*(des_RWP-_RWP), RWPpmax, RWPppmax, rwpmin, rwpmax, MARGIN, &RWPpp);
	JAVLC(_LWP,_LWPp, des_LWPpp+KV_MOCAP*(des_LWPp-_LWPp)+KP_MOCAP*(des_LWP-_LWP), LWPpmax, LWPppmax, lwpmin, lwpmax, MARGIN, &LWPpp);

	


	
	//------------ Supporting
	mult_smv(-KV_SUPP, (const float**)_jRFlb_6x18,6,18, _Qlbp_18x1, temp1_6x1);
	diff_vv(des_pRFC_3x1,3, pRFC_3x1, temp3_3x1);
	QTdel(des_qF, qRF_4x1, temp4_3x1);		
	mult_mv((const float**)_jRFlbp_6x18,6,18, _Qlbp_18x1, temp2_6x1);

	Xsupp_12x1[1] = temp1_6x1[1] + KP_SUPP*temp3_3x1[1] - temp2_6x1[1];
	Xsupp_12x1[2] = temp1_6x1[2] + KP_SUPP*temp3_3x1[2] - temp2_6x1[2];
	Xsupp_12x1[3] = temp1_6x1[3] + KP_SUPP*temp3_3x1[3] - temp2_6x1[3];
	Xsupp_12x1[4] = temp1_6x1[4] + KP_SUPP*temp4_3x1[1] - temp2_6x1[4];
	Xsupp_12x1[5] = temp1_6x1[5] + KP_SUPP*temp4_3x1[2] - temp2_6x1[5];
	Xsupp_12x1[6] = temp1_6x1[6] + KP_SUPP*temp4_3x1[3] - temp2_6x1[6];

	mult_smv(-KV_SUPP, (const float**)_jLFlb_6x18,6,18, _Qlbp_18x1, temp1_6x1);
	diff_vv(des_pLFC_3x1,3, pLFC_3x1, temp3_3x1);
	QTdel(des_qF, qLF_4x1, temp4_3x1);
	mult_mv((const float**)_jLFlbp_6x18,6,18, _Qlbp_18x1, temp2_6x1);

	Xsupp_12x1[7] = temp1_6x1[1] + KP_SUPP*temp3_3x1[1] - temp2_6x1[1];
	Xsupp_12x1[8] = temp1_6x1[2] + KP_SUPP*temp3_3x1[2] - temp2_6x1[2];
	Xsupp_12x1[9] = temp1_6x1[3] + KP_SUPP*temp3_3x1[3] - temp2_6x1[3];
	Xsupp_12x1[10] = temp1_6x1[4] + KP_SUPP*temp4_3x1[1] - temp2_6x1[4];
	Xsupp_12x1[11] = temp1_6x1[5] + KP_SUPP*temp4_3x1[2] - temp2_6x1[5];
	Xsupp_12x1[12] = temp1_6x1[6] + KP_SUPP*temp4_3x1[3] - temp2_6x1[6];
	//-----------------

	//------------ COM				
	mult_mv((const float**)_jCOMlb_3x18,3,18, _Qlbp_18x1, temp3_3x1);
	mult_mv((const float**)_jCOMub_3x11,3,11, _Qubp_11x1, temp4_3x1);
	sum_vv(temp3_3x1,3, temp4_3x1, vCOM_3x1);

	diff_vv(_pCOM0_3x1,3, pCOM_3x1, temp3_3x1);
	sum_svsv(-KV_COM_RECOV,vCOM_3x1,3, KP_COM_RECOV,temp3_3x1, ref_aCOM_3x1);

	ZLC(pCOM_3x1[1], vCOM_3x1[1], ref_aCOM_3x1[1], pCOM_3x1[3], 0., ZMPXMIN, ZMPXMAX, 0., 0.01f, &ref_aCOM_3x1[1]);
	ZLC(pCOM_3x1[2], vCOM_3x1[2], ref_aCOM_3x1[2], pCOM_3x1[3], 0., ZMPYMIN, ZMPYMAX, 0., 0.01f, &ref_aCOM_3x1[2]);

	mult_mv((const float**)_jCOMlbp_3x18,3,18, _Qlbp_18x1, temp3_3x1);
	mult_mv((const float**)_jCOMubp_3x11,3,11, _Qubp_11x1, temp4_3x1);
	sum_vv(temp3_3x1,3,temp4_3x1, temp3_3x1);
	mult_mv((const float**)_jCOMub_3x11, 3,11, Qubpp_11x1, temp4_3x1);
	Xcom_2x1[1] = ref_aCOM_3x1[1] - temp3_3x1[1] - temp4_3x1[1];
	Xcom_2x1[2] = ref_aCOM_3x1[2] - temp3_3x1[2] - temp4_3x1[2];
	//-----------------

	//--------- PEL
	temp5_4x1[1] = (float)cos(des_BP/2.f);	// desired PEL orientation
	temp5_4x1[2] = 0.;
	temp5_4x1[3] = (float)sin(des_BP/2.f);
	temp5_4x1[4] = 0.;
	QTdel(temp5_4x1, &_Qlb_19x1[3], temp3_3x1);

	Xpel_3x1[1] = KV_PEL_RECOV*(0.f-_Qlbp_18x1[4]) + KP_PEL_RECOV*temp3_3x1[1];
	Xpel_3x1[2] = KV_PEL_RECOV*(des_BPp-_Qlbp_18x1[5]) + KP_PEL_RECOV*temp3_3x1[2];
	Xpel_3x1[3] = KV_PEL_RECOV*(0.f-_Qlbp_18x1[6]) + KP_PEL_RECOV*temp3_3x1[3];
	//-----------------

	//--------- PCz
	Xpcz_1x1 = KV_PCZ_RECOV*(-_Qlbp_18x1[3]) + KP_PCZ_RECOV*(_pPCz0 - _Qlb_19x1[3]);
	//-----------------


	Xaug_18x1[1] = Xsupp_12x1[1];
	Xaug_18x1[2] = Xsupp_12x1[2];
	Xaug_18x1[3] = Xsupp_12x1[3];
	Xaug_18x1[4] = Xsupp_12x1[4];
	Xaug_18x1[5] = Xsupp_12x1[5];
	Xaug_18x1[6] = Xsupp_12x1[6];
	Xaug_18x1[7] = Xsupp_12x1[7];
	Xaug_18x1[8] = Xsupp_12x1[8];
	Xaug_18x1[9] = Xsupp_12x1[9];
	Xaug_18x1[10] = Xsupp_12x1[10];
	Xaug_18x1[11] = Xsupp_12x1[11];
	Xaug_18x1[12] = Xsupp_12x1[12];
	Xaug_18x1[13] = Xcom_2x1[1];
	Xaug_18x1[14] = Xcom_2x1[2];
	Xaug_18x1[15] = Xpel_3x1[1];
	Xaug_18x1[16] = Xpel_3x1[2];
	Xaug_18x1[17] = Xpel_3x1[3];
	Xaug_18x1[18] = Xpcz_1x1;

	subs_m((const float**)_jAUG_18x18,18,18, _jAUGi_18x18);
	subs_v(Xaug_18x1, 18, Qlbpp_18x1);

	gaussj_mod(_jAUGi_18x18,18,Qlbpp_18x1);
	
	isLimited = FALSE;
	if( JAVLC(_Qlb_19x1[8], _Qlbp_18x1[7], Qlbpp_18x1[7], RHYpmax, RHYppmax, RHYmin, RHYmax, D2R, &Qlbpp_18x1[7]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[9], _Qlbp_18x1[8], Qlbpp_18x1[8], RHRpmax, RHRppmax, RHRmin, RHRmax, D2R, &Qlbpp_18x1[8]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[10], _Qlbp_18x1[9], Qlbpp_18x1[9], RHPpmax, RHPppmax, RHPmin, RHPmax, D2R, &Qlbpp_18x1[9]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[11], _Qlbp_18x1[10], Qlbpp_18x1[10], RKNpmax, RKNppmax, RKNmin, RKNmax, D2R, &Qlbpp_18x1[10]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[12], _Qlbp_18x1[11], Qlbpp_18x1[11], RAPpmax, RAPppmax, RAPmin, RAPmax, D2R, &Qlbpp_18x1[11]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[13], _Qlbp_18x1[12], Qlbpp_18x1[12], RARpmax, RARppmax, RARmin, RARmax, D2R, &Qlbpp_18x1[12]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[14], _Qlbp_18x1[13], Qlbpp_18x1[13], LHYpmax, LHYppmax, LHYmin, LHYmax, D2R, &Qlbpp_18x1[13]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[15], _Qlbp_18x1[14], Qlbpp_18x1[14], LHRpmax, LHRppmax, LHRmin, LHRmax, D2R, &Qlbpp_18x1[14]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[16], _Qlbp_18x1[15], Qlbpp_18x1[15], LHPpmax, LHPppmax, LHPmin, LHPmax, D2R, &Qlbpp_18x1[15]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[17], _Qlbp_18x1[16], Qlbpp_18x1[16], LKNpmax, LKNppmax, LKNmin, LKNmax, D2R, &Qlbpp_18x1[16]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[18], _Qlbp_18x1[17], Qlbpp_18x1[17], LAPpmax, LAPppmax, LAPmin, LAPmax, D2R, &Qlbpp_18x1[17]) )
		isLimited = TRUE;
	if( JAVLC(_Qlb_19x1[19], _Qlbp_18x1[18], Qlbpp_18x1[18], LARpmax, LARppmax, LARmin, LARmax, D2R, &Qlbpp_18x1[18]) )
		isLimited = TRUE;


	if(isLimited)
	{
		subs_m((const float**)_jAUG_18x18,18,18, _jAUGi_18x18);
		//subs_v(Xaug_18x1, 18, Qlbpp_18x1);

		trans(1.f, (const float**)_jAUG_18x18,14,18, _TEMP3_18x18);
		mult_mm((const float**)_jAUG_18x18,14,18, (const float**)_TEMP3_18x18,14, _TEMP4_18x18);
		inv(_TEMP4_18x18,14);
		mult_mm((const float**)_TEMP3_18x18,18,14, (const float**)_TEMP4_18x18,14, _jCONST_DSPi_18x14);
		mult_mm((const float**)_jCONST_DSPi_18x14,18,14, (const float**)_jAUG_18x18,18, _TEMP3_18x18);
		diff_mm((const float**)_EYE_18,18,18, (const float**)_TEMP3_18x18, _NCONST_DSP_18x18);


		for(j=1; j<=18; j++)
		{
			_jTASK_18x18[1][j] = _jPELlb_att_3x18[1][j];
			_jTASK_18x18[2][j] = _jPELlb_att_3x18[2][j];
			_jTASK_18x18[3][j] = _jPELlb_att_3x18[3][j];
			_jTASK_18x18[4][j] = jPCZlb_1x18[j];
		}
		mult_mm((const float**)_jTASK_18x18,4,18, (const float**)_NCONST_DSP_18x18,18, _jTASKN_18x18);
		trans(1.f, (const float**)_jTASKN_18x18,4,18, _TEMP3_18x18);
		mult_mm((const float**)_jTASKN_18x18,4,18, (const float**)_TEMP3_18x18,4, _TEMP4_18x18);
		inv(_TEMP4_18x18,4);
		mult_mm((const float**)_TEMP3_18x18,18,4, (const float**)_TEMP4_18x18,4, _jTASKNi_18x18);

		//subs_v(Xaug_18x1,14, temp8_18x1);
		mult_mv((const float**)_jCONST_DSPi_18x14,18,14, Xaug_18x1, Qlbpp_const_18x1);

		temp7_4x1[1] = Xpel_3x1[1];
		temp7_4x1[2] = Xpel_3x1[2];
		temp7_4x1[3] = Xpel_3x1[3];
		temp7_4x1[4] = Xpcz_1x1;

		mult_mv((const float**)_jTASK_18x18,4,18, Qlbpp_const_18x1, temp8_18x1);
		diff_vv(temp7_4x1,4, temp8_18x1, temp7_4x1);
		mult_mv((const float**)_jTASKNi_18x18,18,4, temp7_4x1, Qlbpp_task_18x1);

		sum_vv(Qlbpp_const_18x1,18, Qlbpp_task_18x1, Qlbpp_18x1);

		i=0;
		while(isLimited)
		{
			isLimited = FALSE;
			if( JAVLC(_Qlb_19x1[8], _Qlbp_18x1[7], Qlbpp_18x1[7], RHYpmax, RHYppmax, RHYmin, RHYmax, D2R, &Qlbpp_18x1[7]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[9], _Qlbp_18x1[8], Qlbpp_18x1[8], RHRpmax, RHRppmax, RHRmin, RHRmax, D2R, &Qlbpp_18x1[8]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[10], _Qlbp_18x1[9], Qlbpp_18x1[9], RHPpmax, RHPppmax, RHPmin, RHPmax, D2R, &Qlbpp_18x1[9]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[11], _Qlbp_18x1[10], Qlbpp_18x1[10], RKNpmax, RKNppmax, RKNmin, RKNmax, D2R, &Qlbpp_18x1[10]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[12], _Qlbp_18x1[11], Qlbpp_18x1[11], RAPpmax, RAPppmax, RAPmin, RAPmax, D2R, &Qlbpp_18x1[11]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[13], _Qlbp_18x1[12], Qlbpp_18x1[12], RARpmax, RARppmax, RARmin, RARmax, D2R, &Qlbpp_18x1[12]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[14], _Qlbp_18x1[13], Qlbpp_18x1[13], LHYpmax, LHYppmax, LHYmin, LHYmax, D2R, &Qlbpp_18x1[13]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[15], _Qlbp_18x1[14], Qlbpp_18x1[14], LHRpmax, LHRppmax, LHRmin, LHRmax, D2R, &Qlbpp_18x1[14]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[16], _Qlbp_18x1[15], Qlbpp_18x1[15], LHPpmax, LHPppmax, LHPmin, LHPmax, D2R, &Qlbpp_18x1[15]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[17], _Qlbp_18x1[16], Qlbpp_18x1[16], LKNpmax, LKNppmax, LKNmin, LKNmax, D2R, &Qlbpp_18x1[16]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[18], _Qlbp_18x1[17], Qlbpp_18x1[17], LAPpmax, LAPppmax, LAPmin, LAPmax, D2R, &Qlbpp_18x1[17]) )
				isLimited = TRUE;
			if( JAVLC(_Qlb_19x1[19], _Qlbp_18x1[18], Qlbpp_18x1[18], LARpmax, LARppmax, LARmin, LARmax, D2R, &Qlbpp_18x1[18]) )
				isLimited = TRUE;

			if(isLimited)
			{
				mult_mv((const float**)_NCONST_DSP_18x18,18,18, Qlbpp_18x1, temp8_18x1);
				sum_vv(Qlbpp_const_18x1,18, temp8_18x1, Qlbpp_18x1);
				
				i++;
				if(i>500)
				{
					isLimited = FALSE;
					wberror("Whole-body motion convergence failure!!\n");
				}
			}
		}
	}
			
	_Qlb_19x1[1] += _Qlbp_18x1[1]*DT + 0.5f*Qlbpp_18x1[1]*DT*DT;
	_Qlb_19x1[2] += _Qlbp_18x1[2]*DT + 0.5f*Qlbpp_18x1[2]*DT*DT;
	_Qlb_19x1[3] += _Qlbp_18x1[3]*DT + 0.5f*Qlbpp_18x1[3]*DT*DT;

	wPELnorm  = (float)sqrt(_Qlbp_18x1[4]*_Qlbp_18x1[4] + _Qlbp_18x1[5]*_Qlbp_18x1[5] + _Qlbp_18x1[6]*_Qlbp_18x1[6]);

	if(wPELnorm < 1e-6)
	{
		Wmatrix(&Qlbpp_18x1[3], _Wpmat_4x4);
		mult_smv(0.25f*DT*DT, (const float**)_Wpmat_4x4,4,4, &_Qlb_19x1[3], temp5_4x1);
		_Qlb_19x1[4] += temp5_4x1[1];
		_Qlb_19x1[5] += temp5_4x1[2];
		_Qlb_19x1[6] += temp5_4x1[3];
		_Qlb_19x1[7] += temp5_4x1[4];
	}
	else
	{
		Wmatrix(&_Qlbp_18x1[3], _Wmat_4x4);
		Wmatrix(&Qlbpp_18x1[3], _Wpmat_4x4);
		mult_mv((const float**)_Wmat_4x4,4,4, &_Qlb_19x1[3], temp5_4x1);
		mult_mv((const float**)_Wpmat_4x4,4,4, &_Qlb_19x1[3], temp6_4x1);
		mult_mv((const float**)_Wmat_4x4,4,4, temp6_4x1, temp7_4x1);

		sum_svsv((float)cos(wPELnorm*DT*0.5f), &_Qlb_19x1[3],4, 1.f/wPELnorm*(float)sin(wPELnorm*DT*0.5f),temp5_4x1, &_Qlb_19x1[3]);
		sum_svsv(1.f, &_Qlb_19x1[3],4, 2.f/(wPELnorm*wPELnorm)*(1.f-(float)cos(wPELnorm*DT*0.5f)),temp6_4x1, &_Qlb_19x1[3]);
		sum_svsv(1.f, &_Qlb_19x1[3],4, 1.f/(wPELnorm*wPELnorm)*(DT-2.f/wPELnorm*(float)sin(wPELnorm*DT*0.5f)), temp7_4x1, &_Qlb_19x1[3]);
	}

	temp = (float)sqrt(_Qlb_19x1[4]*_Qlb_19x1[4] + _Qlb_19x1[5]*_Qlb_19x1[5] + _Qlb_19x1[6]*_Qlb_19x1[6] + _Qlb_19x1[7]*_Qlb_19x1[7]);
	_Qlb_19x1[4] /= temp;
	_Qlb_19x1[5] /= temp;
	_Qlb_19x1[6] /= temp;
	_Qlb_19x1[7] /= temp;

	_Qlb_19x1[8] += _Qlbp_18x1[7]*DT + 0.5f*Qlbpp_18x1[7]*DT*DT;
	_Qlb_19x1[9] += _Qlbp_18x1[8]*DT + 0.5f*Qlbpp_18x1[8]*DT*DT;
	_Qlb_19x1[10] += _Qlbp_18x1[9]*DT + 0.5f*Qlbpp_18x1[9]*DT*DT;
	_Qlb_19x1[11] += _Qlbp_18x1[10]*DT + 0.5f*Qlbpp_18x1[10]*DT*DT;
	_Qlb_19x1[12] += _Qlbp_18x1[11]*DT + 0.5f*Qlbpp_18x1[11]*DT*DT;
	_Qlb_19x1[13] += _Qlbp_18x1[12]*DT + 0.5f*Qlbpp_18x1[12]*DT*DT;
	_Qlb_19x1[14] += _Qlbp_18x1[13]*DT + 0.5f*Qlbpp_18x1[13]*DT*DT;
	_Qlb_19x1[15] += _Qlbp_18x1[14]*DT + 0.5f*Qlbpp_18x1[14]*DT*DT;
	_Qlb_19x1[16] += _Qlbp_18x1[15]*DT + 0.5f*Qlbpp_18x1[15]*DT*DT;
	_Qlb_19x1[17] += _Qlbp_18x1[16]*DT + 0.5f*Qlbpp_18x1[16]*DT*DT;
	_Qlb_19x1[18] += _Qlbp_18x1[17]*DT + 0.5f*Qlbpp_18x1[17]*DT*DT;
	_Qlb_19x1[19] += _Qlbp_18x1[18]*DT + 0.5f*Qlbpp_18x1[18]*DT*DT;

	_Qlbp_18x1[1] += DT*Qlbpp_18x1[1];
	_Qlbp_18x1[2] += DT*Qlbpp_18x1[2];
	_Qlbp_18x1[3] += DT*Qlbpp_18x1[3];
	_Qlbp_18x1[4] += DT*Qlbpp_18x1[4];
	_Qlbp_18x1[5] += DT*Qlbpp_18x1[5];
	_Qlbp_18x1[6] += DT*Qlbpp_18x1[6];
	_Qlbp_18x1[7] += DT*Qlbpp_18x1[7];
	_Qlbp_18x1[8] += DT*Qlbpp_18x1[8];
	_Qlbp_18x1[9] += DT*Qlbpp_18x1[9];
	_Qlbp_18x1[10] += DT*Qlbpp_18x1[10];
	_Qlbp_18x1[11] += DT*Qlbpp_18x1[11];
	_Qlbp_18x1[12] += DT*Qlbpp_18x1[12];
	_Qlbp_18x1[13] += DT*Qlbpp_18x1[13];
	_Qlbp_18x1[14] += DT*Qlbpp_18x1[14];
	_Qlbp_18x1[15] += DT*Qlbpp_18x1[15];
	_Qlbp_18x1[16] += DT*Qlbpp_18x1[16];
	_Qlbp_18x1[17] += DT*Qlbpp_18x1[17];
	_Qlbp_18x1[18] += DT*Qlbpp_18x1[18];

	_Qub_11x1[1] += _Qubp_11x1[1]*DT + 0.5f*Qubpp_11x1[1]*DT*DT;
	_Qub_11x1[2] += _Qubp_11x1[2]*DT + 0.5f*Qubpp_11x1[2]*DT*DT;
	_Qub_11x1[3] += _Qubp_11x1[3]*DT + 0.5f*Qubpp_11x1[3]*DT*DT;
	_Qub_11x1[4] += _Qubp_11x1[4]*DT + 0.5f*Qubpp_11x1[4]*DT*DT;
	_Qub_11x1[5] += _Qubp_11x1[5]*DT + 0.5f*Qubpp_11x1[5]*DT*DT;
	_Qub_11x1[6] += _Qubp_11x1[6]*DT + 0.5f*Qubpp_11x1[6]*DT*DT;
	_Qub_11x1[7] += _Qubp_11x1[7]*DT + 0.5f*Qubpp_11x1[7]*DT*DT;
	_Qub_11x1[8] += _Qubp_11x1[8]*DT + 0.5f*Qubpp_11x1[8]*DT*DT;
	_Qub_11x1[9] += _Qubp_11x1[9]*DT + 0.5f*Qubpp_11x1[9]*DT*DT;
	_Qub_11x1[10] += _Qubp_11x1[10]*DT + 0.5f*Qubpp_11x1[10]*DT*DT;
	_Qub_11x1[11] += _Qubp_11x1[11]*DT + 0.5f*Qubpp_11x1[11]*DT*DT;

	_RWP += _RWPp*DT + 0.5f*RWPpp*DT*DT;
	_LWP += _LWPp*DT + 0.5f*LWPpp*DT*DT;


	_Qubp_11x1[1] += DT*Qubpp_11x1[1];
	_Qubp_11x1[2] += DT*Qubpp_11x1[2];
	_Qubp_11x1[3] += DT*Qubpp_11x1[3];
	_Qubp_11x1[4] += DT*Qubpp_11x1[4];
	_Qubp_11x1[5] += DT*Qubpp_11x1[5];
	_Qubp_11x1[6] += DT*Qubpp_11x1[6];
	_Qubp_11x1[7] += DT*Qubpp_11x1[7];
	_Qubp_11x1[8] += DT*Qubpp_11x1[8];
	_Qubp_11x1[9] += DT*Qubpp_11x1[9];
	_Qubp_11x1[10] += DT*Qubpp_11x1[10];
	_Qubp_11x1[11] += DT*Qubpp_11x1[11];

	_RWPp += DT*RWPpp;
	_LWPp += DT*LWPpp;
	
	Joint[RHY].RefAngleCurrent = _Qlb_19x1[8]*R2D;
	Joint[RHR].RefAngleCurrent = _Qlb_19x1[9]*R2D;
	Joint[RHP].RefAngleCurrent = _Qlb_19x1[10]*R2D;
	Joint[RKN].RefAngleCurrent = _Qlb_19x1[11]*R2D;
	Joint[RAP].RefAngleCurrent = _Qlb_19x1[12]*R2D;
	Joint[RAR].RefAngleCurrent = _Qlb_19x1[13]*R2D;
	Joint[LHY].RefAngleCurrent = _Qlb_19x1[14]*R2D;
	Joint[LHR].RefAngleCurrent = _Qlb_19x1[15]*R2D;
	Joint[LHP].RefAngleCurrent = _Qlb_19x1[16]*R2D;
	Joint[LKN].RefAngleCurrent = _Qlb_19x1[17]*R2D;
	Joint[LAP].RefAngleCurrent = _Qlb_19x1[18]*R2D;
	Joint[LAR].RefAngleCurrent = _Qlb_19x1[19]*R2D;

	Joint[WST].RefAngleCurrent = _Qub_11x1[1]*R2D;
	Joint[RSP].RefAngleCurrent = _Qub_11x1[2]*R2D;
	Joint[RSR].RefAngleCurrent = _Qub_11x1[3]*R2D - OFFSET_RSR;
	Joint[RSY].RefAngleCurrent = _Qub_11x1[4]*R2D;
	Joint[REB].RefAngleCurrent = _Qub_11x1[5]*R2D;
	Joint[RWY].RefAngleCurrent = _Qub_11x1[6]*R2D;
	Joint[RWP].RefAngleCurrent = _RWP*R2D;
	Joint[LSP].RefAngleCurrent = _Qub_11x1[7]*R2D;
	Joint[LSR].RefAngleCurrent = _Qub_11x1[8]*R2D - OFFSET_LSR;
	Joint[LSY].RefAngleCurrent = _Qub_11x1[9]*R2D;
	Joint[LEB].RefAngleCurrent = _Qub_11x1[10]*R2D;
	Joint[LWY].RefAngleCurrent = _Qub_11x1[11]*R2D;
	Joint[LWP].RefAngleCurrent = _LWP*R2D;

	Joint[NKY].RefVelCurrent = _Qub_window.dNK_JW_3x1[1];
	Joint[NK1].RefVelCurrent = _Qub_window.dNK_JW_3x1[2];
	Joint[NK2].RefVelCurrent = _Qub_window.dNK_JW_3x1[3];

	Joint[RF1].RefVelCurrent = _Qub_window.dRF_JW_5x1[1];
	Joint[RF2].RefVelCurrent = _Qub_window.dRF_JW_5x1[2];
	Joint[RF3].RefVelCurrent = _Qub_window.dRF_JW_5x1[3];
	Joint[RF4].RefVelCurrent = _Qub_window.dRF_JW_5x1[4];
	Joint[RF5].RefVelCurrent = _Qub_window.dRF_JW_5x1[5];

	Joint[LF1].RefVelCurrent = _Qub_window.dLF_JW_5x1[1];
	Joint[LF2].RefVelCurrent = _Qub_window.dLF_JW_5x1[2];
	Joint[LF3].RefVelCurrent = _Qub_window.dLF_JW_5x1[3];
	Joint[LF4].RefVelCurrent = _Qub_window.dLF_JW_5x1[4];
	Joint[LF5].RefVelCurrent = _Qub_window.dLF_JW_5x1[5];

	return 0;
}



int InitQubWindow(float wst, float rsp, float rsr_, float rsy, float reb, float rwy, float rwp,
				     float lsp, float lsr_, float lsy, float leb, float lwy, float lwp,
					 float dNKY, float dNK1, float dNK2,
					 float dRF1, float dRF2, float dRF3, float dRF4, float dRF5,
					 float dLF1, float dLF2, float dLF3, float dLF4, float dLF5,
					 float body_pitch)
{
	unsigned int i;

	float rsr = rsr_ + OFFSET_RSR*D2R;
	float lsr = lsr_ + OFFSET_LSR*D2R;

	
	for(i=0; i<WIND_SIZE_WST; i++)			
		_Qub_window.wind_WST[i] = wst;
	for(i=0; i<WIND_SIZE_RSP; i++)			
		_Qub_window.wind_RSP[i] = rsp;
	for(i=0; i<WIND_SIZE_RSR; i++)
		_Qub_window.wind_RSR[i] = rsr;
	for(i=0; i<WIND_SIZE_RSY; i++)
		_Qub_window.wind_RSY[i] = rsy;
	for(i=0; i<WIND_SIZE_REB; i++)
		_Qub_window.wind_REB[i] = reb;
	for(i=0; i<WIND_SIZE_RWY; i++)
		_Qub_window.wind_RWY[i] = rwy;
	for(i=0; i<WIND_SIZE_RWP; i++)
		_Qub_window.wind_RWP[i] = rwp;
	for(i=0; i<WIND_SIZE_LSP; i++)
		_Qub_window.wind_LSP[i] = lsp;
	for(i=0; i<WIND_SIZE_LSR; i++)	
		_Qub_window.wind_LSR[i] = lsr;
	for(i=0; i<WIND_SIZE_LSY; i++)
		_Qub_window.wind_LSY[i] = lsy;
	for(i=0; i<WIND_SIZE_LEB; i++)
		_Qub_window.wind_LEB[i] = leb;
	for(i=0; i<WIND_SIZE_LWY; i++)	
		_Qub_window.wind_LWY[i] = lwy;
	for(i=0; i<WIND_SIZE_LWP; i++)
		_Qub_window.wind_LWP[i] = lwp;
	for(i=0; i<WIND_SIZE_BP; i++)
		_Qub_window.wind_BP[i] = 0.;

	_Qub_window.dNK_JW_3x1[1] = dNKY;
	_Qub_window.dNK_JW_3x1[2] = dNK1;
	_Qub_window.dNK_JW_3x1[3] = dNK2;

	_Qub_window.dLF_JW_5x1[1] = dLF1;
	_Qub_window.dLF_JW_5x1[2] = dLF2;
	_Qub_window.dLF_JW_5x1[3] = dLF3;
	_Qub_window.dLF_JW_5x1[4] = dLF4;
	_Qub_window.dLF_JW_5x1[5] = dLF5;

	_Qub_window.dRF_JW_5x1[1] = dRF1;
	_Qub_window.dRF_JW_5x1[2] = dRF2;
	_Qub_window.dRF_JW_5x1[3] = dRF3;
	_Qub_window.dRF_JW_5x1[4] = dRF4;
	_Qub_window.dRF_JW_5x1[5] = dRF5;

	_Qub_window.head_WST = 0;
	_Qub_window.head_RSP = 0;
	_Qub_window.head_RSR = 0;
	_Qub_window.head_RSY = 0;
	_Qub_window.head_REB = 0;
	_Qub_window.head_RWY = 0;
	_Qub_window.head_RWP = 0;
	_Qub_window.head_LSP = 0;
	_Qub_window.head_LSR = 0;
	_Qub_window.head_LSY = 0;
	_Qub_window.head_LEB = 0;
	_Qub_window.head_LWY = 0;
	_Qub_window.head_LWP = 0;
	_Qub_window.head_BP = 0;

	_Qub_window.tail_WST = 4;
	_Qub_window.tail_RSP = 4;
	_Qub_window.tail_RSR = 4;
	_Qub_window.tail_RSY = 4;
	_Qub_window.tail_REB = 4;
	_Qub_window.tail_RWY = 4;
	_Qub_window.tail_RWP = 4;
	_Qub_window.tail_LSP = 4;
	_Qub_window.tail_LSR = 4;
	_Qub_window.tail_LSY = 4;
	_Qub_window.tail_LEB = 4;
	_Qub_window.tail_LWY = 4;
	_Qub_window.tail_LWP = 4;
	_Qub_window.tail_BP = 4;

	_Qub_window.index_maxWST = 0;
	_Qub_window.index_maxRSP = 0;
	_Qub_window.index_maxRSR = 0;
	_Qub_window.index_maxRSY = 0;
	_Qub_window.index_maxREB = 0;
	_Qub_window.index_maxRWY = 0;
	_Qub_window.index_maxRWP = 0;
	_Qub_window.index_maxLSP = 0;
	_Qub_window.index_maxLSR = 0;
	_Qub_window.index_maxLSY = 0;
	_Qub_window.index_maxLEB = 0;
	_Qub_window.index_maxLWY = 0;
	_Qub_window.index_maxLWP = 0;

	_Qub_window.index_minWST = 0;
	_Qub_window.index_minRSP = 0;
	_Qub_window.index_minRSR = 0;
	_Qub_window.index_minRSY = 0;
	_Qub_window.index_minREB = 0;
	_Qub_window.index_minRWY = 0;
	_Qub_window.index_minRWP = 0;
	_Qub_window.index_minLSP = 0;
	_Qub_window.index_minLSR = 0;
	_Qub_window.index_minLSY = 0;
	_Qub_window.index_minLEB = 0;
	_Qub_window.index_minLWY = 0;
	_Qub_window.index_minLWP = 0;

	_Qub_window.maxWST = wst;
	_Qub_window.maxRSP = rsp;
	_Qub_window.maxRSR = rsr;
	_Qub_window.maxRSY = rsy;
	_Qub_window.maxREB = reb;
	_Qub_window.maxRWY = rwy;
	_Qub_window.maxRWP = rwp;
	_Qub_window.maxLSP = lsp;
	_Qub_window.maxLSR = lsr;
	_Qub_window.maxLSY = lsy;
	_Qub_window.maxLEB = leb;
	_Qub_window.maxLWY = lwy;
	_Qub_window.maxLWP = lwp;

	_Qub_window.minWST = wst;
	_Qub_window.minRSP = rsp;
	_Qub_window.minRSR = rsr;
	_Qub_window.minRSY = rsy;
	_Qub_window.minREB = reb;
	_Qub_window.minRWY = rwy;
	_Qub_window.minRWP = rwp;
	_Qub_window.minLSP = lsp;
	_Qub_window.minLSR = lsr;
	_Qub_window.minLSY = lsy;
	_Qub_window.minLEB = leb;
	_Qub_window.minLWY = lwy;
	_Qub_window.minLWP = lwp;

	return 0;

}

