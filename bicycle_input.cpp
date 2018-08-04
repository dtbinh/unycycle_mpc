/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include "bicycle_dynamics_L.cpp"
#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 38){ 
      mexErrMsgTxt("This problem expects 38 right hand side argument(s) since you have defined 38 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState xp_dot;
    DifferentialState yp_dot;
    DifferentialState psi_dot;
    DifferentialState epsi;
    DifferentialState ey;
    DifferentialState s;
    DifferentialState L;
    Control delta_f;
    Control a_x;
    double *mexinput0_temp = NULL; 
    if( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || !(mxGetM(prhs[0])==1 && mxGetN(prhs[0])==1) ) { 
      mexErrMsgTxt("Input 0 must be a noncomplex scalar double.");
    } 
    mexinput0_temp = mxGetPr(prhs[0]); 
    double mexinput0 = *mexinput0_temp; 

    double *mexinput1_temp = NULL; 
    if( !mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) || !(mxGetM(prhs[1])==1 && mxGetN(prhs[1])==1) ) { 
      mexErrMsgTxt("Input 1 must be a noncomplex scalar double.");
    } 
    mexinput1_temp = mxGetPr(prhs[1]); 
    double mexinput1 = *mexinput1_temp; 

    double *mexinput2_temp = NULL; 
    if( !mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) || !(mxGetM(prhs[2])==1 && mxGetN(prhs[2])==1) ) { 
      mexErrMsgTxt("Input 2 must be a noncomplex scalar double.");
    } 
    mexinput2_temp = mxGetPr(prhs[2]); 
    double mexinput2 = *mexinput2_temp; 

    double *mexinput3_temp = NULL; 
    if( !mxIsDouble(prhs[3]) || mxIsComplex(prhs[3]) || !(mxGetM(prhs[3])==1 && mxGetN(prhs[3])==1) ) { 
      mexErrMsgTxt("Input 3 must be a noncomplex scalar double.");
    } 
    mexinput3_temp = mxGetPr(prhs[3]); 
    double mexinput3 = *mexinput3_temp; 

    double *mexinput4_temp = NULL; 
    if( !mxIsDouble(prhs[4]) || mxIsComplex(prhs[4]) || !(mxGetM(prhs[4])==1 && mxGetN(prhs[4])==1) ) { 
      mexErrMsgTxt("Input 4 must be a noncomplex scalar double.");
    } 
    mexinput4_temp = mxGetPr(prhs[4]); 
    double mexinput4 = *mexinput4_temp; 

    double *mexinput5_temp = NULL; 
    if( !mxIsDouble(prhs[5]) || mxIsComplex(prhs[5]) || !(mxGetM(prhs[5])==1 && mxGetN(prhs[5])==1) ) { 
      mexErrMsgTxt("Input 5 must be a noncomplex scalar double.");
    } 
    mexinput5_temp = mxGetPr(prhs[5]); 
    double mexinput5 = *mexinput5_temp; 

    double *mexinput6_temp = NULL; 
    if( !mxIsDouble(prhs[6]) || mxIsComplex(prhs[6]) || !(mxGetM(prhs[6])==1 && mxGetN(prhs[6])==1) ) { 
      mexErrMsgTxt("Input 6 must be a noncomplex scalar double.");
    } 
    mexinput6_temp = mxGetPr(prhs[6]); 
    double mexinput6 = *mexinput6_temp; 

    double *mexinput7_temp = NULL; 
    if( !mxIsDouble(prhs[7]) || mxIsComplex(prhs[7]) || !(mxGetM(prhs[7])==1 && mxGetN(prhs[7])==1) ) { 
      mexErrMsgTxt("Input 7 must be a noncomplex scalar double.");
    } 
    mexinput7_temp = mxGetPr(prhs[7]); 
    double mexinput7 = *mexinput7_temp; 

    double *mexinput8_temp = NULL; 
    if( !mxIsDouble(prhs[8]) || mxIsComplex(prhs[8]) || !(mxGetM(prhs[8])==1 && mxGetN(prhs[8])==1) ) { 
      mexErrMsgTxt("Input 8 must be a noncomplex scalar double.");
    } 
    mexinput8_temp = mxGetPr(prhs[8]); 
    double mexinput8 = *mexinput8_temp; 

    double *mexinput9_temp = NULL; 
    if( !mxIsDouble(prhs[9]) || mxIsComplex(prhs[9]) || !(mxGetM(prhs[9])==1 && mxGetN(prhs[9])==1) ) { 
      mexErrMsgTxt("Input 9 must be a noncomplex scalar double.");
    } 
    mexinput9_temp = mxGetPr(prhs[9]); 
    double mexinput9 = *mexinput9_temp; 

    double *mexinput10_temp = NULL; 
    if( !mxIsDouble(prhs[10]) || mxIsComplex(prhs[10]) || !(mxGetM(prhs[10])==1 && mxGetN(prhs[10])==1) ) { 
      mexErrMsgTxt("Input 10 must be a noncomplex scalar double.");
    } 
    mexinput10_temp = mxGetPr(prhs[10]); 
    double mexinput10 = *mexinput10_temp; 

    double *mexinput11_temp = NULL; 
    if( !mxIsDouble(prhs[11]) || mxIsComplex(prhs[11]) || !(mxGetM(prhs[11])==1 && mxGetN(prhs[11])==1) ) { 
      mexErrMsgTxt("Input 11 must be a noncomplex scalar double.");
    } 
    mexinput11_temp = mxGetPr(prhs[11]); 
    double mexinput11 = *mexinput11_temp; 

    double *mexinput12_temp = NULL; 
    if( !mxIsDouble(prhs[12]) || mxIsComplex(prhs[12]) || !(mxGetM(prhs[12])==1 && mxGetN(prhs[12])==1) ) { 
      mexErrMsgTxt("Input 12 must be a noncomplex scalar double.");
    } 
    mexinput12_temp = mxGetPr(prhs[12]); 
    double mexinput12 = *mexinput12_temp; 

    double *mexinput13_temp = NULL; 
    if( !mxIsDouble(prhs[13]) || mxIsComplex(prhs[13]) || !(mxGetM(prhs[13])==1 && mxGetN(prhs[13])==1) ) { 
      mexErrMsgTxt("Input 13 must be a noncomplex scalar double.");
    } 
    mexinput13_temp = mxGetPr(prhs[13]); 
    double mexinput13 = *mexinput13_temp; 

    double *mexinput14_temp = NULL; 
    if( !mxIsDouble(prhs[14]) || mxIsComplex(prhs[14]) || !(mxGetM(prhs[14])==1 && mxGetN(prhs[14])==1) ) { 
      mexErrMsgTxt("Input 14 must be a noncomplex scalar double.");
    } 
    mexinput14_temp = mxGetPr(prhs[14]); 
    double mexinput14 = *mexinput14_temp; 

    double *mexinput15_temp = NULL; 
    if( !mxIsDouble(prhs[15]) || mxIsComplex(prhs[15]) || !(mxGetM(prhs[15])==1 && mxGetN(prhs[15])==1) ) { 
      mexErrMsgTxt("Input 15 must be a noncomplex scalar double.");
    } 
    mexinput15_temp = mxGetPr(prhs[15]); 
    double mexinput15 = *mexinput15_temp; 

    double *mexinput16_temp = NULL; 
    if( !mxIsDouble(prhs[16]) || mxIsComplex(prhs[16]) || !(mxGetM(prhs[16])==1 && mxGetN(prhs[16])==1) ) { 
      mexErrMsgTxt("Input 16 must be a noncomplex scalar double.");
    } 
    mexinput16_temp = mxGetPr(prhs[16]); 
    double mexinput16 = *mexinput16_temp; 

    double *mexinput17_temp = NULL; 
    if( !mxIsDouble(prhs[17]) || mxIsComplex(prhs[17]) || !(mxGetM(prhs[17])==1 && mxGetN(prhs[17])==1) ) { 
      mexErrMsgTxt("Input 17 must be a noncomplex scalar double.");
    } 
    mexinput17_temp = mxGetPr(prhs[17]); 
    double mexinput17 = *mexinput17_temp; 

    double *mexinput18_temp = NULL; 
    if( !mxIsDouble(prhs[18]) || mxIsComplex(prhs[18]) || !(mxGetM(prhs[18])==1 && mxGetN(prhs[18])==1) ) { 
      mexErrMsgTxt("Input 18 must be a noncomplex scalar double.");
    } 
    mexinput18_temp = mxGetPr(prhs[18]); 
    double mexinput18 = *mexinput18_temp; 

    double *mexinput19_temp = NULL; 
    if( !mxIsDouble(prhs[19]) || mxIsComplex(prhs[19]) || !(mxGetM(prhs[19])==1 && mxGetN(prhs[19])==1) ) { 
      mexErrMsgTxt("Input 19 must be a noncomplex scalar double.");
    } 
    mexinput19_temp = mxGetPr(prhs[19]); 
    double mexinput19 = *mexinput19_temp; 

    double *mexinput20_temp = NULL; 
    if( !mxIsDouble(prhs[20]) || mxIsComplex(prhs[20]) || !(mxGetM(prhs[20])==1 && mxGetN(prhs[20])==1) ) { 
      mexErrMsgTxt("Input 20 must be a noncomplex scalar double.");
    } 
    mexinput20_temp = mxGetPr(prhs[20]); 
    double mexinput20 = *mexinput20_temp; 

    double *mexinput21_temp = NULL; 
    if( !mxIsDouble(prhs[21]) || mxIsComplex(prhs[21]) || !(mxGetM(prhs[21])==1 && mxGetN(prhs[21])==1) ) { 
      mexErrMsgTxt("Input 21 must be a noncomplex scalar double.");
    } 
    mexinput21_temp = mxGetPr(prhs[21]); 
    double mexinput21 = *mexinput21_temp; 

    double *mexinput22_temp = NULL; 
    if( !mxIsDouble(prhs[22]) || mxIsComplex(prhs[22]) || !(mxGetM(prhs[22])==1 && mxGetN(prhs[22])==1) ) { 
      mexErrMsgTxt("Input 22 must be a noncomplex scalar double.");
    } 
    mexinput22_temp = mxGetPr(prhs[22]); 
    double mexinput22 = *mexinput22_temp; 

    double *mexinput23_temp = NULL; 
    if( !mxIsDouble(prhs[23]) || mxIsComplex(prhs[23]) || !(mxGetM(prhs[23])==1 && mxGetN(prhs[23])==1) ) { 
      mexErrMsgTxt("Input 23 must be a noncomplex scalar double.");
    } 
    mexinput23_temp = mxGetPr(prhs[23]); 
    double mexinput23 = *mexinput23_temp; 

    double *mexinput24_temp = NULL; 
    if( !mxIsDouble(prhs[24]) || mxIsComplex(prhs[24]) || !(mxGetM(prhs[24])==1 && mxGetN(prhs[24])==1) ) { 
      mexErrMsgTxt("Input 24 must be a noncomplex scalar double.");
    } 
    mexinput24_temp = mxGetPr(prhs[24]); 
    double mexinput24 = *mexinput24_temp; 

    double *mexinput25_temp = NULL; 
    if( !mxIsDouble(prhs[25]) || mxIsComplex(prhs[25]) || !(mxGetM(prhs[25])==1 && mxGetN(prhs[25])==1) ) { 
      mexErrMsgTxt("Input 25 must be a noncomplex scalar double.");
    } 
    mexinput25_temp = mxGetPr(prhs[25]); 
    double mexinput25 = *mexinput25_temp; 

    double *mexinput26_temp = NULL; 
    if( !mxIsDouble(prhs[26]) || mxIsComplex(prhs[26]) || !(mxGetM(prhs[26])==1 && mxGetN(prhs[26])==1) ) { 
      mexErrMsgTxt("Input 26 must be a noncomplex scalar double.");
    } 
    mexinput26_temp = mxGetPr(prhs[26]); 
    double mexinput26 = *mexinput26_temp; 

    double *mexinput27_temp = NULL; 
    if( !mxIsDouble(prhs[27]) || mxIsComplex(prhs[27]) || !(mxGetM(prhs[27])==1 && mxGetN(prhs[27])==1) ) { 
      mexErrMsgTxt("Input 27 must be a noncomplex scalar double.");
    } 
    mexinput27_temp = mxGetPr(prhs[27]); 
    double mexinput27 = *mexinput27_temp; 

    double *mexinput28_temp = NULL; 
    if( !mxIsDouble(prhs[28]) || mxIsComplex(prhs[28]) || !(mxGetM(prhs[28])==1 && mxGetN(prhs[28])==1) ) { 
      mexErrMsgTxt("Input 28 must be a noncomplex scalar double.");
    } 
    mexinput28_temp = mxGetPr(prhs[28]); 
    double mexinput28 = *mexinput28_temp; 

    double *mexinput29_temp = NULL; 
    if( !mxIsDouble(prhs[29]) || mxIsComplex(prhs[29]) || !(mxGetM(prhs[29])==1 && mxGetN(prhs[29])==1) ) { 
      mexErrMsgTxt("Input 29 must be a noncomplex scalar double.");
    } 
    mexinput29_temp = mxGetPr(prhs[29]); 
    double mexinput29 = *mexinput29_temp; 

    double *mexinput30_temp = NULL; 
    if( !mxIsDouble(prhs[30]) || mxIsComplex(prhs[30]) || !(mxGetM(prhs[30])==1 && mxGetN(prhs[30])==1) ) { 
      mexErrMsgTxt("Input 30 must be a noncomplex scalar double.");
    } 
    mexinput30_temp = mxGetPr(prhs[30]); 
    double mexinput30 = *mexinput30_temp; 

    double *mexinput31_temp = NULL; 
    if( !mxIsDouble(prhs[31]) || mxIsComplex(prhs[31]) || !(mxGetM(prhs[31])==1 && mxGetN(prhs[31])==1) ) { 
      mexErrMsgTxt("Input 31 must be a noncomplex scalar double.");
    } 
    mexinput31_temp = mxGetPr(prhs[31]); 
    double mexinput31 = *mexinput31_temp; 

    double *mexinput32_temp = NULL; 
    if( !mxIsDouble(prhs[32]) || mxIsComplex(prhs[32]) || !(mxGetM(prhs[32])==1 && mxGetN(prhs[32])==1) ) { 
      mexErrMsgTxt("Input 32 must be a noncomplex scalar double.");
    } 
    mexinput32_temp = mxGetPr(prhs[32]); 
    double mexinput32 = *mexinput32_temp; 

    double *mexinput33_temp = NULL; 
    if( !mxIsDouble(prhs[33]) || mxIsComplex(prhs[33]) || !(mxGetM(prhs[33])==1 && mxGetN(prhs[33])==1) ) { 
      mexErrMsgTxt("Input 33 must be a noncomplex scalar double.");
    } 
    mexinput33_temp = mxGetPr(prhs[33]); 
    double mexinput33 = *mexinput33_temp; 

    double *mexinput34_temp = NULL; 
    if( !mxIsDouble(prhs[34]) || mxIsComplex(prhs[34]) || !(mxGetM(prhs[34])==1 && mxGetN(prhs[34])==1) ) { 
      mexErrMsgTxt("Input 34 must be a noncomplex scalar double.");
    } 
    mexinput34_temp = mxGetPr(prhs[34]); 
    double mexinput34 = *mexinput34_temp; 

    double *mexinput35_temp = NULL; 
    if( !mxIsDouble(prhs[35]) || mxIsComplex(prhs[35]) || !(mxGetM(prhs[35])==1 && mxGetN(prhs[35])==1) ) { 
      mexErrMsgTxt("Input 35 must be a noncomplex scalar double.");
    } 
    mexinput35_temp = mxGetPr(prhs[35]); 
    double mexinput35 = *mexinput35_temp; 

    double *mexinput36_temp = NULL; 
    if( !mxIsDouble(prhs[36]) || mxIsComplex(prhs[36]) || !(mxGetM(prhs[36])==1 && mxGetN(prhs[36])==1) ) { 
      mexErrMsgTxt("Input 36 must be a noncomplex scalar double.");
    } 
    mexinput36_temp = mxGetPr(prhs[36]); 
    double mexinput36 = *mexinput36_temp; 

    double *mexinput37_temp = NULL; 
    if( !mxIsDouble(prhs[37]) || mxIsComplex(prhs[37]) || !(mxGetM(prhs[37])==1 && mxGetN(prhs[37])==1) ) { 
      mexErrMsgTxt("Input 37 must be a noncomplex scalar double.");
    } 
    mexinput37_temp = mxGetPr(prhs[37]); 
    double mexinput37 = *mexinput37_temp; 

    DifferentialEquation acadodata_f1;
    IntermediateState setc_is_1(10);
    setc_is_1(0) = autotime;
    setc_is_1(1) = xp_dot;
    setc_is_1(2) = yp_dot;
    setc_is_1(3) = psi_dot;
    setc_is_1(4) = epsi;
    setc_is_1(5) = ey;
    setc_is_1(6) = s;
    setc_is_1(7) = L;
    setc_is_1(8) = delta_f;
    setc_is_1(9) = a_x;
    CFunction cLinkModel_1( 7, dynamics ); 
    acadodata_f1 << cLinkModel_1(setc_is_1); 

    OCP ocp1(mexinput0, mexinput1, 10);
    ocp1.minimizeMayerTerm(L);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(AT_START, xp_dot == mexinput2);
    ocp1.subjectTo(AT_START, yp_dot == mexinput3);
    ocp1.subjectTo(AT_START, psi_dot == mexinput4);
    ocp1.subjectTo(AT_START, epsi == mexinput5);
    ocp1.subjectTo(AT_START, ey == mexinput6);
    ocp1.subjectTo(AT_START, s == mexinput7);
    ocp1.subjectTo(AT_START, L == 0.00000000000000000000e+00);


    OptimizationAlgorithm algo1(ocp1);
    algo1.set( KKT_TOLERANCE, 1.000000E-10 );
    returnValue returnvalue = algo1.solve();

    VariablesGrid out_states; 
    VariablesGrid out_parameters; 
    VariablesGrid out_controls; 
    VariablesGrid out_disturbances; 
    VariablesGrid out_algstates; 
    algo1.getDifferentialStates(out_states);
    algo1.getControls(out_controls);
    const char* outputFieldNames[] = {"STATES", "CONTROLS", "PARAMETERS", "DISTURBANCES", "ALGEBRAICSTATES", "CONVERGENCE_ACHIEVED"}; 
    plhs[0] = mxCreateStructMatrix( 1,1,6,outputFieldNames ); 
    mxArray *OutS = NULL;
    double  *outS = NULL;
    OutS = mxCreateDoubleMatrix( out_states.getNumPoints(),1+out_states.getNumValues(),mxREAL ); 
    outS = mxGetPr( OutS );
    for( int i=0; i<out_states.getNumPoints(); ++i ){ 
      outS[0*out_states.getNumPoints() + i] = out_states.getTime(i); 
      for( int j=0; j<out_states.getNumValues(); ++j ){ 
        outS[(1+j)*out_states.getNumPoints() + i] = out_states(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES",OutS );
    mxArray *OutC = NULL;
    double  *outC = NULL;
    OutC = mxCreateDoubleMatrix( out_controls.getNumPoints(),1+out_controls.getNumValues(),mxREAL ); 
    outC = mxGetPr( OutC );
    for( int i=0; i<out_controls.getNumPoints(); ++i ){ 
      outC[0*out_controls.getNumPoints() + i] = out_controls.getTime(i); 
      for( int j=0; j<out_controls.getNumValues(); ++j ){ 
        outC[(1+j)*out_controls.getNumPoints() + i] = out_controls(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"CONTROLS",OutC );
    mxArray *OutP = NULL;
    double  *outP = NULL;
    OutP = mxCreateDoubleMatrix( out_parameters.getNumPoints(),1+out_parameters.getNumValues(),mxREAL ); 
    outP = mxGetPr( OutP );
    for( int i=0; i<out_parameters.getNumPoints(); ++i ){ 
      outP[0*out_parameters.getNumPoints() + i] = out_parameters.getTime(i); 
      for( int j=0; j<out_parameters.getNumValues(); ++j ){ 
        outP[(1+j)*out_parameters.getNumPoints() + i] = out_parameters(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"PARAMETERS",OutP );
    mxArray *OutW = NULL;
    double  *outW = NULL;
    OutW = mxCreateDoubleMatrix( out_disturbances.getNumPoints(),1+out_disturbances.getNumValues(),mxREAL ); 
    outW = mxGetPr( OutW );
    for( int i=0; i<out_disturbances.getNumPoints(); ++i ){ 
      outW[0*out_disturbances.getNumPoints() + i] = out_disturbances.getTime(i); 
      for( int j=0; j<out_disturbances.getNumValues(); ++j ){ 
        outW[(1+j)*out_disturbances.getNumPoints() + i] = out_disturbances(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"DISTURBANCES",OutW );
    mxArray *OutZ = NULL;
    double  *outZ = NULL;
    OutZ = mxCreateDoubleMatrix( out_algstates.getNumPoints(),1+out_algstates.getNumValues(),mxREAL ); 
    outZ = mxGetPr( OutZ );
    for( int i=0; i<out_algstates.getNumPoints(); ++i ){ 
      outZ[0*out_algstates.getNumPoints() + i] = out_algstates.getTime(i); 
      for( int j=0; j<out_algstates.getNumValues(); ++j ){ 
        outZ[(1+j)*out_algstates.getNumPoints() + i] = out_algstates(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"ALGEBRAICSTATES",OutZ );
    mxArray *OutConv = NULL;
    if ( returnvalue == SUCCESSFUL_RETURN ) { OutConv = mxCreateDoubleScalar( 1 ); }else{ OutConv = mxCreateDoubleScalar( 0 ); } 
    mxSetField( plhs[0],0,"CONVERGENCE_ACHIEVED",OutConv );


    clearAllStaticCounters( ); 
 
} 

