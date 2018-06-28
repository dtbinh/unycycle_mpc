/**
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
 *    Author: David Ariens  --  http://www.acadotoolkit.org/matlab 
 *    Date: 2010
 *
 *    EMPTY TEMPLATE    
 *
 *    Compilation:
 *     - Go to the folder <ACADOtoolkit-inst-dir>/interfaces/matlab/
 *     - Run: makemex('examples/mexfiles/empty.cpp', 'empty', 'examples/mexfiles/');
 *     - Run: cd ('examples/mexfiles/');
 *     - Run: empty();
 */

//#include <acado_toolkit.hpp>                    // Include the ACADO toolkit
//#include <acado/utils/matlab_acado_utils.hpp>   // Include specific Matlab utils

//USING_NAMESPACE_ACADO                           // Open the namespace

#include "mex.h"
// Start the MEX function. Do NOT change the header of this function.
//
// int nlhs contains the number of left hand arguments (outputs *plhs[])
// int nrhs contains the number of right hand arguments (inputs *prhs[])
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
    //clearAllStaticCounters( );                  // Clear software counters
 
    
    
    
    // WRITE YOUR CPP CODE HERE
    printf("Hello world! \n");
    // WRITE YOUR CPP CODE HERE
    
    int i; 
    
    i=1; 
    
    if(i>=0)
        i = i + 2 -2 *3*i;
  
    
	double multiplier;              /* input scalar */
    double *inMatrix;               /* 1xN input matrix */
    size_t ncols;                   /* size of matrix */
    double *outMatrix;              /* output matrix */
    
        /* get the value of the scalar input  */
    multiplier = mxGetScalar(prhs[0]);

    /* create a pointer to the real data in the input matrix  */
    inMatrix = mxGetPr(prhs[0]);

    /* get dimensions of the input matrix */
    ncols = mxGetN(prhs[0]);

    /* create the output matrix */
    plhs[0] = mxCreateDoubleMatrix(1,(mwSize)ncols,mxREAL);

    /* get a pointer to the real data in the output matrix */
    outMatrix = mxGetPr(plhs[0]);
    
    /*
    for(i=0; i<3; i++)
    {
        *(outMatrix+i) = *(inMatrix+i);
    }
     */
    
   // outMatrix[0] = ncols;
  //  outMatrix[1]=2;
    
    
    int cc;
    double matrix_line; 
    double  psi_;
    double v; 
    v = *inMatrix;
        cc = 1; 
    if (v>cc)
        //matrix_line = [cos(psi_), -v*sin(psi_); sin(psi_),  v*cos(psi_)]; 
        v = 2;
    else
       // matrix_line = [cos(psi_), -cc*sin(psi_); sin(psi_),  cc*cos(psi_)]; 
        v = 2;
     
    outMatrix[0] = v;
    outMatrix[1]=2;
    

    /* call the computational routine */
    //arrayProduct(multiplier,inMatrix,outMatrix,(mwSize)ncols);
    
    

    //clearAllStaticCounters( );                  // Clear software counters
} 

