void dynamics( double *x, double *f,  void  *user_data ){

    // x[0] -> time t
    // x[1] -> xp_dot
    // x[2] -> yp_dot
    // x[3] -> psi_dot
    // x[4] -> epsi
    // x[5] -> ey
    // x[6] -> s 
    // x[7] -> L
    // x[8] -> u(1), delta_f
    // x[9] -> u(2), a_x
    
    /*
    %states:
xp_dot = y(1);  %lateral speed
yp_dot = y(2);  %longitudinal speed
psi_dot = y(3); 
epsi = y(4);
ey= y(5);  %lateral position
s = y(6);  %logitudinal position 
    
        delta_f = u(1);   %steering angle 
a_x = u(2);    %acc 
     */
        
    if(x[1]<= 0)
    {
        x[1]= 1e-5;
    }
    
    double t = x[0];
    double xp_dot = x[1];
    double yp_dot = x[2];
    double psi_dot = x[3];
    double epsi = x[4];
    double ey = x[5];
    double s = x[6];
    double L = x[7];   //the cost function 
    double delta_f = x[8];
    double a_x = x[9];
    
 //constant: 
    double a = 1.41; 
    double b = 1.576; 
    double mu =0.5; 
    double Fzf = 21940/2; 
    double Fzr = 21940/2; 
    double cf = 65000; 
    double cr = 65000; 
    double m = 2194; 
    double Iz = 4770; 
    double psi_dot_com = 0;
    double p =Iz/(m*b);
    
    //reference trajectory: 
    double ref[5];
    
    double vx = 10;
    double vy = 0;
    
    double px_ref = vx*(t);
    double py_ref = 0* sin(1*t);
    double psi_ref;
    double psi_dot_ref; 
     
    double px_dot_ref = vx;
    double py_dot_ref = 0*cos(1*t);
    
    double px_ddot_ref = 0; 
    double py_ddot_ref = -0*sin(1*t); 
     
    double v_i_ref = sqrt(px_dot_ref*px_dot_ref + py_dot_ref*py_dot_ref);
      
     if (v_i_ref!=0)
     {
        double sin_psi = py_dot_ref/v_i_ref; 
        double cos_psi = px_dot_ref/v_i_ref; 
        double cos_psi_dot = (px_ddot_ref*v_i_ref- px_dot_ref*(px_dot_ref*px_ddot_ref+py_dot_ref*py_ddot_ref)/v_i_ref)
        /v_i_ref/v_i_ref;
        double sin_psi_dot = (py_ddot_ref*v_i_ref- py_dot_ref*(px_dot_ref*px_ddot_ref+py_dot_ref*py_ddot_ref)/v_i_ref)
        /v_i_ref/v_i_ref;
        
        if (sin_psi > 0){
            psi_ref = acos(cos_psi);
            psi_dot_ref = -1/sqrt(1-cos_psi*cos_psi)*cos_psi_dot;
        }
        else if (sin_psi < 0){
            psi_ref = -acos(cos_psi);
            psi_dot_ref = 1/sqrt(1-cos_psi*cos_psi)*cos_psi_dot;
        }
        else if (sin_psi == 0)
        {
            if (cos_psi < 0){
                psi_ref = -3.14159265; 
                psi_dot_ref = -1/sqrt(1-sin_psi*sin_psi)*sin_psi_dot;
            }
            else{
                psi_ref = 0;
                psi_dot_ref = 1/sqrt(1-sin_psi*sin_psi)*sin_psi_dot;
            }
        }
     }
     else
     {
        psi_ref = 0; 
     }      
     
    //psi_ref = 0;  //test 
    ref[0] = px_ref; 
    ref[1] = py_ref;
    ref[2] = v_i_ref;   //vx
    ref[3] = psi_ref;   //psi
    ref[4] = 0; 
    ref[5] = psi_dot_ref;  
       
    //state cost function: 
    double delta_x = 20*(s-ref[0])* (s-ref[0]) +  30*(ey -ref[1])* (ey-ref[1]) + 2*(xp_dot-ref[2])* (xp_dot-ref[2]) + 2*(epsi-ref[3])* (epsi-ref[3])
                     + 0.2*(yp_dot-ref[4])* (yp_dot-ref[4]) + 2*(psi_dot-ref[5])* (psi_dot-ref[5]);
    
     /*
    f[0] = yp_dot*psi_dot + a_x;   //dot xp_dot
    f[1] =  -2*(cf+cr)/(m*xp_dot)*yp_dot-2*(a*cf-b*cr)/m/xp_dot*psi_dot-xp_dot*psi_dot + 2*cf/m*delta_f;   // dot yp_dot
    f[2] =  -2*(a*cf-b*cr)/Iz/xp_dot*yp_dot-2*(a*a*cf+b*b*cr)/Iz/xp_dot*psi_dot + 2*a*cf/Iz*delta_f;   //dot  psi_dot
    f[3] =  psi_dot - psi_dot_com;    // dot epsi
    f[4] =  yp_dot*cos(epsi) + xp_dot*sin(epsi);    // dot ey 
    f[5] =  xp_dot*cos(epsi)-yp_dot*sin(epsi);     // dot s 
    f[6] =  1*delta_x+  10*delta_f*delta_f + 10*a_x*a_x;       //dot(L)
    //notice the gain in the cost functions should be carefully tunned. 
      */
    
    //linerized model: 
    f[0] =  a_x;   //dot xp_dot
    f[1] =  -2*(cf+cr)/(m*xp_dot)*yp_dot-2*(a*cf-b*cr)/m/xp_dot*psi_dot + 2*cf/m*delta_f;   // dot yp_dot
    f[2] =  -2*(a*cf-b*cr)/Iz/xp_dot*yp_dot-2*(a*a*cf+b*b*cr)/Iz/xp_dot*psi_dot + 2*a*cf/Iz*delta_f;   //dot  psi_dot
    f[3] =  psi_dot - psi_dot_com;    // dot epsi
    f[4] =  yp_dot + xp_dot*epsi;    // dot ey 
    f[5] =  xp_dot ;     // dot s 
    f[6] =  0.01*delta_x+  20*delta_f*delta_f + 20*a_x*a_x;       //dot(L)
    //notice the gain in the cost functions should be carefully tunned. 
     
    
}
