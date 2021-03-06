void dynamics_forcontrol( double *x, double *f,  void  *user_data ){

    // x[0] -> time t
    // x[1] -> px
    // x[2] -> py
    // x[3] -> v
    // x[4] -> psi
    // x[5] -> L
    // x[6] -> u(1), a
    // x[7] -> u(2), psi_dot
    
    double t = x[0];
    double px = x[1];
    double py = x[2];
    double v = x[3];
    double psi = x[4];
    //double L = x[5];   //the cost function 
    double a = x[5];
    double psi_dot = x[6];
    
    double dis_dynamics[4];  //disturbance 
    
    int i;
    
    for (i=0; i<4; i++)
    {
        dis_dynamics[i]= x[i+7];
    }
    
     //user_data[0]=0;
    
    double ref[3];
    
    double vx = 20;
    double vy = 0;
    
    double px_ref = vx*(t);
    double py_ref = 5* sin(t);
    double psi_ref;
     
    double px_dot_ref = vx;
    double py_dot_ref = 5*cos(t);
     
    double v_i_ref = sqrt(px_dot_ref*px_dot_ref + py_dot_ref*py_dot_ref);
      
     if (v_i_ref!=0)
     {
        double sin_psi = py_dot_ref/v_i_ref; 
        double cos_psi = px_dot_ref/v_i_ref; 
        if (sin_psi > 0)
            psi_ref = acos(cos_psi);
        else if (sin_psi < 0)
            psi_ref = -acos(cos_psi);
        else if (sin_psi == 0)
        {
            if (cos_psi < 0)
                psi_ref = -3.14159265; 
            else
                psi_ref = 0;
        }
      }
     else
     {
        psi_ref = 0; 
     }    
    
    
    ref[0] = 10*t;
    ref[1] = 0;
    ref[2] = 10; 
    ref[3] = 0;
    
    
    //ref[0] = px_ref; 
    //ref[1] = py_ref;
    //ref[2] = v_i_ref; 
    //ref[3] = psi_ref;
    
    
    double delta_x = (px-ref[0])* (px-ref[0]) +  (py -ref[1])* (py-ref[1]) + (v-ref[2])* (v-ref[2]) + (psi-ref[3])* (psi-ref[3]);

    f[0] =  v*cos(psi);       //dot(px)
    f[1] =  v*sin(psi);       //dot(py)
    f[2] =  a;              //dot(v)
    f[3] =  psi_dot;                    //dot(psi)
   // f[4] =  10*delta_x+  2*a*a + 2*psi_dot*psi_dot;       //dot(L)
    
}
