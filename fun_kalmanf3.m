function s=fun_kalmanf3(s,p,r1,r2,r3,r4,rc,rk1,rk2,rk3,rk4,rck1,Laplacian)
 


   K=s.x(4,1);
   dt=0.05;
   d=0;

    s.A=[1 (rc - rck1)' 0 0 0 0; 0 1 0 0 0 0 0; 0 0 1 0 0 0 0; 0 0 0 1 0 0 0; 0 0 0 0 1 (rc - rck1)';0 0 0 0 0 1 0;0 0 0 0 0 0 1];
   s.C=[1 (r1-rc)' 0 0 0 0;
        1 (r2-rc)' 0 0 0 0;
        1 (r3-rc)' 0 0 0 0;
        1 (r4-rc)' 0 0 0 0;
        0 0 0 0 1 (rk1-rc)';
        0 0 0 0 1 (rk2-rc)';
        0 0 0 0 1 (rk3-rc)';
        0 0 0 0 1 (rk4-rc)'];
    
  
    
 
    s.D=[0.5*(kron((r1-rc),(r1-rc))') 0 0 0 0;
         0.5*(kron((r2-rc),(r2-rc))') 0 0 0 0;
         0.5*(kron((r3-rc),(r3-rc))') 0 0 0 0;
         0.5*(kron((r4-rc),(r4-rc))') 0 0 0 0;
         0 0 0 0 0.5*(kron((rk1-rc),(rk1-rc))');
         0 0 0 0 0.5*(kron((rk2-rc),(rk2-rc))');
         0 0 0 0 0.5*(kron((rk3-rc),(rk3-rc))');
         0 0 0 0 0.5*(kron((rk4-rc),(rk4-rc))')];
     
                
 
 s.H=Laplacian*[1 1 1 1 1 1 1 1]';
    

    
    s.G=[1 0 0 -Laplacian*dt -1 0 0];
    
    h=[K*Laplacian;
       Laplacian*eye(2)*(rc-rck1);
       0;
       K*Laplacian;
       Laplacian*eye(2)*(rc-rck1)];  
    

   
    s.R=.5*eye(8);
    s.W=.5*eye(8);
    s.M=.5*eye(7);
    s.U=.5*eye(8);
    
   % Prediction for state vector and covariance:
   s.x = s.A*s.x + h;
   s.P = s.A*s.P*s.A' + s.M;

   % Compute Kalman gain factor:
   K = s.P*s.C'*inv(s.C*s.P*s.C'+s.D*s.U*s.D'+s.R);
       
   % Correction based on observation:
   s.x = s.x + K*(p - s.C*s.x - s.D*s.H);
   s.P = inv(inv(s.P) + s.C'*inv(s.D*s.U*s.D' + s.R)*s.C);
   
   %constrain
   s.x= s.x -s.P*s.G'*inv(s.G*s.P*s.G')*(s.G*s.x-d)
   


   
   
end


