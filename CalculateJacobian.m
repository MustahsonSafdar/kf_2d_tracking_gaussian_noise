function  Hj = CalculateJacobian(x_state)

   Hj = zeros(3,4);
   
    %recover state parameters
    px = x_state(1);
    py = x_state(2);
    vx = x_state(3);
    vy = x_state(4);

    %pre-compute a set of terms to avoid repeated calculation
    c1 = px*px+py*py;
    c2 = sqrt(c1);
    c3 = (c1*c2);

    % check division by zero
    if(abs(c1) < 0.0001)
        disp( "Function CalculateJacobian() has Error: Division by Zero");
        return;
    end

    % compute the Jacobian matrix
    % Here, conventional cartensian axis are considered
    Hj = [px/c2,                 py/c2,                  0,      0;
          py/c1,                 -px/c1,                  0,      0; 
          py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3,  px/c2,  py/c2];

end