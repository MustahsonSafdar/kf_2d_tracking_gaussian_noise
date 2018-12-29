function z_pred = RadarCartesianToPolar(x_state)
   %{
   * convert radar measurements from cartesian coordinates (x, y, vx, vy) to
   * polar (rho, phi, rho_dot) coordinates
   %}

   px = x_state(1);
   py = x_state(2);
   vx = x_state(3);
   vy = x_state(4);

   rho = sqrt(px*px + py*py);
   phi = atan2(px,py);  % returns values between -pi and pi

    % if rho is very small, set it to 0.0001 to afunction division by 0 in computing rho_dot
   if(rho < 0.000001)
       rho = 0.000001;
   end

   rho_dot = (px * vx + py * vy) / rho;

   z_pred = [rho; phi; rho_dot];

end