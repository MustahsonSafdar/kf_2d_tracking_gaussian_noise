function rmse = CalculateRMSE(estimations, ground_truth) 

   % Calculate the RMSE here.
   rmse = zeros(size(ground_truth,2),1);

   % check the validity of the following inputs:
   %  * the estimation vector size should not be zero
   %  * the estimation vector size should equal ground truth vector size
   if(size(estimations,2) ~= size(ground_truth,2) || size(estimations,2) == 0)
       disp( "Invalid estimation or ground_truth data");
       return;
   end

   %accumulate squared residuals

   residual = estimations - ground_truth;
   %coefficient-wise multiplication
   residual = residual.*residual;
   resiudal_sum = sum(residual);
   rmse = (resiudal_sum./size(ground_truth,1))';

end