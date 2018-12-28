
function [x_k_k_1, P_k_k_1] = predict(x_k_k, P_k_k, F_)

    x_k_k_1 = F_ * x_k_k;
    P_k_k_1 = F_ * P_k_k * transpose(F_);
    
end