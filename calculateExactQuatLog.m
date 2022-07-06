%% Calculating the exact log_q(eta) function

function eta = calculateExactQuatLog(q)

if(abs(q(1)) < 1)
    angle = acos(q(1));
    axis = norm(q(2:4), 2);
    eta = (angle/axis) * q(2:4);
else
   eta = q(2:4); 
end
end