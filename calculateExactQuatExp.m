%% Calculating the exact exp_q(eta) function

function q = calculateExactQuatExp(eta)

angle = zeros(size(eta, 2), 1);
q = zeros(4, size(eta, 2));

for i = 1 : size(eta, 2)
    angle(i) = norm(eta(:, i), 2);
    axis = eta(:, i) ./ angle(i);
    if(angle(i) < 1e-5)
        q(:, i) = [1; eta(:, i)];
    else
        q(:, i) = [cos(angle(i)); axis*sin(angle(i))];
    end
end

end