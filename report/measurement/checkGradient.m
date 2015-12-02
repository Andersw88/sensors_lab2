function numgrad = checkGradient(J, theta)
% Calculate numerical gradients
% theta: a vector of parameters
% J: a function that outputs a real-number. Calling y = J(theta) will 
% return the function value at theta.

numgrad = zeros(size(theta));
e = zeros(length(theta),1);
epsilon = 1e-4;

% ====================== YOUR CODE HERE ======================
% Use a for-loop to calculate the numerical gradient for each parameter in
% theta. Set the ith element in e to epsilon and calculate the numerical
% gradient. Don't forget to reset the ith element to 0 again before
% calculating the next gradient. 

for i = 1:size(theta)
    e(i) = epsilon;
%    numgrad(i) = (J(theta + e) - J(theta - e))/(2*epsilon);
    numgrad(i) = (J(theta + e) - J(theta - e))
    numgrad(i) = numgrad(i)/(2*epsilon);
    e(i) = 0;
end
%numgrad

% ============================================================

end
