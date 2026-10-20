function delay = calcDelay(order, tauK)
    % how much later than t = 1 a unit ramp through an order-n lag (time constant tauK) reaches 1
    syms t tau k
    y = t - order*tau + tau*exp(-t/tau) * symsum((order - k)*(t/tau)^k / factorial(k), k, 0, order-1);
    tSol = vpasolve(subs(y, tau, tauK) == 1, t, order*0.01 + 1); % no closed form, start near n*tau + 1
    delay = double(tSol - 1);
end
