function p = pdf(x, mu, sigma)
    coeff = 1 / (sigma * sqrt(2 * pi));
    exponent = exp(-0.5 * ((x - mu) / sigma).^2);
    p = coeff * exponent;
end