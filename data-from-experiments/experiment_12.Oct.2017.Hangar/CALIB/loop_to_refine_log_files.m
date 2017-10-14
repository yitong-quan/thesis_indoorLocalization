for j = 1:6
    for i = 1:7
        digits(2);
        expriN = vpa(j + i/10);
        data_format_refine_HTerm(expriN);
    end
end