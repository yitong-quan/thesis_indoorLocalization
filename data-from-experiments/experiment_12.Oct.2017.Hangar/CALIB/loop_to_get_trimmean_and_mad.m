Tri = zeros(1,7);
Mad = zeros(1,7);

for i = 2:7
    digits(2);
    expriN = vpa(2 + i/10);
    [trim_mean, m_a_d] = read_file_generate_matrix(expriN);
    Tri(i) = trim_mean;
    Mad(i) = m_a_d;
end    