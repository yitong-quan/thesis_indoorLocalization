
// Trilateration with distances D(m, n), n nodes, m signals
function [M, S] = newton_distancemap(D, M0, S0, fixedMics)
	
	[m, n] = size(D); // Number of m signals, n nodes
	max_steps = 300;

	// Create initial positions
	M = 0.001 * (rand(p, n) - 0.5);
	if size(M) == size(M0) then
		M = M0;
	end;
	S = 0.001 * (rand(p, m) - 0.5);
	if size(S) == size(S0) then
		S = S0;
	end;

	for u = 1:max_steps
	
		// Initialize Newton arrays
		A = zeros(m * n, p * (n + m));
		b = zeros(m * n, 1);
	
		for i = 1:n
			for j = 1:m

				index = (i - 1) * m + j;
				x_norm = norm(M(:, i) - S(:, j));
				
				// Create Jacobian Q: Derivative of f_ij
				// d f_ij / d x_(i/j) == 0 if k != i && k != j
				if ~fixedMics then // Mics can be moved?
					A(index, p*(i-1)+1:p*i) = (M(:, i) - S(:, j))' / x_norm'; // d f_ij / d x_i
				end
				A(index, p*n+p*(j-1)+1:p*n+p*j) = (-M(:, i) + S(:, j))' / x_norm'; // d f_ij / d x_j
				//printf("Fill row %d, col_i %d-%d, col_j %d-%d, size (%d, %d)\n", index, p*(i-1)+1, p*i, p*n+p*(j-1)+1, p*n+p*j, size(A, 1), size(A, 2));
				
				// Create b
				// Evaluate function f_ij = || M_i - S_j || - d_ij
				b(index) = x_norm - D(j, i);
				
			end; // for j
		end; // for i

		//disp(A);
		
		// Calculate x = A^{-1} * b using least squares method
		if 1 & u > 100 then
			A = A * 4;
			x = pinv(A' * A) * (A' * b);
		else
			x = 0.1 * A' * b; // <== Gradient method
		end;
		
		// Update result vector with update vector
		for i = 1:n
			M(:, i) = M(:, i) - x(p*(i-1)+1:i*p);
		end;
		for j = 1:m
			S(:, j) = S(:, j) - x(p*n+p*(j-1)+1:p*n+p*j);
		end;

		if u < 10 | modulo(u, 10) == 0 then
			printf("Counter: %d, norm x: %.6f, norm b: %.6f\n", u, norm(x), norm(b));
		end;
		 
		if norm(x) < 0.000001 then
			break;
		end;
			
	end; // for u
	
	printf("Newton finished, %d steps, remaining error b: %.6f\n", u, norm(b));
		
endfunction;

// 1st derivative phi' = d phi / d (dim)
function X = phiPrime(pM, pS)
	X = -(pM - pS)' / norm(pM - pS); 
endfunction;

// Gradient + Newton iteration with TDOA data
// Parameters: n mics, m sources, tM(n, m), M(p, n), S(p, m)
function [M, S] = newton_hyperbola(tM, M, S, fixedMics)

	// Read n, m, assume p to be global
	[n, m] = size(tM);
	
	// If no initialization is given create random central positions
	if size(M, 2) < n then
		M = 0.001 * (rand(p, n) - 0.5);
	end;
	if size(S, 2) < m then
		S = 0.001 * (rand(p, m) - 0.5);
	end;
	
	max_steps = 300;
	counter = 0;
	
	while 1 do
		
		b = zeros(m * (n - 1), 1);
		A = zeros(m * (n - 1), p * ((n - 1) + m));
		
		for i = 2:n // Constraints: Mikes
			for j = 1:m // Constraints: Sources
			
				index = (i - 2) * m + j;
				
				// Create b
				//printf("b-index: %d\n", index);
				b(index) = norm(M(:, 1) - S(:, j)) - norm(M(:, i) - S(:, j)) - (tM(1, j) - tM(i, j));
				
				// Create A
				//printf("A-index (src): (%d, %d:%d)\n", index, (j - 1) * p + 1, j * p);
				A(index, (j - 1) * p + 1 : j * p) = phiPrime(M(:, 1), S(:, j)) - phiPrime(M(:, i), S(:, j));
				
				if ~fixedMics then // Mics can be moved?
					//printf("A-index (mic): (%d, %d:%d)\n", index, p * m + (i - 2) * p + 1, p * m + (i - 1) * p);
					A(index, p * m + (i - 2) * p + 1 : p * m + (i - 1) * p) = phiPrime(M(:, i), S(:, j)); 
				end; // fixed mics
			
			end;
		end;
		
		//printf("Have rank: %d, size (%d, %d)\n", rank(A), size(A, 1), size(A, 2));
						
		// Calculate x = A^{-1} * b using least squares method
		if 1 & counter > -1 then
			A = A * 1;
			x = pinv(A' * A) * (A' * b);
		else
			x = 0.1 * A' * b; // <== Gradient method
		end;
				
		// Update position vectors: (S/M) += x'
		for j = 1:m // Sources
			index = (j - 1) * p + 1;
			S(:, j) = S(:, j) - x(index : index + (p - 1));
		end;
		for i = 2:n // Mikes
			index = p * m + (i - 2) * p + 1;
			M(:, i) = M(:, i) - x(index : index + (p - 1));
		end;
			
		counter = counter + 1;
		
		if counter < 10 | modulo(counter, 10) == 0 then
			printf("Counter: %d, norm x: %.6f, norm b: %.6f\n", counter, norm(x), norm(b));
		end;
		 
		if norm(x) < 0.000001 | counter >= max_steps then 
			break;
		end;
			
	end; // while
	
	printf("Newton finished, %d steps\n", counter);
	
endfunction;


