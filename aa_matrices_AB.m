function [ A , B ] = aa_matrices_AB( RHS , x , t , u ,turb, wind, n , m )

del = 1.0e-3;

f0 = feval( RHS , x , t , u, turb, wind);
A = zeros(n,n);
B = zeros(n,m);
for j = 1 : n
  dx = zeros( n , 1 );
  dx(j) = del;
  A(:,j)=( feval( RHS , x + dx , t , u, turb, wind) - f0 ) / del;
end

for j = 1 : m
  du = zeros( m , 1 );
  du(j) = del;
  B(:,j)=( feval( RHS , x , t , u + du, turb, wind) - f0 ) / del;
end

end