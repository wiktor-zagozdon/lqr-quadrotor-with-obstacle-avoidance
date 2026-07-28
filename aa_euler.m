function y = aa_euler( RHS , x , t , dt, u, turb, wind)  

  c_bet = 0.90;
  del_state = 1.0e-6;   %  1.0e-7
  coef = - c_bet / del_state;
  n = size(x,1);
  y = zeros( n , 1 );
  Jacob = zeros( n , n );
  y0 = feval( RHS , x , t , u, turb, wind);  
  for i = 1 : n  
    tmp = x(i);
    x(i) = x(i) + del_state;
    vec = feval( RHS , x , t , u, turb, wind);
    x(i) = tmp;
    Jacob(:,i) = coef * ( vec - y0 );
  end
  Jacob = Jacob + eye(n,n) / dt;
  dx = Jacob^(-1) * y0;
  y = x + dx;

end