w = 100 * 2 * pi  ;
r = 1.414 ;

t = tf([w^2], [1, r * w, w^2]) ;

bode(t) ;