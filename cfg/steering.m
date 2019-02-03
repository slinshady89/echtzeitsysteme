y =[-1000 -925 -850 -775 -700 -625 -550 -475 -400 -325 -250 -175 -100 -50 -25 -10 100 175 250 325 400 475 550 625 700 775 850 925 1000];
x = [22 21 19 18 17 16 15 13 11 9.5 8 6 4 3 2 1 -1 -2.5 -4 -6 -7.5 -9 -10.5 -13.5 -15.5 -17.5 -20 -20.5 -22];
size(x)
size(y)
[p,S] = polyfit(x,y,7)
hold on
plot(x,y,'r-')
new_x = 22:1:-22;
new_y = polyval(p,x);
plot(x, new_y, 'b.')
title('-0.001x^5-0.0045x^4+0.0257x^3+0.6983x^2-44.0491x+57.8917')
xlabel('steering Angle in Deg °')
ylabel('steering control input')
grid on