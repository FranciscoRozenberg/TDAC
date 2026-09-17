out=load("alpha.mat");
alphak = out.out.d1;
alphaa = out.out.d2;
alpha = out.out.d3;

%plot(alpha)

alpha_sel = alpha(4500:6500);

plot(alpha_sel)

