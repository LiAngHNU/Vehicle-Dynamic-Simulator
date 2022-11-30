function pos = get_intersection(l1, l2)
a1 = l1(2);
b1 = l1(1);
c1 = l1(3);
a2 = l2(2);
b2 = l2(1);
c2 = l2(3);
x0 = (b1*c2-b2*c1)/(a1*b2-a2*b1);
y0 = (c1*a2-c2*a1)/(a1*b2-a2*b1);
pos = [x0, y0];
end