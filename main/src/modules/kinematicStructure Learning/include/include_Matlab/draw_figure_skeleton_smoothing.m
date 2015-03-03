figure(5001)
set(gcf,'color','w');
clf
c = flipud(c);
mesh(c)
view(0,-90);
grid off
axis off
hold on
plot(sk_idx_w,height-sk_idx_h, 'Marker','.','Color','w','MarkerSize',5);