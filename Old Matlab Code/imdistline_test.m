figure, imshow('Slide1.PNG');
h = imdistline(gca);
api = iptgetapi(h);
fcn = makeConstrainToRectFcn('imline',...
                              get(gca,'XLim'),get(gca,'YLim'));
api.setDragConstraintFcn(fcn);   