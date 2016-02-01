clc;
clear('all');
%close('all');

p = Projector(1);
c = Camera(0,0);
c.startCapture();

response = zeros(256, 1);

for i=0:255
    tex = repmat(uint8(i), [1 1 3]);
    p.displayTexture(tex);
    pause(0.1);
    I = c.getFrame();
    %I= I(900:1000,900:1000);
    response(i+1) = mean(I(:));
end

figure;
plot(response);
xlabel('Input intensity');
ylabel('Output intensity');




























