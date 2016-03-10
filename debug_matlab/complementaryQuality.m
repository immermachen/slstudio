warning('off', 'Images:initSize:adjustingMag');
clear all;
close all;
%%
dir = '~/slstudio/build-SLStudio-Desktop-Debug/experiment/Set1/dataCapturedForTest/';
threshold_complementary = 0.01; %TODO
numImg = 2050*2448;
black = double(imread(strcat(dir,'0_61.bmp') ) ); %20 21, 50 51,
A = double(imread(strcat(dir,'0_00.bmp') ) ); %20 21, 50 51,
B = double(imread(strcat(dir,'0_01.bmp') ) );
A = double(imread(strcat(dir,'0_02.bmp') ) );
B = double(imread(strcat(dir,'0_03.bmp') ) );
 A = double(imread(strcat(dir,'0_04.bmp') ) );
 B = double(imread(strcat(dir,'0_05.bmp') ) );
 A = double(imread(strcat(dir,'0_06.bmp') ) );
 B = double(imread(strcat(dir,'0_07.bmp') ) );
 A = double(imread(strcat(dir,'0_08.bmp') ) );
 B = double(imread(strcat(dir,'0_09.bmp') ) );
% A = double(imread(strcat(dir,'0_16.bmp') ) ); %improved_procent =  0.0069031 improved_num =  11022
% B = double(imread(strcat(dir,'0_17.bmp') ) );

% the following images need to sobel filter to sharpen edges; 
%A = double(imread(strcat(dir,'0_18.bmp') ) ); % improved_procent =  0.0082797 improved_num =  13220
%B = double(imread(strcat(dir,'0_19.bmp') ) );
%A = double(imread(strcat(dir,'0_20.bmp') ) ); % improved_procent =  0.016408
%B = double(imread(strcat(dir,'0_21.bmp') ) );

% A = double(imread(strcat(dir,'0_50.bmp') ) );
% B = double(imread(strcat(dir,'0_51.bmp') ) );
% A = double(imread(strcat(dir,'1_48.bmp') ) );
% B = double(imread(strcat(dir,'1_49.bmp') ) );
% A = double(imread(strcat(dir,'1_50.bmp') ) );
% B = double(imread(strcat(dir,'1_51.bmp') ) );
A60 = double(imread( strcat(dir,'0_60.bmp') ) );
B61 = double(imread( strcat(dir,'0_61.bmp') ) );
diff = A60-B61;   Maskfull = diff > 10;
%figure; imshow(Maskfull); title('basic mask'); figure; imshow(uint8(A)); title('full image');
%A = A.*Maskfull;  B = B.*Maskfull;
numAvail = sum(sum(Maskfull))

%sharpen image 
H = fspecial('sobel'); % better than fspecial('prewitt'); and %A1 = imsharpen(A); A1 = imadjust(A);
H = H'; % for vertical direction 
A1 = imfilter(A,H); B1 = imfilter(B,H); 
figure; imshow(uint8(A)); title('full masked image');
figure; imshow(uint8(A1)); title('full masked image and filter');

if(1)
diff = A-B;
mask = diff>0;
figure;  imshow(uint8(diff));  title('diff');
figure;  imshow(mask);         title('mask');
stable_count = sum(sum(mask))  %  
stable_procent = stable_count/numAvail %0.3141

A = A1; B = B1;
 
diff = A-B;
mask = diff>0;
figure;  imshow(uint8(diff));  title('diff_Filter');
figure;  imshow(mask);         title('mask_Filter');
stable_count1 = sum(sum(mask))  %  
stable_procent1 = stable_count1/numAvail %0.3141

improved_num = stable_count1 - stable_count
improved_procent = stable_procent1 - stable_procent
end

if(0)  % see how well the sobel filter
diff = abs(A-B);%imabsdiff(A,B) only for uint8, if use abs, the image must be convert to double;
maxVal0 = max(max(A))
maxVal1 = max(max(B))
maxval = max(maxVal0, maxVal1);
threshold = threshold_complementary * maxval
mask = diff>threshold;
figure;  imshow(uint8(diff));  title('diff');
figure;  imshow(mask);         title('mask');
stable_count = sum(sum(mask))  %  1422489  1637042 1612619
error_procent = stable_count/numImg %0.3141

A = A1; B = B1;
 
diff = abs(A-B);%imabsdiff(A,B) only for uint8, if use abs, the image must be convert to double;
maxVal0 = max(max(A))
maxVal1 = max(max(B))
maxval = max(maxVal0, maxVal1);
threshold = threshold_complementary * maxval
mask = diff>threshold;
figure; imshow(uint8(diff)); title('diff improved');
figure; imshow(mask);      title('mask improved');
stable_count1 = sum(sum(mask))  %  1422489  1637042 1612619
error_procent1 = stable_count1/numImg %0.3141
improved_num = stable_count1 - stable_count
end

if(0)
% see 
hp1 = B(1000,:);
x = 1:1:2448;
figure;
hold on;
plot(x,hp1,'r*');  %one row
plot(x,hp1,'b-');  %one row
hold off
x = 1:1:2050;
end

if(0) % see the diff of complementary two images
diff=A-B;
hp1 = diff(1000,:);
x = 1:1:2448;
figure;
hold on;
plot(x,hp1,'r*');  %one row
plot(x,hp1,'b-');  %one row
hold off
x = 1:1:2050;
end

if(0) %!!!!!! using full black background, this does not work!!!!!!! for Prof.
threshold_complementary = 0.1; %TODO
diff = abs(A-black);%imabsdiff(A,B) only for uint8, if use abs, the image must be convert to double;
maxVal0 = max(max(A))
maxVal1 = max(max(black))
maxval = max(maxVal0, maxVal1);
threshold = threshold_complementary * maxval
mask = diff>threshold;
figure;
imshow(diff);
figure;
imshow(mask);
stable_count = sum(sum(mask))
error_procent = unstable_count/numImg

hp1 = diff(1000,:);
x = 1:1:2448;
figure;
hold on;
plot(x,hp1,'r*');  %one row
plot(x,hp1,'b-');  %one row
hold off
x = 1:1:2050;
end
