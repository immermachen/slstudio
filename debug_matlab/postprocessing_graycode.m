warning('off', 'Images:initSize:adjustingMag');
clear all;
close all;
%% 
dir = '~/slstudio/build-SLStudio-Desktop-Debug/experiment/Set1/dataCapturedForTest/';
numImg = 2050*2448;
if(0)
  inputs = {strcat(dir,'0_21.bmp'), strcat(dir,'0_20.bmp'),strcat(dir,'0_19.bmp'),strcat(dir,'0_18.bmp'),strcat(dir,'0_17.bmp'),strcat(dir,'0_16.bmp'),strcat(dir,'0_15.bmp'),strcat(dir,'0_14.bmp')};
  outputs = {'0_21.bmp', '0_20.bmp','0_19.bmp', '0_18.bmp', '0_17.bmp', '0_16.bmp', '0_15.bmp', '0_14.bmp'};
    
  A60 = double(imread( strcat(dir,'0_60.bmp') ) );
  B61 = double(imread( strcat(dir,'0_61.bmp') ) );
  diff = A60-B61;   Maskfull = diff > 10;

  numList = size(inputs);
  numList = numList(2)
  for i=1:2:numList
    A = double(imread( inputs{i} ) );   %A = AA(:,:,i);  B = BB(:,:,i);
    B = double(imread( inputs{i+1} ) );
    
    %figure; imshow(Maskfull); title('basic mask'); figure; imshow(uint8(A)); title('full image');
    A = A.*Maskfull;  B = B.*Maskfull;
    numAvail = sum(sum(Maskfull))

    %sharpen image 
    H = fspecial('sobel'); % better than fspecial('prewitt'); and %A1 = imsharpen(A); A1 = imadjust(A);
    H = H'; % for vertical direction 
    % sharr filter instead of sobel filter    
    Stencil= [3  0 -3; 10 0 -10; 3  0 -3];    
    A1=imfilter(A,Stencil,'conv','same','replicate');
    B1=imfilter(B,Stencil,'conv','same','replicate');
    %figure; imshow(uint8(A)); title('full masked image');
    figure; imshow(uint8(A1)); title('full masked image and sobel filter');
    %figure; imshow(uint8(B)); title('full masked image');
    figure; imshow(uint8(B1)); title('full masked image and sobel filter');
    
    diff = A-B;
    mask = diff>0;
    %figure;  imshow(uint8(diff));  title('diff');  figure;  imshow(mask);         title('mask');
    stable_count = sum(sum(mask))  %  
    stable_procent = stable_count/numAvail %0.3141

    A = A1; B = B1;
     
    diff = A-B;
    mask = diff>0;
    %figure;  imshow(uint8(diff));  title('diff_Filter'); figure;  imshow(mask);         title('mask_Filter');
    stable_count1 = sum(sum(mask))  %  
    stable_procent1 = stable_count1/numAvail %0.3141

    improved_num = stable_count1 - stable_count
    improved_procent = stable_procent1 - stable_procent
    
    %save
    imwrite(A, outputs{i});
    imwrite(B, outputs{i+1});
  end

end

% -----vertical------- the following images need to sobel filter to sharpen edges; 
if(0)
  inputs = {strcat(dir,'0_51.bmp'), strcat(dir,'0_50.bmp'),strcat(dir,'0_49.bmp'),strcat(dir,'0_48.bmp'),strcat(dir,'0_47.bmp'),strcat(dir,'0_46.bmp'),strcat(dir,'0_45.bmp'),strcat(dir,'0_44.bmp')};
  outputs = {'0_51.bmp', '0_50.bmp','0_49.bmp', '0_48.bmp', '0_47.bmp', '0_46.bmp', '0_45.bmp', '0_44.bmp'};

  A60 = double(imread( strcat(dir,'0_60.bmp') ) );
  B61 = double(imread( strcat(dir,'0_61.bmp') ) );
  diff = A60-B61;   Maskfull = diff > 10;

  numList = size(inputs);
  numList = numList(2)
  for i=1:2:numList
    A = double(imread( inputs{i} ) );   %A = AA(:,:,i);  B = BB(:,:,i);
    B = double(imread( inputs{i+1} ) );
    
    %figure; imshow(Maskfull); title('basic mask'); figure; imshow(uint8(A)); title('full image');
    A = A.*Maskfull;  B = B.*Maskfull;
    numAvail = sum(sum(Maskfull))

    %sharpen image 
    %H = fspecial('sobel'); % better than fspecial('prewitt'); and %A1 = imsharpen(A); A1 = imadjust(A);
    %H = H;  A1 = imfilter(A,H); B1 = imfilter(B,H); 
    % sharr filter instead of sobel filter    
    Stencil= [3  0 -3; 10 0 -10; 3  0 -3];
    Stencil = Stencil';
    A1=imfilter(A,Stencil,'conv','same','replicate');
    B1=imfilter(B,Stencil,'conv','same','replicate');

    %I = A; background = imopen(I,strel('disk',100,0)); I2 = I - background; I3 = imadjust(I2); bw = imbinarize(I3); bw = bwareaopen(bw, 50); A1=bw;
    %I = B; background = imopen(I,strel('disk',100,0)); I2 = I - background; I3 = imadjust(I2); bw = imbinarize(I3); bw = bwareaopen(bw, 50); B1=bw;

    %figure; imshow(uint8(A)); title('full masked image'); figure; imshow(uint8(A1)); title('full masked image and sobel filter');
    %figure; imshow(uint8(B)); title('full masked image'); figure; imshow(uint8(B1)); title('full masked image and sobel filter');
    
    diff = A-B;
    mask = diff>0;
    %figure;  imshow(uint8(diff));  title('diff');  figure;  imshow(mask);         title('mask');
    stable_count = sum(sum(mask))  %  
    stable_procent = stable_count/numAvail %0.3141

    A = A1; B = B1;
     
    diff = A-B;
    mask = diff>0;
    %figure;  imshow(uint8(diff));  title('diff_Filter'); figure;  imshow(mask);         title('mask_Filter');
    stable_count1 = sum(sum(mask))  %  
    stable_procent1 = stable_count1/numAvail %0.3141

    improved_num = stable_count1 - stable_count
    improved_procent = stable_procent1 - stable_procent
    
    %save
    A = im2uint8(A); B = im2uint8(B);
    imwrite(uint8(A), outputs{i});
    imwrite(uint8(B), outputs{i+1});
  end

end
 
 
%%% ------C2-----------

%% 
dir = '~/slstudio/build-SLStudio-Desktop-Debug/experiment/Set1/dataCapturedForTest/';
numImg = 2050*2448;
if(1)
  inputs = {strcat(dir,'1_21.bmp'), strcat(dir,'1_20.bmp'),strcat(dir,'1_19.bmp'),strcat(dir,'1_18.bmp'),strcat(dir,'1_17.bmp'),strcat(dir,'1_16.bmp'),strcat(dir,'1_15.bmp'),strcat(dir,'1_14.bmp')};
  outputs = {'1_21.bmp', '1_20.bmp','1_19.bmp', '1_18.bmp', '1_17.bmp', '1_16.bmp', '1_15.bmp', '1_14.bmp'};
    
  A60 = double(imread( strcat(dir,'1_60.bmp') ) );
  B61 = double(imread( strcat(dir,'1_61.bmp') ) );
  diff = A60-B61;   Maskfull = diff > 10;

  numList = size(inputs);
  numList = numList(2)
  for i=1:2:numList
    A = double(imread( inputs{i} ) );   %A = AA(:,:,i);  B = BB(:,:,i);
    B = double(imread( inputs{i+1} ) );
    
    %figure; imshow(Maskfull); title('basic mask'); figure; imshow(uint8(A)); title('full image');
    A = A.*Maskfull;  B = B.*Maskfull;
    numAvail = sum(sum(Maskfull))

    %sharpen image 
    H = fspecial('sobel'); % better than fspecial('prewitt'); and %A1 = imsharpen(A); A1 = imadjust(A);
    H = H'; % for vertical direction 
    % sharr filter instead of sobel filter    
    Stencil= [3  0 -3; 10 0 -10; 3  0 -3];    
    A1=imfilter(A,Stencil,'conv','same','replicate');
    B1=imfilter(B,Stencil,'conv','same','replicate');
    %figure; imshow(uint8(A)); title('full masked image');
    figure; imshow(uint8(A1)); title('full masked image and sobel filter');
    %figure; imshow(uint8(B)); title('full masked image');
    figure; imshow(uint8(B1)); title('full masked image and sobel filter');
    
    diff = A-B;
    mask = diff>0;
    %figure;  imshow(uint8(diff));  title('diff');  figure;  imshow(mask);         title('mask');
    stable_count = sum(sum(mask))  %  
    stable_procent = stable_count/numAvail %0.3141

    A = A1; B = B1;
     
    diff = A-B;
    mask = diff>0;
    %figure;  imshow(uint8(diff));  title('diff_Filter'); figure;  imshow(mask);         title('mask_Filter');
    stable_count1 = sum(sum(mask))  %  
    stable_procent1 = stable_count1/numAvail %0.3141

    improved_num = stable_count1 - stable_count
    improved_procent = stable_procent1 - stable_procent
    
    %save
    imwrite(A, outputs{i});
    imwrite(B, outputs{i+1});
  end

end

% -----vertical------- the following images need to sobel filter to sharpen edges; 
if(1)
  inputs = {strcat(dir,'1_51.bmp'), strcat(dir,'1_50.bmp'),strcat(dir,'1_49.bmp'),strcat(dir,'1_48.bmp'),strcat(dir,'1_47.bmp'),strcat(dir,'1_46.bmp'),strcat(dir,'1_45.bmp'),strcat(dir,'1_44.bmp')};
  outputs = {'1_51.bmp', '1_50.bmp','1_49.bmp', '1_48.bmp', '1_47.bmp', '1_46.bmp', '1_45.bmp', '1_44.bmp'};
    
  A60 = double(imread( strcat(dir,'1_60.bmp') ) );
  B61 = double(imread( strcat(dir,'1_61.bmp') ) );
  diff = A60-B61;   Maskfull = diff > 10;

  numList = size(inputs);
  numList = numList(2)
  for i=1:2:numList
    A = double(imread( inputs{i} ) );   %A = AA(:,:,i);  B = BB(:,:,i);
    B = double(imread( inputs{i+1} ) );
    
    %figure; imshow(Maskfull); title('basic mask'); figure; imshow(uint8(A)); title('full image');
    A = A.*Maskfull;  B = B.*Maskfull;
    numAvail = sum(sum(Maskfull))

    %sharpen image 
    %H = fspecial('sobel'); % better than fspecial('prewitt'); and %A1 = imsharpen(A); A1 = imadjust(A);
    %H = H;  A1 = imfilter(A,H); B1 = imfilter(B,H); 
    % sharr filter instead of sobel filter    
    Stencil= [3  0 -3; 10 0 -10; 3  0 -3];
    Stencil = Stencil';
    A1=imfilter(A,Stencil,'conv','same','replicate');
    B1=imfilter(B,Stencil,'conv','same','replicate');

    %I = A; background = imopen(I,strel('disk',100,0)); I2 = I - background; I3 = imadjust(I2); bw = imbinarize(I3); bw = bwareaopen(bw, 50); A1=bw;
    %I = B; background = imopen(I,strel('disk',100,0)); I2 = I - background; I3 = imadjust(I2); bw = imbinarize(I3); bw = bwareaopen(bw, 50); B1=bw;

    %figure; imshow(uint8(A)); title('full masked image'); figure; imshow(uint8(A1)); title('full masked image and sobel filter');
    %figure; imshow(uint8(B)); title('full masked image'); figure; imshow(uint8(B1)); title('full masked image and sobel filter');
    
    diff = A-B;
    mask = diff>0;
    %figure;  imshow(uint8(diff));  title('diff');  figure;  imshow(mask);         title('mask');
    stable_count = sum(sum(mask))  %  
    stable_procent = stable_count/numAvail %0.3141

    A = A1; B = B1;
     
    diff = A-B;
    mask = diff>0;
    %figure;  imshow(uint8(diff));  title('diff_Filter'); figure;  imshow(mask);         title('mask_Filter');
    stable_count1 = sum(sum(mask))  %  
    stable_procent1 = stable_count1/numAvail %0.3141

    improved_num = stable_count1 - stable_count
    improved_procent = stable_procent1 - stable_procent
    
    %save
    A = im2uint8(A); B = im2uint8(B);
    imwrite(uint8(A), outputs{i});
    imwrite(uint8(B), outputs{i+1});
  end

end 