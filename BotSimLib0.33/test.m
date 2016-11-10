tic
    endloop=26;
    for i=1:endloop % defines height values for next loop
        if i<(endloop/2 + 2)
            f(i)=3+i;
        else
            f(i)=f(i-1)-1;
        end 
    end
    [x,y]=meshgrid(-256:255,-256:255); % create grid
    z=sqrt(x.^2+y.^2); % define circle
    for k=1:endloop
        c=(z<f(k));
        cf=fftshift(fft2(c)); % apply transform
            fl = log(1+abs(cf)); fm = max(fl(:));
            imshow(im2uint8(fl/fm)); 
        frame=getframe; % close;
        im=frame2im(frame);
        [imind,map]=rgb2ind(im,256);
        if k==1
            imwrite(imind,map,'circle3.gif','DelayTime',0.1,'LoopCount',inf);
        else
            imwrite(imind,map,'circle3.gif', 'DelayTime',0.1, 'WriteMode', 'append');
        end
    end
    close all
    toc