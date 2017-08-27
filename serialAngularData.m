pkg load instrument-control;
pkg load signal;

s1 = serial("/dev/ttyACM1", 9600);
srl_flush(s1);
datax = [];
datay = [];
dataz = [];
negativex = false;
negativey = false;
negativez = false;
hp = fir1(50,0.003,"high");

for i = 0:2000
  while srl_read(s1,1) ~= 's'
  endwhile

  if srl_read(s1,4) == ['t','a','r','t']
    %Read angular rate data (16 bit twos comp from two 1 byte packets)
    dataread = srl_read(s1,6); 
    
    %Data is in [MSByte LSByte] so e.g. dataread(1) is bits 16-9 and dataread(2) is bits 8-1
    
    %Check for negatives
    negativex = ~bitget(dataread(1),8);
    negativey = ~bitget(dataread(3),8);
    negativez = ~bitget(dataread(5),8);
    
    %Combine bytes and convert from twos comp
    if negativex
      fullbytex = -1*int16(bitshift(uint16(dataread(1)),8) + uint16(dataread(2)));
    else
      dataread(1) = dataread(1) - 128;
      fullbytex = int16(bitcmp(bitshift(uint16(dataread(1)),8) + uint16(dataread(2)),16)+1 - 32768);
    endif
    
    if negativey
      fullbytey = -1*int16(bitshift(uint16(dataread(3)),8) + uint16(dataread(4)));
    else
      dataread(3) = dataread(3) - 128;
      fullbytey = int16(bitcmp(bitshift(uint16(dataread(3)),8) + uint16(dataread(4)),16)+1 - 32768);
    endif
    
    if negativez
      fullbytez = -1*int16(bitshift(uint16(dataread(5)),8) + uint16(dataread(6)));
    else
      dataread(5) = dataread(5) - 128;
      fullbytez = int16(bitcmp(bitshift(uint16(dataread(5)),8) + uint16(dataread(6)),16)+1 - 32768);
    endif
    
    
    %Resolution of 250 dps, converting to degrees
    xd = double(fullbytex)/114.285 ;
    yd = double(fullbytey)/114.285;
    zd = double(fullbytez)/114.285;
    
    if numel(datax) > 1
      xd = xd - datax(1);
      yd = yd - datay(1);
      zd = zd - dataz(1);
    endif
    
    datax = [datax xd];
    datay = [datay yd];
    dataz = [dataz zd];

    anglex = cumtrapz(datax);
    fanglex = filter(hp,1,anglex);
    fanglex = fanglex/150;  %calibration into degrees (roughly)

    angley = cumtrapz(datay);
    fangley = filter(hp,1,angley);
    fangley = fangley/150;

    anglez = cumtrapz(dataz);
    fanglez = filter(hp,1,anglez);
    fanglez = fanglez/150;
    
    if mod(i,15) == 0  %Plot every 10 points to avoid large latency when plotting real time
      hold off; %Clear plot
      %plot(datax,'r');
      plot(fanglex,'r');
      hold on;
      %plot(datay,'g');
      plot(fangley,'g');
      hold on;
      %plot(dataz,'b');
      plot(fanglez,'b');
      hold on;
      drawnow;
    endif
  endif
endfor

hold off;
%plot(datax,'r');
plot(fanglex,'r');
hold on;
%plot(datay,'g');
plot(fangley,'g');
%plot(dataz,'b');
plot(fanglez,'b');


%Alright let's do some processing. Integrate for angle, and filter off the offset
%hp = fir1(150,1e-5,"high");

anglex = cumtrapz(datax);
fanglex = filter(hp,1,anglex);
fanglex = fanglex/150;  %calibration into degrees (roughly)

angley = cumtrapz(datay);
fangley = filter(hp,1,angley);
fangley = fangley/150;

anglez = cumtrapz(dataz);
fanglez = filter(hp,1,anglez);
fanglez = fanglez/150;

%figure;
%hold on;
%plot(fanglex,'r');
%plot(fangley,'g');
%plot(fanglez,'b');
%
%ix = cumtrapz(fanglex); %ix = integral of anglex
%iy = cumtrapz(fangley);
%iz = cumtrapz(fanglez);
%
%f_ix = filter(hp,1,ix);
%f_iy = filter(hp,1,iy);
%f_iz = filter(hp,1,iz);
%
%f_ix = f_ix/150;  %This is totally arbitrary, just being consistent
%f_iy = f_iy/150;
%f_iz = f_iz/150;
%
%%Assuming an angle of 0 degrees for x, y, and z is desired, we can make a PID loop
%%The error is simply the angle, e(t) = u
%%The input should depend reliably on something we can control
%%The process is: 
%% - Take reading
%% - Calculate error, the difference (already done when angle = 0)
%% - Calculate summation and difference values of e(t)
%% - Apply u(t) = Kp*e + Ki * cumtrapz(e) + Kd * diff(e) to the controlled device
%% This won't work here because we can't control anything (e.g. a motor)...
%% But we can see the u(t) signal.
%
%Kp = 1;
%Ki = 0.05;
%Kd = 0.05;
%
%ux = Kp*-fanglex+Ki*-f_ix+Kd*-datax;
%uy = Kp*-fangley+Ki*-f_iy+Kd*-datay;
%uz = Kp*-fanglez+Ki*-f_iz+Kd*-dataz;
%
%figure;
%hold on;
%plot(ux,'r');
%plot(uy,'g');
%plot(uz,'b');
