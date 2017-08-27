pkg load instrument-control;
pkg load signal;

s1 = serial("/dev/ttyACM0");
set(s1,'baudrate',115200);
set(s1, 'bytesize',8);
set(s1,'parity','n');
set(s1,'stopbits',1);
set(s1,'timeout',100); %10.0 seconds

sleep(0.5);
srl_flush(s1);

datax = [];
datay = [];
dataz = [];
negativex = false;
negativey = false;
negativez = false;
hp = fir1(150,0.01,"high");
xd = 0;
yd = 0;
zd = 0;
delaytime = 1000; %delay time in microseconds (us)

hold on;
for i = 1:3000
  srl_write(s1,"x");
  xd = str2num(char(srl_read(s1,6)));
  srl_write(s1,"s");
  usleep(delaytime);

  srl_write(s1,"y");
  yd = str2num(char(srl_read(s1,6)));
  srl_write(s1,"s");
  usleep(delaytime);
  
  srl_write(s1,"z");
  zd = str2num(char(srl_read(s1,6)));
  srl_write(s1,"s");
  usleep(delaytime);
  
  datax = [datax xd/131.068-1.4951]; %Convert from 16 bit signed (max 32767) to 250 deg/s 
  datay = [datay yd/131.068+0.96593];  %Also add/subtract the measured mean offset 
  dataz = [dataz zd/131.068+0.1717];
  
  anglex = cumtrapz(datax)/32.55;
  angley = cumtrapz(datay)/32.55;
  anglez = cumtrapz(dataz)/32.55;
  
  if mod(i, 20) == 0
    hold off;
    plot(anglex,'r');
    hold on;
    plot(angley,'b');
    plot(anglez,'g');
    drawnow;
  endif
endfor
%
%%printf(" %s \n %s \n %s \n", datax, datay, dataz);
