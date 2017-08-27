pkg load instrument-control;
s1 = serial("/dev/ttyACM1", 9600);
srl_flush(s1);
datax = [];
datay = [];
dataz = [];
negativex = false;
negativey = false;
negativez = false;

for i = 0:5000

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
    xd = double(fullbytex)/114.285;
    yd = double(fullbytey)/114.285;
    zd = double(fullbytez)/114.285;
    
    datax = [datax xd];
    datay = [datay yd];
    dataz = [dataz zd];
    
    if mod(i,10) == 0  %Plot every 10 points to avoid large latency when plotting real time
      hold off; %Clear plot
      plot(datax,'r');
      hold on;
      plot(datay,'g');
      hold on;
      plot(dataz,'b');
      hold on;
      drawnow;
    endif
  endif
endfor

hold off;
plot(datax,'r');
hold on;
plot(datay,'g');
hold on;
plot(dataz,'b');
hold on;
