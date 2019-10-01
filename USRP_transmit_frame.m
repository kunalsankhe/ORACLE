%sample to run the function

%For short packet run below
%USRP_tx_send_data(900e6,1, 70, 'WiFi_80211a', false, 100); 

% For long packet run below 
%USRP_tx_send_data(900e6,1, 70, 'WiFi_80211a_long', true, 30); 


% Select tx_gain 70 for B210, 72 for B210, 0 for X310, 8 for N210. 

function USRP_transmit_frame(center_frequency, channel_mapping, tx_gain, masterClockRate, interpolationFactor, WiFi_packet_filename, isLongPacket, tx_timecount)
    radioFound = false;
    radiolist = findsdru;
    for i = 1:length(radiolist)
      if strcmp(radiolist(i).Status, 'Success')
        if (strcmp(radiolist(i).Platform, 'B210'))
            radio = comm.SDRuTransmitter('Platform','B210', ...
                     'SerialNum', radiolist(i).SerialNum);
            radio.MasterClockRate = masterClockRate; %20e6;
            radio.InterpolationFactor = interpolationFactor; %1;     
            radioFound = true;
            break;
        end
        if (strcmp(radiolist(i).Platform, 'B200'))
            radio = comm.SDRuTransmitter('Platform','B200', ...
                     'SerialNum', radiolist(i).SerialNum);
            radio.MasterClockRate = masterClockRate; %20e6; 
            radio.InterpolationFactor = interpolationFactor; %4;     
            radioFound = true;
            break;
        end
        if (strcmp(radiolist(i).Platform, 'X300') || ...
            strcmp(radiolist(i).Platform, 'X310'))
            radio = comm.SDRuTransmitter('Platform',radiolist(i).Platform, ...
                     'IPAddress', radiolist(i).IPAddress);
            radio.MasterClockRate = masterClockRate; %120e6;
            radio.InterpolationFactor = interpolationFactor; %24;    
            radioFound = true;
            break;
        end
        if (strcmp(radiolist(i).Platform, 'N200/N210/USRP2'))
            radio = comm.SDRuTransmitter('Platform',radiolist(i).Platform, ...
                     'IPAddress', radiolist(i).IPAddress);
            %radio.MasterClockRate = 120e6;
            radio.InterpolationFactor = interpolationFactor; %20    
            radioFound = true;
            break;
        end
      end
    end
    
    if ~radioFound
        error(message('sdru:examples:NeedMIMORadio'));
    end
    
    radio.ChannelMapping = channel_mapping;
    radio.CenterFrequency = center_frequency; % For B210
    radio.Gain =tx_gain;
    radio.UnderrunOutputPort = true;
    radio.EnableBurstMode=false;
    %radio.NumFramesInBurst = 1;
    %radio;
    
    load(WiFi_packet_filename, 'txWaveform');
    %txWaveform = xx.txWaveform;
    % Scale the normalized signal to avoid saturation of RF stages
    powerScaleFactor = 0.8;
    txWaveform = txWaveform.*(1/max(abs(txWaveform))*powerScaleFactor);
    
    zero_pad_length = ceil(length(txWaveform)/375000)*375000 - length(txWaveform);
    
    txWaveform = [txWaveform; complex(zeros(zero_pad_length,1))];
    
    
    
    % Cast the transmit signal to int16, this is the native format for the SDR
    % hardware
    txWaveform = int16(txWaveform*2^15);
    
    %disp('Press enter to start transmission');
    %pause;

    disp('started transmission');
    
    if(isLongPacket)    
        for j=1:tx_timecount    
            for i=1:375000:length(txWaveform)
                if(i+374999 > length(txWaveform))
                    last_i = length(txWaveform) - i;                    
                else
                    last_i = i+374999;                    
                end
                bufferUnderflow = step(radio,txWaveform(i:last_i));
                if bufferUnderflow~=0
                    warning('sdru:examples:DroppedSamples','Dropped samples')
                end
            end
        end
    else
        for j=1:tx_timecount
            for i=1:100        
                bufferUnderflow = step(radio,txWaveform);        
                if bufferUnderflow~=0
                    warning('sdru:examples:DroppedSamples','Dropped samples')
                end                 
            end
        end   
    end
    
end
