%Example: decode_wifi_packet('WiFi_cable_X310_3123D64_#1__run1.mat','TransmitBit_1000pckt_July13.mat',true,true,true, false, 1, true, 1000)

function [BER, estimatedChannel] = decode_wifi_packet(filename, txBit_filename, showConstellation, showSpectrum, displayFlag, isLongPacket, channelInfo_filename, bandwidth, numPackets, isSaveChannelState, num_packets_tosave)
    
%             load(filename,'txWaveform')
%      rxWaveform=txWaveform;
%     
    load(filename,'wifi_rx_data')
    rxWaveform=wifi_rx_data;
    load(txBit_filename,'transmitBits'); % Load transmitted bits
    %numPackets=1; %long packet (1000 packets)
    %num_packets_tosave =100
    BER=-2*ones(num_packets_tosave,1);
    error_number=[];
    
    %estimatedChannel = zeros(num_packets_tosave,52);
    estimatedChannel = [];
    demodulated_sym = [];
    % Setup the constellation diagram viewer for equalized WLAN symbols
%     constellation = comm.ConstellationDiagram('Title','Equalized WLAN Symbols',...
%                                     'ShowReferenceConstellation',false);

    nonHTcfg = wlanNonHTConfig;
    nonHTcfg.ChannelBandwidth=bandwidth; 
    nonHTcfg.PSDULength = 1024;
    fs = wlanSampleRate(nonHTcfg);       % Sampling rate
    chanBW=bandwidth;
    % Get the required field indices within a PSDU
    indLSTF = wlanFieldIndices(nonHTcfg,'L-STF');
    indLLTF = wlanFieldIndices(nonHTcfg,'L-LTF');
    indLSIG = wlanFieldIndices(nonHTcfg,'L-SIG');
    Ns = indLSIG(2)-indLSIG(1)+1; % Number of samples in an OFDM symbol


    % Setup Spectrum viewer
%     if(showSpectrum)
%         saScope = dsp.SpectrumAnalyzer('SampleRate',fs,'YLimits',[-120 10]);
%         saScope(rxWaveform)
%         pause(1);
%     end

    
    %rxWaveform = resample(burstCaptures,fs,fs*osf);
    rxWaveformLen = size(rxWaveform,1);
    searchOffset = 0; % Offset from start of the waveform in samples

    % Minimum packet length is 10 OFDM symbols
    lstfLen = double(indLSTF(2)); % Number of samples in L-STF
    minPktLen = lstfLen*5;
    pktInd = 1;
    sr = wlanSampleRate(nonHTcfg); % Sampling rate
    fineTimingOffset = [];
    packetSeq = [];
    %displayFlag = 0; % Flag to display the decoded information
    count=1;
    isFirstValidPacket=true;
    % Receiver processing
    
    while (searchOffset + minPktLen) <= rxWaveformLen
        try
        
        
            % Packet detect

            pktOffset = wlanPacketDetect(rxWaveform, chanBW, searchOffset, 0.8);

            % Adjust packet offset
            pktOffset = searchOffset+pktOffset;
            if isempty(pktOffset) || (pktOffset+double(indLSIG(2))>rxWaveformLen)
                if pktInd==1
                    disp('** No packet detected **');
                end
                break;
            end

             % Extract non-HT fields and perform coarse frequency offset correction
            % to allow for reliable symbol timing
            nonHT = rxWaveform(pktOffset+(indLSTF(1):indLSIG(2)),:);
            coarseFreqOffset = wlanCoarseCFOEstimate(nonHT,chanBW);
            nonHT = helperFrequencyOffset(nonHT,fs,-coarseFreqOffset);

            % Symbol timing synchronization
            fineTimingOffset = wlanSymbolTimingEstimate(nonHT,chanBW);

            % Adjust packet offset
            pktOffset = pktOffset+fineTimingOffset;

            % Timing synchronization complete: Packet detected and synchronized
            % Extract the NonHT preamble field after synchronization and
            % perform frequency correction
            if (pktOffset<0) || ((pktOffset+minPktLen)>rxWaveformLen)
                searchOffset = pktOffset+1.5*lstfLen;
                continue;
            end
            
            if(displayFlag)
                fprintf('\nPacket-%d detected at index %d\n',pktInd,pktOffset+1);
            end

            % Extract first 7 OFDM symbols worth of data for format detection and
            % L-SIG decoding
            nonHT = rxWaveform(pktOffset+(1:7*Ns),:);
            nonHT = helperFrequencyOffset(nonHT,fs,-coarseFreqOffset);

            % Perform fine frequency offset correction on the synchronized and
            % coarse corrected preamble fields
            lltf = nonHT(indLLTF(1):indLLTF(2),:);           % Extract L-LTF
            fineFreqOffset = wlanFineCFOEstimate(lltf,chanBW);
            nonHT = helperFrequencyOffset(nonHT,fs,-fineFreqOffset);
            cfoCorrection = coarseFreqOffset+fineFreqOffset; % Total CFO

            % Channel estimation using L-LTF
            lltf = nonHT(indLLTF(1):indLLTF(2),:);
            demodLLTF = wlanLLTFDemodulate(lltf,chanBW);
            chanEstLLTF = wlanLLTFChannelEstimate(demodLLTF,chanBW);
            
            
            
            
           
            %

            % Noise estimation
            noiseVarNonHT = helperNoiseEstimate(demodLLTF);

            % Packet format detection using the 3 OFDM symbols immediately
            % following the L-LTF
            format = wlanFormatDetect(nonHT(indLLTF(2)+(1:3*Ns),:), ...
                chanEstLLTF,noiseVarNonHT,chanBW);
            if(displayFlag)
                disp(['  ' format ' format detected']);
            end
            if ~strcmp(format,'Non-HT')
                if(displayFlag)
                    fprintf('  A format other than Non-HT has been detected\n');
                end
                searchOffset = pktOffset+1.5*lstfLen;
                continue;
            end

            % Recover L-SIG field bits
            [recLSIGBits,failCheck] = wlanLSIGRecover( ...
                   nonHT(indLSIG(1):indLSIG(2),:), ...
                   chanEstLLTF,noiseVarNonHT,chanBW);

            if failCheck
                if(displayFlag)
                    fprintf('  L-SIG check fail \n');
                end
                searchOffset = pktOffset+1.5*lstfLen;
                continue;
            else
                if(displayFlag)
                    fprintf('  L-SIG check pass \n');
                end
            end

            % Retrieve packet parameters based on decoded L-SIG
            [lsigMCS,lsigLen,rxSamples] = helperInterpretLSIG(recLSIGBits,sr);

            if (rxSamples+pktOffset)>length(rxWaveform)
                if(displayFlag)
                    disp('** Not enough samples to decode packet **');
                end
                break;
            end

            % Apply CFO correction to the entire packet
            rxWaveform(pktOffset+(1:rxSamples),:) = helperFrequencyOffset(...
                rxWaveform(pktOffset+(1:rxSamples),:),fs,-cfoCorrection);

            % Create a receive Non-HT config object
             rxNonHTcfg = wlanNonHTConfig;
             rxNonHTcfg.ChannelBandwidth=bandwidth;
             rxNonHTcfg.MCS = lsigMCS;
             rxNonHTcfg.PSDULength = lsigLen;

            % Get the data field indices within a PPDU
            indNonHTData = wlanFieldIndices(rxNonHTcfg,'NonHT-Data');

            % Recover PSDU bits using transmitted packet parameters and channel
            % estimates from L-LTF
            rxPSDU=[];
            [rxPSDU,eqSym] = wlanNonHTDataRecover(rxWaveform(pktOffset+...
                   (indNonHTData(1):indNonHTData(2)),:), ...
                   chanEstLLTF,noiseVarNonHT,rxNonHTcfg);
            
            demodulated_sym = [demodulated_sym; reshape(eqSym,[],1)]; 
            
            if (~isLongPacket) 
                if length(rxPSDU) == length(transmitBits)
                    [error_number(pktInd,:), ~]=biterr(rxPSDU,transmitBits);
                else
                    error_number(pktInd,:) = length(transmitBits);
                end
                
                estimatedChannel(pktInd,:) =  chanEstLLTF;
            
            elseif (isLongPacket)

                if (length(rxPSDU)==8192)
                    if(isFirstValidPacket)
                        %valid_pktInd=1;
                        isFirstValidPacket=false;
                        count=find_packet_location(txBit_filename, rxPSDU, numPackets);                    
                    end
                    %count

                    if (count>1000)
                        count=1;
                    end
                    [error_number(pktInd), ~]=biterr(rxPSDU,transmitBits(8192*(count-1)+1:8192*count));
                    count=count+1;
                    %valid_pktInd=valid_pktInd+1;                
                else
                    %count=count+1;
                    error_number(pktInd)=8192;
                end
                
            end
            
            
            

%             if(showConstellation)   
%                 constellation(reshape(eqSym,[],1)); % Current constellation
%                 pause(1); % Allow constellation to repaint
%                 release(constellation); % Release previous constellation plot
%             end

            displayExtraFlag=false;
            % Display decoded information
            if displayExtraFlag
                fprintf('  Estimated CFO: %5.1f Hz\n\n',cfoCorrection); %#ok<UNRCH>

                disp('  Decoded L-SIG contents: ');
                fprintf('                            MCS: %d\n',lsigMCS);
                fprintf('                         Length: %d\n',lsigLen);
                fprintf('    Number of samples in packet: %d\n\n',rxSamples);

    %             fprintf('  EVM:\n');
    %             fprintf('    EVM peak: %0.3f%%  EVM RMS: %0.3f%%\n\n', ...
    %             evm.Peak,evm.RMS);
    % 
    %             fprintf('  Decoded MAC Sequence Control field contents:\n');
    %             fprintf('    Sequence number:%d\n',packetSeq(pktInd));

            end
            
        
                   % Update search index
            searchOffset = pktOffset+double(indNonHTData(2));
            pktInd = pktInd+1;
            
            %if (pktInd >=1000)
                %pause;
                %break
            % Finish processing when a duplicate packet is detected. The
            % recovered data includes bits from duplicate frame
            if length(unique(packetSeq))<length(packetSeq)
                break
            end  
          %rethrow(ME)
            %channel_filename =strcat(channelInfo_filename, '_pktInd_', num2str(pktInd));
            if(isSaveChannelState ==true)         
                save(channelInfo_filename,'estimatedChannel');
            end
            
            %save only one packet
            if pktInd>num_packets_tosave
                break;
            end
            
        catch  ME
            fprintf(1,'The identifier was:\n%s',ME.identifier);
            fprintf(1,'There was an error! The message was:\n%s',ME.message); 
               if (strcmp(ME.identifier,'MATLAB:catenate:dimensionMismatch'))
                  msg = ['Dimension mismatch occurred: First argument has ', ...
                        num2str(size(A,2)),' columns while second has ', ...
                        num2str(size(B,2)),' columns.'];
                    causeException = MException('MATLAB:myCode:dimensions',msg);
                    ME = addCause(ME,causeException);
               end
          break;     
        end
    end
    

    
    
    %fname = strrep(filename,'WiFi','Demod_WiFi');
    %fname= strcat('Demod_', filename);
    %save(fname, 'demodulated_sym')
    
    if(~isempty(error_number))
        %display('BER')
        %BER=sum(error_number)/(length(error_number)*8192);
%         if (length(error_number) < num_packets_tosave)            
%             %BER = [error_number/8192; ones(num_packets_tosave-length(error_number),1)];
%             BER = [error_number/8192];
%         else
            BER = error_number/8192;
        %end
    else
        if(displayFlag)
            disp('Packets are undecodable');
        end
%         BER=-1*ones(num_packets_tosave,1);
%         estimatedChannel = zeros(num_packets_tosave,52);
        BER=-1;
        estimatedChannel = zeros(1,52);
    end   
    
    
end
