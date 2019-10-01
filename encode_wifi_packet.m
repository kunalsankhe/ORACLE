function encode_wifi_packet(numPackets, tx_bit_filename, wifi_packet_filename, MCS, bandwidth)
    nonHTcfg = wlanNonHTConfig;
    nonHTcfg.NumTransmitAntennas = 1;
    nonHTcfg.ChannelBandwidth=bandwidth;
    nonHTcfg.MCS = MCS;
    nonHTcfg.PSDULength = 1024;
    %chanBW='CBW20';
    fs = wlanSampleRate(nonHTcfg); 
    idleInterval = 0.01e-3;% Sampling rate
    %idleInterval = 0;% Sampling rate
    
    lstf = wlanLSTF(nonHTcfg);
    lltf = wlanLLTF(nonHTcfg);
    lsig = wlanLSIG(nonHTcfg);
    %preamble = [lstf;lltf;lsig];
    
    rng(0) % Initialize the random number generator    
    txWaveform = [];
    transmitBits=[];
    for k=1:numPackets
        bit_length = nonHTcfg.PSDULength*8
        txPSDU = randi([0 1],nonHTcfg.PSDULength*8,1); % Generate PSDU data in bits
        beaconCfg = wlanMACFrameConfig('FrameType', 'Beacon');
        disp(beaconCfg);
        % Create a management frame-body configuration object
        frameBodyCfg = wlanMACManagementConfig;
        disp(frameBodyCfg);
        % Beacon Interval
        frameBodyCfg.BeaconInterval = 0;
        % SSID
        frameBodyCfg.SSID = 'TEST_BEACON';
        % Update management frame-body configuration
        beaconCfg.ManagementConfig = frameBodyCfg;
        
        % Generate beacon frame
        beaconFrame = wlanMACFrame(beaconCfg);
        
        %helperWLANExportToPCAP({ beaconFrame}, 'macFrames.pcap');
        
        % Convert the PSDU in hexadecimal format to bits
        decimalBytes = hex2dec(beaconFrame);
        bitsPerByte = 8;                % Number of bits in 1 byte
        psduBits = reshape(de2bi(decimalBytes, bitsPerByte)', [], 1);
        %txPSDU = psduBits        
        txPSDU(bit_length-length(psduBits)+1:bit_length,1)= psduBits;
        transmitBits=[transmitBits;txPSDU(1:bit_length-length(psduBits)); psduBits ];        
        %data = wlanNonHTData(txPSDU,nonHTcfg);
        data = wlanWaveformGenerator(txPSDU, nonHTcfg, 'IdleTime', idleInterval);
        %txWaveform = [txWaveform; preamble; data];
        txWaveform = [txWaveform; data];
    end
    save(tx_bit_filename,'transmitBits');
    save(wifi_packet_filename, 'txWaveform');
end