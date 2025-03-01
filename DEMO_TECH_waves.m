%% Real-Time Audio Signal Visualization
fs = 44100;
frameSize = 1024;

deviceReader = audioDeviceReader('SampleRate', fs, 'SamplesPerFrame', frameSize);
deviceWriter = audioDeviceWriter('SampleRate', fs);

scope = dsp.TimeScope('SampleRate', fs, ...
                      'TimeSpan', 0.1, ... 
                      'BufferLength', 10 * fs, ...
                      'YLimits', [-1 1], ... %
                      'Title', 'Input, Inverted, and Combined Signals', ...
                      'NumInputPorts', 3, ...
                      'ShowGrid', true, ...
                      'ChannelNames', {'Input Signal', 'Inverted Signal', 'Combined Signal'});

disp('Processing... Press Ctrl+C to stop.');

try
    while true
        % Read audio data from the microphone
        inputSignal = deviceReader();
        
        % Generate the inverted signal
        invertedSignal = -inputSignal;
        
        % Create the combined signal (input + inverted)
        combinedSignal = inputSignal + invertedSignal;
        
        % Visualize all three signals in the time scope
        scope(inputSignal, invertedSignal, combinedSignal);
        
        % Output the combined signal to the speakers
        deviceWriter(combinedSignal);
    end
catch
    release(deviceReader);
    release(deviceWriter);
    release(scope);
    disp('Processing stopped.');
end