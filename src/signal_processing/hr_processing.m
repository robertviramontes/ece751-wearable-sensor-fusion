function [heart_rate] = hr_processing(ppg_data, mic_data)
%HR_PROCESSING Implements the DSP block to determine the heartrate
%   Takes in PPG data and MEMS microphone data and 
%   merges the processed signals to predict the heart rate.
hold off;
plot(ppg_data)
hold on;
plot(mic_data)
hold off;
% return a heart rate 
heart_rate = round(100*rand(1))+60;
end

