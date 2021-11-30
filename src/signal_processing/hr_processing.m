function [heart_rate] = hr_processing(ppg_data, mic_data)
%HR_PROCESSING Implements the DSP block to determine the heartrate
%   Takes in PPG data and MEMS microphone data and 
%   merges the processed signals to predict the heart rate.
% find peaks
[pks, locs] = findpeaks(ppg_data);

% return a heart rate 
heart_rate = 60;
end

