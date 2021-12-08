function [heartbeat] = wavelet_peaks( ...
        vals, Fs)

% ppg_x = linspace(0, vals_length*(1/Fs), vals_length);

% ppg_sum = ir_vals + red_vals;

[cA,~] = dwt(vals,'sym4');
xrec = idwt(cA,zeros(size(cA)),'sym4'); 

[~, locs] = findpeaks(xrec, Fs, 'MinPeakDistance', 0.3, 'MinPeakWidth', 0.1);
diff_locs = diff(locs); % difference in s
bpms = 60 * 1./diff_locs; % convert to bpm
heartbeat = mean(bpms);

end

