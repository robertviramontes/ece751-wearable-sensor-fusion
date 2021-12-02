function [heartbeat] = hr_processing(ir_vals, red_vals, sound_vals)
    ir_vals_no_dc = ir_vals - mean(ir_vals);
    red_vals_no_dc = red_vals - mean(red_vals);
    ppg = ir_vals_no_dc + red_vals_no_dc;    
    %taking fft of all values together. But they should be taken in chunks
    L = length(ppg);
    Fs = 400;
    Y = fft(ppg);
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(L/2))/L;
    % stem(f,P1)
    % figure
    
    L2 = length(sound_vals);
    Fs2 = 400;
    Y2 = fft(sound_vals);
    P22 = abs(Y2/L2);
    P12 = P22(1:L2/2+1);
    P12(2:end-1) = 2*P12(2:end-1);
    f2 = Fs2*(0:(L2/2))/L2;
    % stem(f2,P12)
    
    %finding min and max frequency index (b/w 1 and 4) for ppg
    for k = 1 : length(f)
        a = f(k);
        if a >= 1
            a = k;
            break
        end
    end
    
    for k = 1 : length(f)
        b = f(k);
        if b >= 4
            b = k;
            break
        end
    end
    
    chunk_f = f(1, a:b)
    chunk_p1 = P1(1, a:b)
    %plot(chunk_f, chunk_p1)
    [max_p1, idx_p1] = max(chunk_p1)
    hrtbeat_ppg = chunk_f(idx_p1) * 60
    
    
    %finding min and max frequency index (b/w 1 and 4) for sound
    for k = 1 : length(f2)
        a = f2(k);
        if a >= 1
            a = k;
            break
        end
    end
    
    for k = 1 : length(f2)
        b = f2(k);
        if b >= 4
            b = k;
            break
        end
    end
    
    chunk_f2 = f2(1, a:b);
    chunk_p12 = P12(1, a:b);
    %plot(chunk_f2, chunk_p12)
    [max_p12, idx_p12] = max(chunk_p12)
    hrtbeat_sound = chunk_f2(idx_p12) * 60
    
    %the previous value would be multiplied by 0.3 when running in loop
    heartbeat = 0.4*hrtbeat_ppg + 0.3*hrtbeat_sound

end
