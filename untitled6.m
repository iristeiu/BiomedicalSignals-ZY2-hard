clear;
% close all;
clc;
 
%[t,rawSig1] = importfile('sample1.csv');
 
%rawSig1 = 10 * (rawSig1 - mean(rawSig1));   % artificial removal of the DC component
 
%[t,rawSig2] = importfile1('sample2.csv');
 
%rawSig2 = 10 * (rawSig2 - mean(rawSig2));   % artificial removal of the DC component
 
[t,rawSig3] = importfile2('sample3.csv');
 
rawSig3 = 10 * (rawSig3 - mean(rawSig3));   % artificial removal of the DC component
 
t = t + abs(t(1));
fs = length(t)/t(end);
 
figure(1);
%subplot(3,1,1);
%plot(t,rawSig1);
%ylabel('Raw signal1');

%subplot(3,1,2);
%plot(t,rawSig2);
%ylabel('Raw signal2');

subplot(5,1,1);
plot(t,rawSig3);
ylabel('Raw signal3');

% hihpass filter
d = fdesign.highpass('n,fc', 4, 0.3, fs);
hh = design(d,'butter');
hpSig = filter(hh,rawSig3);
 
subplot(5,1,2);
plot(t,hpSig);
ylabel('Highpassed signal');

% lowpass filter
d = fdesign.lowpass('n,fc', 4, 60, fs);
hl = design(d,'butter');
lpSig = filter(hl,hpSig);
 
subplot(5,1,3);
plot(t,lpSig);
ylabel('Lowpassed signal');

amplification_factor = 1.5;
 % You can adjust this factor according to your requirement
% Amplify the ECG signal
amplified_ecg_lp = amplification_factor * lpSig;
subplot(5,1,4);
plot(t,amplified_ecg_lp);
ylabel('Amplified Sig LP');

% notch filter
d = fdesign.notch('n,f0,bw', 4, 30, 20, fs);
hn = design(d,'butter');
notchSig = filter(hn,amplified_ecg_lp);
 
subplot(5,1,5);
plot(t,notchSig);
ylabel('After notch');
xlabel('Time');
sig = notchSig;
[pksR, locsR, wR, pR] = findpeaks(sig, 'MinPeakHeight', max(sig) / 2);
 
figure(2);
plot(t,sig);
hold on;
plot(locsR/fs, pksR, '^r');
hold off;
 
[pksS, locsS, wS, pS] = findpeaks(-sig, 'MinPeakHeight', max(-sig) / 2);
 
hold on;
plot(locsS/fs, -pksS, 'sg');
hold off;

sampleSig = sig(locsR(1) : locsR(end));    % artificial supression of the seignal segments before first peak and after last peak
[c,lag] = xcorr(sampleSig);
 
figure(3);
subplot(2,1,1);
plot(sampleSig);
subplot(2,1,2);
plot(lag,c);

 
% heart rate detection
 
rr_interval = diff(locsR) / fs;
instantaneous_hr = 60./ rr_interval;
 
hr = mean(instantaneous_hr);
hrv = std(instantaneous_hr);

% SIMPLE pacemaker model
 
atrial_pacing_delay = 50e-3;
ventricular_pacing_delay = 100e-3;
pulseLen = 25e-3;
bradycardiaThreshold = 500e-3;
 
atrial_pacing_pulses = zeros(size(sig));
ventricular_pacing_pulses = zeros(size(sig));
 
for i = 1 : length(rr_interval)
    startIdx = locsR(1) + sum(rr_interval(1:i-1) * fs);
    if rr_interval(i) > bradycardiaThreshold
        atrial_pacing_pulses(startIdx + bradycardiaThreshold * fs + atrial_pacing_delay * fs : startIdx + bradycardiaThreshold * fs + atrial_pacing_delay * fs + pulseLen) = 1;
        ventricular_pacing_pulses(startIdx + bradycardiaThreshold * fs + ventricular_pacing_delay * fs : startIdx + bradycardiaThreshold * fs + ventricular_pacing_delay * fs + pulseLen) = 1;
    end
end
 
figure(4);
subplot(2,1,1);
plot(t,sig);
subplot(2,1,2);
plot(t,atrial_pacing_pulses, 'r','LineWidth',2);
hold on;
plot(t,ventricular_pacing_pulses, 'g','LineWidth',2);
hold off;