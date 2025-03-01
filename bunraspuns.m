%% Preset Audio Signal for processing
Fs = 1000;
ts = 1/Fs;
t = 0:ts:1;
u = 0.1;
disp(t);
x = sin(8*pi*t);
N = size(x);

xn = x + 0.1*randn(1,N(2)); % corrupted by noise

figure(1) ; plot(t, x); legend('x');
figure(2) ; plot(t, xn);legend('xn');

wn = zeros(1,N(2)); %filter coefficients
hn = (1/N(2))*ones(1,N(2)); %impulse response
%dn = sin(8*pi*t);
dn = randn(1,2*N(2)-1); % random reference signal
yn = zeros(1,2*N(2)-1); %filtered output
en = zeros(); % error signal

% updates the coefficient using the LMS error algorithm
for I = 1:N

    d1 = size(xn);
    n1 = d1(2);
    d2 = size(wn);
    n2 = d2(2);
    n = n1+n2-1;
    xn = [xn, zeros(1,n-n1)];
    wn = [wn, zeros(1,n-n2)];
    % computing the output yn by convoluting xn and wn
    for i=1:n
        for j = 1:i
            yn(i) = yn(i) + xn(j)*wn(i-j+1) ;
        end
    end
 % computes the error signal as a difference betwen the reference signal dn
 % and filtered output yn
    en = dn - yn;
    
    % updates the filter coefficients using the LMS rule
    for i=1:n
        wn(i+1) = wn(i)+u*en(i)*xn(i);
    end
end

% filter coefficients
figure(3);
plot(wn,'-');
legend('wn');
xlabel('nth wt');

 %error signal
figure(4);
plot(en,'-');legend('en');
xlabel('nth wt');


% Set simulation duration (normalized) 
clear
T=1000; 

% We do not know P(z) and S(z) in reality. So we have to make dummy paths
Pw=[0.01 0.25 0.5 1 0.5 0.25 0.01];
Sw=Pw*0.25;

% Remember that the first task is to estimate S(z). So, we can generate a
% white noise signal,
x_iden=randn(1,T);

% send it to the actuator, and measure it at the sensor position, 
y_iden=filter(Sw, 1, x_iden);

% Then, start the identification process
Shx=zeros(1,16);     % the state of Sh(z)
Shw=zeros(1,16);     % the weight of Sh(z)
e_iden=zeros(1,T);   % data buffer for the identification error

% and apply least mean square algorithm
mu=0.1;                         % learning rate
for k=1:T               % discrete time k
    Shx=[x_iden(k) Shx(1:15)];  % update the state
    Shy=sum(Shx.*Shw);	        % calculate output of Sh(z)
    e_iden(k)=y_iden(k)-Shy;    % calculate error         
    Shw=Shw+mu*e_iden(k)*Shx;   % adjust the weight
end

% Lets check the result
% subplot(2,1,1)
subplot(2,1,1)
plot([1:T], e_iden)
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Identification error');
subplot(2,1,2)
stem(Sw) 
hold on 
stem(Shw, 'r*')
ylabel('Amplitude');
xlabel('Numbering of filter tap');
legend('Coefficients of S(z)', 'Coefficients of Sh(z)')


% The second task is the active control itself. Again, we need to simulate 
% the actual condition. In practice, it should be an iterative process of
% 'measure', 'control', and 'adjust'; sample by sample. Now, let's generate 
% the noise: 
% X=randn(1,T);
X=randn(1,T);
% and measure the arriving noise at the sensor position,
Yd=filter(Pw, 1, X);
  
% Initiate the system,
Cx=zeros(1,16);       % the state of C(z)
Cw=zeros(1,16);       % the weight of C(z)
Sx=zeros(size(Sw));   % the dummy state for the secondary path
e_cont=zeros(1,T);    % data buffer for the control error
Xhx=zeros(1,16);      % the state of the filtered x(k)

% and apply the FxLMS algorithm
mu=0.1;                            % learning rate
for k=1:T,                         % discrete time k
    Cx=[X(k) Cx(1:15)];            % update the controller state    
    Cy=sum(Cx.*Cw);                % calculate the controller output	
    Sx=[Cy Sx(1:length(Sx)-1)];    % propagate to secondary path
    e_cont(k)=Yd(k)-sum(Sx.*Sw);   % measure the residue
    Shx=[X(k) Shx(1:15)];          % update the state of Sh(z)
    Xhx=[sum(Shx.*Shw) Xhx(1:15)]; % calculate the filtered x(k)
    Cw=Cw+mu*e_cont(k)*Xhx;        % adjust the controller weight
end

% Report the result
figure
subplot(2,1,1)
plot([1:T], e_cont)
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Noise residue')
subplot(2,1,2)
plot([1:T], Yd) 
hold on 
plot([1:T], Yd-e_cont, 'r:')
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Noise signal', 'Control signal')


Y= Xhx+Shx;
plot(Y)
%%
% Plot the signals to visualize noise cancellation
figure;

% Plot the input noise signal (Y_d)
subplot(3,1,1);
plot([1:T], Yd, 'b');
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Input Noise (Y_d)');
title('Input Noise Signal');

% Plot the generated canceling signal (Y_s)
Ys = filter(Sw, 1, Cx); % Generate the canceling signal
subplot(3,1,2);
plot([1:T], Ys, 'r');
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Generated Canceling Signal (Y_s)');
title('Generated Signal');
%% something else

%% Parameters
Fs = 1000; % Sampling frequency
ts = 1 / Fs; % Sampling period
t = 0:ts:1-ts; % Time vector for 1 second
u = 0.1; % Learning rate for LMS
x = sin(8 * pi * t); % Desired clean sinusoidal signal
N = length(x);

% Generate noisy input signal
xn = x + 0.1 * randn(1, N); % Sinusoidal signal corrupted by noise

% Visualization: Original and Noisy Signal
figure;
subplot(3,1,1);
plot(t, x, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Amplitude');
title('Original Signal');
legend('x');

subplot(3,1,2);
plot(t, xn, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Amplitude');
title('Noisy Signal');
legend('xn');

% LMS Algorithm to Compute the Canceling Signal
filterOrder = 32; % Number of filter taps
wn = zeros(1, filterOrder); % Adaptive filter coefficients
yn = zeros(1, N); % Output of the filter (approximating noise)
en = zeros(1, N); % Error signal (residual noise)
cancelingSignal = zeros(1, N); % The canceling signal

for k = filterOrder:N
    % Extract a frame of the noisy input signal
    xFrame = xn(k:-1:k-filterOrder+1); % Current frame of length equal to filter order
    
    % Compute the filter output (approximated noise)
    yn(k) = sum(wn .* xFrame); % Adaptive filter output
    
    % Compute the error signal (difference between input and desired)
    en(k) = x(k) - yn(k); % Error signal measures residual noise
    
    % Compute the canceling signal (inverse of filter output)
    cancelingSignal(k) = -yn(k); % Inverted signal to cancel the input noise
    
    % Update the filter coefficients using the LMS rule
    wn = wn + u * en(k) * xFrame; % LMS weight update
end

% Combine the Input Signal and Canceling Signal
combinedSignal = xn + cancelingSignal; % Ideally, this should approach the original signal

% Visualization: Canceling and Combined Signals
subplot(3,1,3);
plot(t, combinedSignal, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Amplitude');
title('Combined Signal (Input + Canceling Signal)');
legend('Combined Signal');

figure;
% Canceling Signal
subplot(2,1,1);
plot(t, cancelingSignal, 'm', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Amplitude');
title('Canceling Signal');
legend('Canceling Signal');

% Residual Noise
subplot(2,1,2);
plot(t, en, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Amplitude');
title('Residual Noise');
legend('Error Signal (en)');


% Plot the combined signal (Y_d + Y_s)
subplot(3,1,3);
combinedSignal = Yd + Ys; % Add the noise and canceling signals
plot([1:T], combinedSignal, 'g');
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Combined Signal (Y_d + Y_s)');
title('Combined Signal (Noise Canceled)');

