clc
clear
close all;

Data = readmatrix("Speed_Input.csv");
time = Data(:,1);
speed = Data(:,2);
max(speed);
min(speed);
%Norm_speed = (speed-min(speed))./(max(speed)-min(speed));

%%
% Param PSO
c=2; w=0.3; particles=5; iteration=10; Var=2;

% search space
lb = [0, 1];
ub = [2, 6];

% Weighting factors for the objectives
W_mse = 0.7;  % Weight for MSE (speed tracking performance)
W_energy = 0.3;  % Weight for energy consumption

% optimization steps
c_cf=0;

% Initialization

% Initialization of Energy
initial_energy = zeros(1, particles);

for m=1:particles
    for n=1:Var
        v(m,n)=0;
        x(m,n)=lb(n) + rand(1) * (ub(n) - lb(n));
        xp(m,n)=x(m,n);
    end
    %Model param
    kp=x(m,1);
    ki=x(m,2);

    sim('PID_MO.slx');

    error1 = ans.ScopeData1(:,3) - ans.ScopeData1(:,2);


    % MSE (objective function)
    ff1=0;
    sim1=size(ans.ScopeData1(:,2),1);
    for m1=1:sim1
        ff1 = ff1 + (error1(m1)^2); % Squared error
    end
    MSE(m) = ff1/sim1; % Calculate mean by dividing by the number of samples

    % Calculate energy consumption
    power = abs(ans.ScopeData11(:,2));  % Instantaneous power (assuming power is non-negative)
    energy(m) = trapz(ans.ScopeData11(:,1), power);  % Integrate power over time to get energy consumption

    % minEnergy = min(energy);
    % maxEnergy = max(energy);
    % 
    % % Check if minEnergy and maxEnergy are the same
    % if minEnergy == maxEnergy
    %     % Handle the case where all energy values are the same
    %     normalized_energy(m) = 0.5; % Set to any value between 0 and 1 (e.g., 0.5)
    % else
    %     % Normalize energy values using the modified formula
    %     normalized_energy(m) = (energy(m) - minEnergy) / (maxEnergy - minEnergy);
    % end

    %Calculate normalized energy values
    normalized_energy(m) = (energy(m) - min(energy)) / (max(energy) - min(energy));


    % Combined objective function (weighted sum)
    Objective(m) = W_mse * MSE(m) + W_energy * normalized_energy(m);
    disp(['MSE(m) = ' num2str(W_mse * MSE(m)) ',  energy(m) = ' num2str(W_energy * normalized_energy(m)) ', objective = ' num2str(Objective(m))])

end


% find best value
[Best_performance,location]=min(Objective);
fg=Best_performance;
xg(1)=x(location,1);
xg(2)=x(location,2);


%%
tic; % Start measuring time
for i=1:iteration
    for m=1:particles
        for n=1:Var
            v(m,n)=(w*v(m,n))+(c*rand*(xp(m,n)-x(m,n)))+(c*rand*(xg(n)-x(m,n)));
            x(m,n)=x(m,n)+v(m,n);
        end

        % chek bound
        for n=1:Var
            if x(m,n)<lb(n)
                x(m,n)=lb(n);
            end

            if x(m,n)>ub(n)
                x(m,n)=ub(n);
            end
        end
        %Model param
        kp=x(m,1);
        ki=x(m,2);

        sim('PID_MO.slx');

        error2 = ans.ScopeData1(:,3) - ans.ScopeData1(:,2);

        % Objective Function
        ff2=0;
        sim2=size(ans.ScopeData1(:,2),1);
        for m2=1:sim2
            ff2=ff2+(error2(m2)^2); % Squared error
        end
        MSEp(m)=ff2/sim2;

        % Calculate energy consumption for this particle
        power = abs(ans.ScopeData11(:,2));  % Instantaneous power (assuming power is non-negative)
        energyp(m) = trapz(ans.ScopeData11(:,1), power);  % Integrate power over time to get energy consumption

        % minEnergyp = min(energyp);
        % maxEnergyp = max(energyp);
        % 
        % % Check if minEnergy and maxEnergy are the same
        % if minEnergyp == maxEnergyp
        %     % Handle the case where all energy values are the same
        %     normalized_energyp(m) = 0.5; % Set to any value between 0 and 1 (e.g., 0.5)
        % else
        %     % Normalize energy values using the modified formula
        %     normalized_energyp(m) = (energyp(m) - minEnergyp) / (maxEnergyp - minEnergyp);
        % end

        % Calculate normalized energy values
        normalized_energyp(m) = (energyp(m) - min(energyp)) / (max(energyp) - min(energyp));

        % Combined objective function (weighted sum)
        Objectivep(m) = W_mse * MSEp(m) + W_energy * normalized_energyp(m);


        % Compare Local
        if Objectivep(m) < Objective(m)
            % Update best performance and position
            Objective(m) = Objectivep(m);
            xp(m,1) = x(m,1);
            xp(m,2) = x(m,2);

            % Update energy for the best particle
            best_energy(m) = energyp(m);
        end

    end

    [B_fg,location]=min(Objective);

    %  compare global
    if B_fg<fg
        fg=B_fg;
        xg(location,1)=xp(location,1);
        xg(location,2)=xp(location,2);
    end
    c_cf=c_cf+1;
    best_cf_ac(c_cf)=fg;

    % Calculate time remaining
    time_elapsed = toc;
    iterations_done = i;
    iterations_remaining = iteration - i;
    time_per_iteration = time_elapsed / iterations_done;
    estimated_total_time = time_per_iteration * iteration;

    % Display progress in the Command Window
    disp(['Iteration ' num2str(iterations_done) ' completed out of ' num2str(iteration) ...
        ', Time elapsed: ' num2str(time_elapsed) ' seconds' ...
        ', Estimated time remaining: ' num2str(estimated_total_time - time_elapsed) ' seconds']);
end

%%
Min_Objective=fg;
kp=xg(1);
ki=xg(2);


sim('PID_MO.slx');

%t_cf=1:c_cf;
M = 1:iteration;
M1 = 1:m;
% figure
% plot(M,best_cf_ac,'r--','LineWidth',2),xlabel('iteration'),ylabel('Complete Objective Value')
% legend('Objective Value for PSO-PID')
% title('Multi-Objective Function')

figure
plot(M1,Objective,'r--','LineWidth',2),xlabel('iteration'),ylabel('Complete Objective Value')
legend('Objective Value for PSO-PID')
title('Multi-Objective Function')

figure
plot(M1,Objectivep,'r--','LineWidth',2),xlabel('iteration'),ylabel('Complete Objective Value')
legend('Objective Value for PSO-PID')
title('Multi-Objective Function')

energyp = sort(energyp, 'descend');
figure
plot(M1,energyp,'r--','LineWidth',2),xlabel('Particles'),ylabel('cost function(Energy Consumption)')
legend('Energy Consumption for PSO-PID')
title('Energy')

MSEp = sort(MSEp, 'descend');
figure
plot(M1,MSEp,'r--','LineWidth',2),xlabel('Particles'),ylabel('cost function(MSE)')
legend('MSE')
title('MSE with each iteration')