
data = readmatrix('0data.csv');
F = data(:, 1:3)';  
T = data(:, 4:6)';  
A = data(:, 7:9)';  
G = data(:, 10:12)';  
R = data(:, 13:21)';  

R = reshape(R, 3, 3, []); %sette R til 3x3 matrise 

g = 9.81;  
K = size(F, 2); 

F_b = estimate_bias(F, T, R);

m = estimate_mass(F, F_b, R, g);

r = estimate_center_of_mass(T, F_b, R, g, m);

fprintf('Estimated bias:\n');
fprintf('Force: [%.4f, %.4f, %.4f] N\n', F_b(1:3));
fprintf('Torque: [%.4f, %.4f, %.4f] Nm\n', F_b(4:6));
fprintf('Estimated mass: %.4f kg\n', m);
fprintf('Estimated center of mass: [%.4f, %.4f, %.4f] mm\n', r*1000);

% Example of using gravity compensation
R_current = R(:,:,1); 
F_g = gravity_compensation(m, r, R_current, g);
fprintf('Gravity compensation for the first measurement:\n');
fprintf('Force: [%.4f, %.4f, %.4f] N\n', F_g(1:3));
fprintf('Torque: [%.4f, %.4f, %.4f] Nm\n', F_g(4:6));

% Functions

function F_b = estimate_bias(F, T, R)
   
    F_b_force = mean(F, 2); %bruker median/gjennomsnitt av målinger 
    
  
    tau_bx = mean(T(1, :));
    tau_by = mean(T(2, :));
    tau_bz = mean(T(3, :));
    
    F_b = [F_b_force; tau_bx; tau_by; tau_bz];
end

function m = estimate_mass(F, F_b, R, g)
    G = zeros(3*size(F,2), 1);
    F_unbiased = zeros(size(F));
    
    for i = 1:size(F,2)
        g_si = R(:,:,i)' * [0; 0; -g];
        G((3*i-2):(3*i)) = g_si;
        F_unbiased(:,i) = F(:,i) - F_b(1:3);
    end
    
    F_stack = reshape(F_unbiased, [], 1); %minste kvadraters metode, tilsvarende formel som papiret
    m = (G' * G) \ (G' * F_stack);
end

function r = estimate_center_of_mass(T, F_b, R, g, m)
    K = size(T, 2);
    A = zeros(3*K, 3);
    T_unbiased = zeros(size(T));
    
    for i = 1:K
        g_si = R(:,:,i)' * [0; 0; -g];
        A_i = [0, -g_si(3), g_si(2);
               g_si(3), 0, -g_si(1);
               -g_si(2), g_si(1), 0];
        A((3*i-2):(3*i), :) = A_i;
        T_unbiased(:,i) = T(:,i) - F_b(4:6);
    end
    
    T_stack = reshape(T_unbiased, [], 1);   %minste kvadraters metode, tilsvarende formel som papiret
    r = (1/m) * ((A' * A) \ (A' * T_stack)); % \ er invvers i matlab derfor formelen avviker litt fra papiret
end

function F_g = gravity_compensation(m, r, R, g)
    g_I = [0; 0; -g];
    F_g = [m * R' * g_I;
           m * cross(R * r, R' * g_I)];  %vektor f_g med formel rett fra papiret
end