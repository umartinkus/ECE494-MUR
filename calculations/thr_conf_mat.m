clc

F1 = [0 1 0];
F2 = [0 1 0];
F3 = [-1 0 1];
F4 = [1 0 1];
F5 = [-1 0 1];
F6 = [1 0 1];

r1 = [-138.94	-394.44	-0.10];
r2 = [138.94	-394.44	-0.10];
r3 = [154.27	-150.10	26.72];
r4 = [154.27	150.10	-26.72];
r5 = [-157.47	149.90	-26.72];
r6 = [-154.27	-149.90	26.72];

thrusters = [F1; F2; F3; F4; F5; F6];
mom_arms = [r1; r2; r3; r4; r5; r6];

T = zeros(6);

for i = 1:6
    normed = thrusters(i,:) / norm(thrusters(i,:));
    moment = cross(mom_arms(i,:) / 1000, normed);

    T(:, i) = [normed, moment];
end

disp(T);
T_inv = inv(T);

print_c_array(T_inv, "T_inv")


function print_c_array(matrix, var_name)
    [rows, cols] = size(matrix);
    fprintf('double %s[%d][%d] = {\n', var_name, rows, cols);

    for i = 1:rows
        fprintf('    {');
        for j = 1:cols
            fprintf('%.5f', matrix(i, j)); % Adjust precision as needed
            if j < cols
                fprintf(', ');
            end
        end
        fprintf('}');
        if i < rows
            fprintf(',');
        end
        fprintf('\n');
    end

    fprintf('};\n');
end
