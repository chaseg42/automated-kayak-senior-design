% Credit: https://www.mathworks.com/matlabcentral/answers/1411387-need-to-do-a-8-bit-fletcher-algorithm-checksum
% Jack:
% 1/18/2025: Modified to return a hexadeicmal value
function [CK_A, CK_B] = fletcher(Buffer)
    CK_A = uint8(0);
    CK_B = uint8(0);
    N = length(Buffer);
    for I = 1:N
        % use mod with 2^8 (which is 256)
        CK_A = mod(CK_A + uint8(Buffer(I)), 256);
        CK_B = mod(CK_B + CK_A, 256);
    end

    CK_A = dec2hex(CK_A);
    CK_B = dec2hex(CK_B);
end
