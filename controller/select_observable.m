function [F, code] = select_observable(file)
    %code = cell2mat(append(extract(file, 27), extract(file, 28))); % codeの抽出
    code = extractBefore(extractAfter(file,'ob'),"_");
    switch code
        case '00'; F = @quaternions_all_kyo;
        case '02'; F = @quaternions_all_02;
        case '23'; F = @quaternions_all_23;
        case '26'; F = @quaternions_all_kyo;
        case '45'; F = @quaternions_all_kyo_45;
        case '71'; F = @quaternions_all_kyo_45;   
        otherwise; F = @quaternions_all;
    end
end