function restoreFileNames(originalFilePaths, renamedFilePaths)

    for i = 1:length(originalFilePaths)

        src = renamedFilePaths{i};
        dst = originalFilePaths{i};

        % 変更後ファイルが存在するときだけ復元
        if isfile(src)

            % 元ファイルが既にある場合は上書きしない
            if isfile(dst)
                warning(['元ファイル名が既に存在するため復元できませんでした:\n' ...
                         '%s\n'], dst);
                continue
            end

            movefile(src, dst);

            fprintf('復元: %s -> %s\n', src, dst);

        end

    end

end